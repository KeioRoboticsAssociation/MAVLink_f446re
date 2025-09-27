#include "motor_factory.hpp"
#include "../../config/robot_config.hpp"

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Motors {

// ConcreteMotorFactory implementation
std::unique_ptr<IMotorController<Config::ServoConfig>> ConcreteMotorFactory::createServo(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::SERVO) {
        return nullptr;
    }

    // Map timer ID to HAL::TimerID enum
    HAL::TimerID timerId = static_cast<HAL::TimerID>(instance->timer_id);

    return std::make_unique<ServoMotorController>(id, hwManager_, timerId, instance->channel);
}

std::unique_ptr<IMotorController<Config::DCMotorConfig>> ConcreteMotorFactory::createDCMotor(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::DC_MOTOR) {
        return nullptr;
    }

    // Map timer ID to HAL::TimerID enum
    HAL::TimerID timerId = static_cast<HAL::TimerID>(instance->timer_id);

    return std::make_unique<DCMotorController>(id, hwManager_, timerId, instance->channel);
}

std::unique_ptr<IMotorController<Config::RoboMasterConfig>> ConcreteMotorFactory::createRoboMaster(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::ROBOMASTER) {
        return nullptr;
    }

    return std::make_unique<RoboMasterMotorController>(id, hwManager_);
}

const Config::MotorInstance* ConcreteMotorFactory::findMotorInstance(uint8_t id) {
    for (const auto& instance : Config::MOTOR_INSTANCES) {
        if (instance.id == id) {
            return &instance;
        }
    }
    return nullptr;
}

// ServoMotorController implementation
ServoMotorController::ServoMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel)
    : id_(id), hwManager_(hwManager), timerId_(timerId), channel_(channel) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

Config::Result<void> ServoMotorController::initialize(const Config::ServoConfig& config) {
    config_ = config;

    // Get servo-specific configuration
    const auto* servoConfig = Config::ConfigAccessor::getServoConfig(id_);
    if (!servoConfig) {
        state_.status = Config::ErrorCode::CONFIG_ERROR;
        return Config::Result<void>(state_.status);
    }

    // Initialize hardware timer
    auto timerResult = hwManager_->startPWM(timerId_, channel_);
    if (!timerResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        return Config::Result<void>(state_.status);
    }

    // Set initial position to startup angle
    state_.targetPosition = servoConfig->startup_angle_deg;
    state_.currentPosition = servoConfig->startup_angle_deg;
    state_.enabled = !servoConfig->start_disabled;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();

    resetWatchdog();
    updateState();

    return Config::Result<void>();
}

Config::Result<void> ServoMotorController::update(float deltaTime) {
    checkWatchdog();
    updateState();

    if (state_.status != Config::ErrorCode::OK) {
        return Config::Result<void>(state_.status);
    }

    // Update position based on velocity constraints
    const auto* config = Config::ConfigAccessor::getServoConfig(id_);
    if (!config) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    float positionError = state_.targetPosition - state_.currentPosition;
    float maxVelocity = config->max_velocity_deg_per_s;

    // Apply velocity limiting
    float velocityCommand = positionError;
    if (velocityCommand > maxVelocity) {
        velocityCommand = maxVelocity;
    } else if (velocityCommand < -maxVelocity) {
        velocityCommand = -maxVelocity;
    }

    // Update position
    state_.currentPosition += velocityCommand * deltaTime;
    state_.currentVelocity = velocityCommand;

    // Convert position to PWM and update hardware
    float pulseWidth = degreesToPulseWidth(state_.currentPosition);

    // Calculate PWM value (assuming 20ms period = 50Hz)
    uint32_t pwmValue = static_cast<uint32_t>((pulseWidth / 20000.0f) * 1000); // Scale to timer period

    auto pwmResult = hwManager_->setPWMDutyCycle(timerId_, channel_, pwmValue);
    if (!pwmResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        reportError(Config::ErrorCode::HARDWARE_ERROR);
        return Config::Result<void>(state_.status);
    }

    return Config::Result<void>();
}

Config::Result<void> ServoMotorController::setCommand(const MotorCommand& cmd) {
    if (cmd.motorId != id_) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    resetWatchdog();

    switch (cmd.mode) {
        case ControlMode::POSITION:
            state_.targetPosition = constrainValue(cmd.targetValue, -90.0f, 90.0f);
            break;
        case ControlMode::DISABLED:
            state_.enabled = false;
            break;
        default:
            return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    state_.enabled = cmd.enable;
    return Config::Result<void>();
}

Config::Result<void> ServoMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    if (!enabled) {
        // Set to neutral position when disabled
        state_.targetPosition = 0.0f;
    }
    updateState();
    return Config::Result<void>();
}

void ServoMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;
    // Set PWM to neutral
    auto pwmResult = hwManager_->setPWMDutyCycle(timerId_, channel_, 1500); // 1.5ms pulse
    updateState();
}

void ServoMotorController::resetWatchdog() {
    lastWatchdogReset_ = HAL_GetTick();
    watchdogExpired_ = false;
}

Config::Result<void> ServoMotorController::runSelfTest() {
    // Basic self-test: try to set PWM values
    auto testResult = hwManager_->setPWMDutyCycle(timerId_, channel_, 1500);
    if (!testResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }
    return Config::Result<void>();
}

Config::Result<void> ServoMotorController::updateConfig(const Config::ServoConfig& config) {
    config_ = config;
    return Config::Result<void>();
}

float ServoMotorController::degreesToPulseWidth(float degrees) {
    const auto* config = Config::ConfigAccessor::getServoConfig(id_);
    if (!config) {
        return config->pulse_neutral_us;
    }

    // Map degrees to pulse width
    float normalizedAngle = degrees / (config->max_angle - config->min_angle);
    float pulseRange = config->pulse_max_us - config->pulse_min_us;
    return config->pulse_min_us + (normalizedAngle + 0.5f) * pulseRange;
}

float ServoMotorController::pulseWidthToDegrees(float pulseWidth) {
    const auto* config = Config::ConfigAccessor::getServoConfig(id_);
    if (!config) {
        return 0.0f;
    }

    float pulseRange = config->pulse_max_us - config->pulse_min_us;
    float normalizedPulse = (pulseWidth - config->pulse_min_us) / pulseRange - 0.5f;
    return normalizedPulse * (config->max_angle - config->min_angle);
}

void ServoMotorController::checkWatchdog() {
    const auto* config = Config::ConfigAccessor::getServoConfig(id_);
    if (config && (HAL_GetTick() - lastWatchdogReset_) > config->watchdog_timeout_ms) {
        if (!watchdogExpired_) {
            watchdogExpired_ = true;
            emergencyStop();
            reportError(Config::ErrorCode::TIMEOUT);
        }
    }
}

void ServoMotorController::reportError(MotorStatus error) {
    state_.errorCount++;
    if (errorCallback_) {
        errorCallback_(id_, error);
    }
}

void ServoMotorController::updateState() {
    state_.lastUpdateTime = HAL_GetTick();
    if (stateCallback_) {
        stateCallback_(id_, state_);
    }
}

// DCMotorController implementation (simplified)
DCMotorController::DCMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel)
    : id_(id), hwManager_(hwManager), timerId_(timerId), channel_(channel) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

Config::Result<void> DCMotorController::initialize(const Config::DCMotorConfig& config) {
    config_ = config;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();
    resetWatchdog();
    return Config::Result<void>();
}

Config::Result<void> DCMotorController::update(float deltaTime) {
    checkWatchdog();
    updateState();
    // TODO: Implement PID control and encoder feedback
    return Config::Result<void>();
}

Config::Result<void> DCMotorController::setCommand(const MotorCommand& cmd) {
    resetWatchdog();
    state_.targetPosition = cmd.targetValue;
    state_.enabled = cmd.enable;
    return Config::Result<void>();
}

Config::Result<void> DCMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    return Config::Result<void>();
}

void DCMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;
    updateState();
}

void DCMotorController::resetWatchdog() {
    lastWatchdogReset_ = HAL_GetTick();
    watchdogExpired_ = false;
}

Config::Result<void> DCMotorController::runSelfTest() {
    return Config::Result<void>();
}

Config::Result<void> DCMotorController::updateConfig(const Config::DCMotorConfig& config) {
    config_ = config;
    return Config::Result<void>();
}

float DCMotorController::calculatePID(float error, float& integral, float& lastError, float kp, float ki, float kd,
                                     float maxIntegral, float maxOutput, float deltaTime) {
    integral += error * deltaTime;
    integral = constrainValue(integral, -maxIntegral, maxIntegral);

    float derivative = (error - lastError) / deltaTime;
    lastError = error;

    float output = kp * error + ki * integral + kd * derivative;
    return constrainValue(output, -maxOutput, maxOutput);
}

void DCMotorController::checkWatchdog() {
    const auto* config = Config::ConfigAccessor::getDCMotorConfig(id_);
    if (config && (HAL_GetTick() - lastWatchdogReset_) > config->watchdog_timeout_ms) {
        if (!watchdogExpired_) {
            watchdogExpired_ = true;
            emergencyStop();
            reportError(Config::ErrorCode::TIMEOUT);
        }
    }
}

void DCMotorController::reportError(MotorStatus error) {
    state_.errorCount++;
    if (errorCallback_) {
        errorCallback_(id_, error);
    }
}

void DCMotorController::updateState() {
    state_.lastUpdateTime = HAL_GetTick();
    if (stateCallback_) {
        stateCallback_(id_, state_);
    }
}

// RoboMasterMotorController implementation (placeholder)
RoboMasterMotorController::RoboMasterMotorController(uint8_t id, HAL::HardwareManager* hwManager)
    : id_(id), hwManager_(hwManager) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

Config::Result<void> RoboMasterMotorController::initialize(const Config::RoboMasterConfig& config) {
    config_ = config;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();
    return Config::Result<void>();
}

Config::Result<void> RoboMasterMotorController::update(float deltaTime) {
    updateState();
    // TODO: Implement CAN communication and control
    (void)deltaTime; // Suppress unused warning
    return Config::Result<void>();
}

Config::Result<void> RoboMasterMotorController::setCommand(const MotorCommand& cmd) {
    state_.targetPosition = cmd.targetValue;
    state_.enabled = cmd.enable;
    return Config::Result<void>();
}

Config::Result<void> RoboMasterMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    return Config::Result<void>();
}

void RoboMasterMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;
    updateState();
}

void RoboMasterMotorController::resetWatchdog() {
    // TODO: Implement watchdog for RoboMaster
}

Config::Result<void> RoboMasterMotorController::runSelfTest() {
    return Config::Result<void>();
}

Config::Result<void> RoboMasterMotorController::updateConfig(const Config::RoboMasterConfig& config) {
    config_ = config;
    return Config::Result<void>();
}

void RoboMasterMotorController::reportError(MotorStatus error) {
    state_.errorCount++;
    if (errorCallback_) {
        errorCallback_(id_, error);
    }
}

void RoboMasterMotorController::updateState() {
    state_.lastUpdateTime = HAL_GetTick();
    if (stateCallback_) {
        stateCallback_(id_, state_);
    }
}

} // namespace Motors