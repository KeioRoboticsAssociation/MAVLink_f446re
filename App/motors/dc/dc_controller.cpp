#include "dc_controller.hpp"
#include <cmath>

namespace Motors {
namespace DC {

DCMotorController::DCMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel)
    : id_(id), hwManager_(hwManager), timerId_(timerId), channel_(channel),
      lastWatchdogReset_(0), watchdogExpired_(false), pidIntegral_(0.0f), pidLastError_(0.0f) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

Config::Result<void> DCMotorController::initialize(const Config::DCMotorConfig& config) {
    config_ = config;

    // Get DC motor-specific configuration
    const auto* dcConfig = Config::ConfigAccessor::getDCMotorConfig(id_);
    if (!dcConfig) {
        state_.status = Config::ErrorCode::CONFIG_ERROR;
        return Config::Result<void>(state_.status);
    }

    // Initialize hardware timer for PWM
    auto timerResult = hwManager_->startPWM(timerId_, channel_);
    if (!timerResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        return Config::Result<void>(state_.status);
    }

    // Initialize state
    state_.targetPosition = 0.0f;
    state_.currentPosition = 0.0f;
    state_.enabled = true;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();

    // Reset PID
    pidIntegral_ = 0.0f;
    pidLastError_ = 0.0f;

    resetWatchdog();
    updateState();

    return Config::Result<void>();
}

Config::Result<void> DCMotorController::update(float deltaTime) {
    checkWatchdog();
    updateState();

    if (state_.status != Config::ErrorCode::OK) {
        return Config::Result<void>(state_.status);
    }

    if (!state_.enabled) {
        // Motor disabled, set PWM to neutral
        auto pwmResult = hwManager_->setPWMDutyCycle(timerId_, channel_, 0);
        if (!pwmResult) {
            state_.status = Config::ErrorCode::HARDWARE_ERROR;
            reportError(Config::ErrorCode::HARDWARE_ERROR);
            return Config::Result<void>(state_.status);
        }
        return Config::Result<void>();
    }

    // Get DC motor configuration
    const auto* config = Config::ConfigAccessor::getDCMotorConfig(id_);
    if (!config) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    // TODO: Read encoder feedback for actual position
    // For now, we'll assume open-loop control
    float positionError = state_.targetPosition - state_.currentPosition;

    // Calculate PID output
    float pidOutput = calculatePID(positionError, pidIntegral_, pidLastError_,
                                 config->speed_kp, config->speed_ki, config->speed_kd,
                                 config->speed_max_integral, config->speed_max_output, deltaTime);

    // Convert PID output to PWM duty cycle (0-1000 range)
    uint32_t pwmValue = static_cast<uint32_t>(constrainValue(std::abs(pidOutput), 0.0f, 1000.0f));

    // Set direction based on sign of PID output
    // TODO: Implement direction control via additional GPIO or H-bridge control

    auto pwmResult = hwManager_->setPWMDutyCycle(timerId_, channel_, pwmValue);
    if (!pwmResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        reportError(Config::ErrorCode::HARDWARE_ERROR);
        return Config::Result<void>(state_.status);
    }

    // Update velocity estimate (simple numerical differentiation)
    static float lastPosition = state_.currentPosition;
    state_.currentVelocity = (state_.currentPosition - lastPosition) / deltaTime;
    lastPosition = state_.currentPosition;

    return Config::Result<void>();
}

Config::Result<void> DCMotorController::setCommand(const MotorCommand& cmd) {
    if (cmd.motorId != id_) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    resetWatchdog();

    switch (cmd.mode) {
        case ControlMode::POSITION:
            state_.targetPosition = cmd.targetValue;
            break;
        case ControlMode::VELOCITY:
            // TODO: Implement velocity control mode
            state_.targetVelocity = cmd.targetValue;
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

Config::Result<void> DCMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    if (!enabled) {
        // Reset PID when disabling
        pidIntegral_ = 0.0f;
        pidLastError_ = 0.0f;
    }
    updateState();
    return Config::Result<void>();
}

void DCMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;

    // Set PWM to zero immediately
    hwManager_->setPWMDutyCycle(timerId_, channel_, 0);

    // Reset PID
    pidIntegral_ = 0.0f;
    pidLastError_ = 0.0f;

    updateState();
}

void DCMotorController::resetWatchdog() {
    lastWatchdogReset_ = HAL_GetTick();
    watchdogExpired_ = false;
}

Config::Result<void> DCMotorController::runSelfTest() {
    // Basic self-test: try to set PWM values
    auto testResult = hwManager_->setPWMDutyCycle(timerId_, channel_, 100);
    if (!testResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    // Return to neutral
    hwManager_->setPWMDutyCycle(timerId_, channel_, 0);
    return Config::Result<void>();
}

Config::Result<void> DCMotorController::updateConfig(const Config::DCMotorConfig& config) {
    config_ = config;
    // Reset PID when config changes
    pidIntegral_ = 0.0f;
    pidLastError_ = 0.0f;
    return Config::Result<void>();
}

float DCMotorController::calculatePID(float error, float& integral, float& lastError,
                                    float kp, float ki, float kd,
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

} // namespace DC
} // namespace Motors