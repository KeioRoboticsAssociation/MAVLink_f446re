/**
 * @file servo_controller.cpp
 * @brief Implements the controller for servo motors.
 */

#include "servo_controller.hpp"

namespace Motors {
namespace Servo {

/**
 * @brief Construct a new ServoMotorController object.
 * @param id The ID of the motor.
 * @param hwManager A pointer to the hardware manager.
 * @param timerId The ID of the timer to use for PWM.
 * @param channel The timer channel.
 */
ServoMotorController::ServoMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel)
    : id_(id), hwManager_(hwManager), timerId_(timerId), channel_(channel),
      lastWatchdogReset_(0), watchdogExpired_(false) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

/**
 * @brief Initializes the servo motor controller.
 * @param config The configuration for the servo motor.
 * @return Result of the operation.
 */
Config::Result<void> ServoMotorController::initialize(const Config::ServoConfig& config) {
    config_ = config;

    const auto* servoConfig = Config::ConfigAccessor::getServoConfig(id_);
    if (!servoConfig) {
        state_.status = Config::ErrorCode::CONFIG_ERROR;
        return Config::Result<void>(state_.status);
    }

    auto timerResult = hwManager_->startPWM(timerId_, channel_);
    if (!timerResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        return Config::Result<void>(state_.status);
    }

    state_.targetPosition = servoConfig->startup_angle_deg;
    state_.currentPosition = servoConfig->startup_angle_deg;
    state_.enabled = !servoConfig->start_disabled;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();

    resetWatchdog();
    updateState();

    return Config::Result<void>();
}

/**
 * @brief Updates the servo motor controller.
 * @param deltaTime The time since the last update.
 * @return Result of the operation.
 */
Config::Result<void> ServoMotorController::update(float deltaTime) {
    checkWatchdog();
    updateState();

    if (state_.status != Config::ErrorCode::OK) {
        return Config::Result<void>(state_.status);
    }

    const auto* config = Config::ConfigAccessor::getServoConfig(id_);
    if (!config) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    float positionError = state_.targetPosition - state_.currentPosition;
    float maxVelocity = config->max_velocity_deg_per_s;
    float velocityCommand = constrainValue(positionError, -maxVelocity, maxVelocity);

    state_.currentPosition += velocityCommand * deltaTime;
    state_.currentVelocity = velocityCommand;

    float pulseWidth = degreesToPulseWidth(state_.currentPosition);
    uint32_t pwmValue = static_cast<uint32_t>((pulseWidth / 20000.0f) * 1000);

    auto pwmResult = hwManager_->setPWMDutyCycle(timerId_, channel_, pwmValue);
    if (!pwmResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        reportError(Config::ErrorCode::HARDWARE_ERROR);
        return Config::Result<void>(state_.status);
    }

    return Config::Result<void>();
}

/**
 * @brief Sets a command for the motor.
 * @param cmd The command to set.
 * @return Result of the operation.
 */
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

/**
 * @brief Enables or disables the motor.
 * @param enabled true to enable, false to disable.
 * @return Result of the operation.
 */
Config::Result<void> ServoMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    if (!enabled) {
        state_.targetPosition = 0.0f;
    }
    updateState();
    return Config::Result<void>();
}

/**
 * @brief Stops the motor immediately.
 */
void ServoMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;
    hwManager_->setPWMDutyCycle(timerId_, channel_, 1500);
    updateState();
}

/**
 * @brief Resets the motor's watchdog timer.
 */
void ServoMotorController::resetWatchdog() {
    lastWatchdogReset_ = HAL_GetTick();
    watchdogExpired_ = false;
}

/**
 * @brief Runs a self-test on the motor.
 * @return Result of the operation.
 */
Config::Result<void> ServoMotorController::runSelfTest() {
    auto testResult = hwManager_->setPWMDutyCycle(timerId_, channel_, 1500);
    if (!testResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }
    return Config::Result<void>();
}

/**
 * @brief Updates the configuration of the motor.
 * @param config The new configuration.
 * @return Result of the operation.
 */
Config::Result<void> ServoMotorController::updateConfig(const Config::ServoConfig& config) {
    config_ = config;
    return Config::Result<void>();
}

/**
 * @brief Converts an angle in degrees to a PWM pulse width.
 * @param degrees The angle in degrees.
 * @return The PWM pulse width in microseconds.
 */
float ServoMotorController::degreesToPulseWidth(float degrees) {
    const auto* config = Config::ConfigAccessor::getServoConfig(id_);
    if (!config) {
        return config->pulse_neutral_us;
    }
    float normalizedAngle = (degrees - config->min_angle) / (config->max_angle - config->min_angle);
    return config->pulse_min_us + normalizedAngle * (config->pulse_max_us - config->pulse_min_us);
}

/**
 * @brief Converts a PWM pulse width to an angle in degrees.
 * @param pulseWidth The PWM pulse width in microseconds.
 * @return The angle in degrees.
 */
float ServoMotorController::pulseWidthToDegrees(float pulseWidth) {
    const auto* config = Config::ConfigAccessor::getServoConfig(id_);
    if (!config) {
        return 0.0f;
    }
    float normalizedPulse = (pulseWidth - config->pulse_min_us) / (config->pulse_max_us - config->pulse_min_us);
    return config->min_angle + normalizedPulse * (config->max_angle - config->min_angle);
}

/**
 * @brief Checks the watchdog timer and stops the motor if it has expired.
 */
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

/**
 * @brief Reports an error.
 * @param error The error to report.
 */
void ServoMotorController::reportError(MotorStatus error) {
    state_.errorCount++;
    if (errorCallback_) {
        errorCallback_(id_, error);
    }
}

/**
 * @brief Updates the state of the motor and invokes the state callback.
 */
void ServoMotorController::updateState() {
    state_.lastUpdateTime = HAL_GetTick();
    if (stateCallback_) {
        stateCallback_(id_, state_);
    }
}

} // namespace Servo
} // namespace Motors