/**
 * @file dc_controller.cpp
 * @brief Implements the controller for DC motors.
 */

#include "dc_controller.hpp"
#include <cmath>

namespace Motors {
namespace DC {

/**
 * @brief Construct a new DCMotorController object.
 * @param id The ID of the motor.
 * @param hwManager A pointer to the hardware manager.
 * @param timerId The ID of the timer to use for PWM.
 * @param channel The timer channel.
 */
DCMotorController::DCMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel)
    : id_(id), hwManager_(hwManager), timerId_(timerId), channel_(channel),
      lastWatchdogReset_(0), watchdogExpired_(false), pidIntegral_(0.0f), pidLastError_(0.0f) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

/**
 * @brief Initializes the DC motor controller.
 * @param config The configuration for the DC motor.
 * @return Result of the operation.
 */
Config::Result<void> DCMotorController::initialize(const Config::DCMotorConfig& config) {
    config_ = config;

    const auto* dcConfig = Config::ConfigAccessor::getDCMotorConfig(id_);
    if (!dcConfig) {
        state_.status = Config::ErrorCode::CONFIG_ERROR;
        return Config::Result<void>(state_.status);
    }

    auto timerResult = hwManager_->startPWM(timerId_, channel_);
    if (!timerResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        return Config::Result<void>(state_.status);
    }

    state_.targetPosition = 0.0f;
    state_.currentPosition = 0.0f;
    state_.enabled = true;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();

    pidIntegral_ = 0.0f;
    pidLastError_ = 0.0f;

    resetWatchdog();
    updateState();

    return Config::Result<void>();
}

/**
 * @brief Updates the DC motor controller.
 * @param deltaTime The time since the last update.
 * @return Result of the operation.
 */
Config::Result<void> DCMotorController::update(float deltaTime) {
    checkWatchdog();
    updateState();

    if (state_.status != Config::ErrorCode::OK) {
        return Config::Result<void>(state_.status);
    }

    if (!state_.enabled) {
        auto pwmResult = hwManager_->setPWMDutyCycle(timerId_, channel_, 0);
        if (!pwmResult) {
            state_.status = Config::ErrorCode::HARDWARE_ERROR;
            reportError(Config::ErrorCode::HARDWARE_ERROR);
            return Config::Result<void>(state_.status);
        }
        return Config::Result<void>();
    }

    const auto* config = Config::ConfigAccessor::getDCMotorConfig(id_);
    if (!config) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    float positionError = state_.targetPosition - state_.currentPosition;

    float pidOutput = calculatePID(positionError, pidIntegral_, pidLastError_,
                                 config->speed_kp, config->speed_ki, config->speed_kd,
                                 config->speed_max_integral, config->speed_max_output, deltaTime);

    uint32_t pwmValue = static_cast<uint32_t>(constrainValue(std::abs(pidOutput), 0.0f, 1000.0f));

    auto pwmResult = hwManager_->setPWMDutyCycle(timerId_, channel_, pwmValue);
    if (!pwmResult) {
        state_.status = Config::ErrorCode::HARDWARE_ERROR;
        reportError(Config::ErrorCode::HARDWARE_ERROR);
        return Config::Result<void>(state_.status);
    }

    static float lastPosition = state_.currentPosition;
    state_.currentVelocity = (state_.currentPosition - lastPosition) / deltaTime;
    lastPosition = state_.currentPosition;

    return Config::Result<void>();
}

/**
 * @brief Sets a command for the motor.
 * @param cmd The command to set.
 * @return Result of the operation.
 */
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

/**
 * @brief Enables or disables the motor.
 * @param enabled true to enable, false to disable.
 * @return Result of the operation.
 */
Config::Result<void> DCMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    if (!enabled) {
        pidIntegral_ = 0.0f;
        pidLastError_ = 0.0f;
    }
    updateState();
    return Config::Result<void>();
}

/**
 * @brief Stops the motor immediately.
 */
void DCMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;
    hwManager_->setPWMDutyCycle(timerId_, channel_, 0);
    pidIntegral_ = 0.0f;
    pidLastError_ = 0.0f;
    updateState();
}

/**
 * @brief Resets the motor's watchdog timer.
 */
void DCMotorController::resetWatchdog() {
    lastWatchdogReset_ = HAL_GetTick();
    watchdogExpired_ = false;
}

/**
 * @brief Runs a self-test on the motor.
 * @return Result of the operation.
 */
Config::Result<void> DCMotorController::runSelfTest() {
    auto testResult = hwManager_->setPWMDutyCycle(timerId_, channel_, 100);
    if (!testResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }
    hwManager_->setPWMDutyCycle(timerId_, channel_, 0);
    return Config::Result<void>();
}

/**
 * @brief Updates the configuration of the motor.
 * @param config The new configuration.
 * @return Result of the operation.
 */
Config::Result<void> DCMotorController::updateConfig(const Config::DCMotorConfig& config) {
    config_ = config;
    pidIntegral_ = 0.0f;
    pidLastError_ = 0.0f;
    return Config::Result<void>();
}

/**
 * @brief Calculates the PID output.
 * @return The PID output.
 */
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

/**
 * @brief Checks the watchdog timer and stops the motor if it has expired.
 */
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

/**
 * @brief Reports an error.
 * @param error The error to report.
 */
void DCMotorController::reportError(MotorStatus error) {
    state_.errorCount++;
    if (errorCallback_) {
        errorCallback_(id_, error);
    }
}

/**
 * @brief Updates the state of the motor and invokes the state callback.
 */
void DCMotorController::updateState() {
    state_.lastUpdateTime = HAL_GetTick();
    if (stateCallback_) {
        stateCallback_(id_, state_);
    }
}

} // namespace DC
} // namespace Motors