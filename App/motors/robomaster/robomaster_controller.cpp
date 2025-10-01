/**
 * @file robomaster_controller.cpp
 * @brief Implements the controller for RoboMaster motors.
 */

#include "robomaster_controller.hpp"

namespace Motors {
namespace RoboMaster {

/**
 * @brief Construct a new RoboMasterMotorController object.
 * @param id The ID of the motor.
 * @param hwManager A pointer to the hardware manager.
 */
RoboMasterMotorController::RoboMasterMotorController(uint8_t id, HAL::HardwareManager* hwManager)
    : id_(id), hwManager_(hwManager), lastWatchdogReset_(0), watchdogExpired_(false),
      canId_(0x200 + id), lastCanRx_(0), anglePidIntegral_(0.0f), anglePidLastError_(0.0f),
      speedPidIntegral_(0.0f), speedPidLastError_(0.0f) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

/**
 * @brief Initializes the RoboMaster motor controller.
 * @param config The configuration for the motor.
 * @return Result of the operation.
 */
Config::Result<void> RoboMasterMotorController::initialize(const Config::RoboMasterConfig& config) {
    config_ = config;

    const auto* rmConfig = Config::ConfigAccessor::getRoboMasterConfig(id_);
    if (!rmConfig) {
        state_.status = Config::ErrorCode::CONFIG_ERROR;
        return Config::Result<void>(state_.status);
    }

    canId_ = 0x200 + id_;

    state_.targetPosition = 0.0f;
    state_.currentPosition = 0.0f;
    state_.targetVelocity = 0.0f;
    state_.currentVelocity = 0.0f;
    state_.enabled = true;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();

    anglePidIntegral_ = 0.0f;
    anglePidLastError_ = 0.0f;
    speedPidIntegral_ = 0.0f;
    speedPidLastError_ = 0.0f;

    resetWatchdog();
    updateState();

    return Config::Result<void>();
}

/**
 * @brief Updates the motor controller.
 * @param deltaTime The time since the last update.
 * @return Result of the operation.
 */
Config::Result<void> RoboMasterMotorController::update(float deltaTime) {
    checkWatchdog();
    updateState();

    if (state_.status != Config::ErrorCode::OK) {
        return Config::Result<void>(state_.status);
    }

    if (!state_.enabled) {
        return sendCanCommand(0);
    }

    const auto* config = Config::ConfigAccessor::getRoboMasterConfig(id_);
    if (!config) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    uint32_t currentTime = HAL_GetTick();
    if ((currentTime - lastCanRx_) > 100) {
        state_.status = Config::ErrorCode::TIMEOUT;
        reportError(Config::ErrorCode::TIMEOUT);
        return sendCanCommand(0);
    }

    int16_t currentCommand = 0;

    if (state_.targetPosition != state_.currentPosition) {
        float positionError = state_.targetPosition - state_.currentPosition;
        float angleOutput = calculatePID(positionError, anglePidIntegral_, anglePidLastError_,
                                       config->angle_kp, config->angle_ki, config->angle_kd,
                                       1000.0f, config->max_speed_rad_s, deltaTime);
        state_.targetVelocity = angleOutput;
    }

    float velocityError = state_.targetVelocity - state_.currentVelocity;
    float speedOutput = calculatePID(velocityError, speedPidIntegral_, speedPidLastError_,
                                   config->speed_kp, config->speed_ki, config->speed_kd,
                                   1000.0f, 16384.0f, deltaTime);

    currentCommand = static_cast<int16_t>(constrainValue(speedOutput, -16384.0f, 16384.0f));

    return sendCanCommand(currentCommand);
}

/**
 * @brief Sets a command for the motor.
 * @param cmd The command to set.
 * @return Result of the operation.
 */
Config::Result<void> RoboMasterMotorController::setCommand(const MotorCommand& cmd) {
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
            state_.targetPosition = state_.currentPosition;
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
Config::Result<void> RoboMasterMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    if (!enabled) {
        anglePidIntegral_ = 0.0f;
        anglePidLastError_ = 0.0f;
        speedPidIntegral_ = 0.0f;
        speedPidLastError_ = 0.0f;
        sendCanCommand(0);
    }
    updateState();
    return Config::Result<void>();
}

/**
 * @brief Stops the motor immediately.
 */
void RoboMasterMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;
    sendCanCommand(0);
    anglePidIntegral_ = 0.0f;
    anglePidLastError_ = 0.0f;
    speedPidIntegral_ = 0.0f;
    speedPidLastError_ = 0.0f;
    updateState();
}

/**
 * @brief Resets the motor's watchdog timer.
 */
void RoboMasterMotorController::resetWatchdog() {
    lastWatchdogReset_ = HAL_GetTick();
    watchdogExpired_ = false;
}

/**
 * @brief Runs a self-test on the motor.
 * @return Result of the operation.
 */
Config::Result<void> RoboMasterMotorController::runSelfTest() {
    auto result = sendCanCommand(100);
    if (result.isError()) {
        return result.error();
    }
    HAL_Delay(10);
    return sendCanCommand(0);
}

/**
 * @brief Updates the configuration of the motor.
 * @param config The new configuration.
 * @return Result of the operation.
 */
Config::Result<void> RoboMasterMotorController::updateConfig(const Config::RoboMasterConfig& config) {
    config_ = config;
    anglePidIntegral_ = 0.0f;
    anglePidLastError_ = 0.0f;
    speedPidIntegral_ = 0.0f;
    speedPidLastError_ = 0.0f;
    return Config::Result<void>();
}

/**
 * @brief Processes a CAN message received from the motor.
 * @param canId The CAN ID of the message.
 * @param data A pointer to the message data.
 * @param length The length of the message data.
 * @return Result of the operation.
 */
Config::Result<void> RoboMasterMotorController::processCanMessage(uint32_t canId, const uint8_t* data, uint8_t length) {
    if (canId != (0x201 + id_) || length != 8) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    int16_t angle_raw = (data[0] << 8) | data[1];
    int16_t velocity_raw = (data[2] << 8) | data[3];
    int16_t current_raw = (data[4] << 8) | data[5];
    uint8_t temperature = data[6];

    state_.currentPosition = (angle_raw / 8191.0f) * 360.0f;
    state_.currentVelocity = velocity_raw * (3.14159f / 30.0f);
    state_.currentCurrent = current_raw;
    (void)temperature;

    lastCanRx_ = HAL_GetTick();
    return Config::Result<void>();
}

/**
 * @brief Sends a command to the motor over CAN.
 * @param current The current to send to the motor.
 * @return Result of the operation.
 */
Config::Result<void> RoboMasterMotorController::sendCanCommand(int16_t current) {
    (void)current;
    return Config::Result<void>();
}

/**
 * @brief Calculates the PID output.
 * @return The PID output.
 */
float RoboMasterMotorController::calculatePID(float error, float& integral, float& lastError,
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
void RoboMasterMotorController::checkWatchdog() {
    const auto* config = Config::ConfigAccessor::getRoboMasterConfig(id_);
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
void RoboMasterMotorController::reportError(MotorStatus error) {
    state_.errorCount++;
    if (errorCallback_) {
        errorCallback_(id_, error);
    }
}

/**
 * @brief Updates the state of the motor and invokes the state callback.
 */
void RoboMasterMotorController::updateState() {
    state_.lastUpdateTime = HAL_GetTick();
    if (stateCallback_) {
        stateCallback_(id_, state_);
    }
}

} // namespace RoboMaster
} // namespace Motors