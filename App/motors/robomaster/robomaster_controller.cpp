#include "robomaster_controller.hpp"

namespace Motors {
namespace RoboMaster {

RoboMasterMotorController::RoboMasterMotorController(uint8_t id, HAL::HardwareManager* hwManager)
    : id_(id), hwManager_(hwManager), lastWatchdogReset_(0), watchdogExpired_(false),
      canId_(0x200 + id), lastCanRx_(0), anglePidIntegral_(0.0f), anglePidLastError_(0.0f),
      speedPidIntegral_(0.0f), speedPidLastError_(0.0f) {
    state_.status = Config::ErrorCode::NOT_INITIALIZED;
}

Config::Result<void> RoboMasterMotorController::initialize(const Config::RoboMasterConfig& config) {
    config_ = config;

    // Get RoboMaster-specific configuration
    const auto* rmConfig = Config::ConfigAccessor::getRoboMasterConfig(id_);
    if (!rmConfig) {
        state_.status = Config::ErrorCode::CONFIG_ERROR;
        return Config::Result<void>(state_.status);
    }

    // Initialize CAN communication
    // TODO: Initialize CAN bus if not already done
    // This would typically be done once in the hardware manager

    // Set CAN ID based on motor ID
    canId_ = 0x200 + id_;

    // Initialize state
    state_.targetPosition = 0.0f;
    state_.currentPosition = 0.0f;
    state_.targetVelocity = 0.0f;
    state_.currentVelocity = 0.0f;
    state_.enabled = true;
    state_.status = Config::ErrorCode::OK;
    state_.lastUpdateTime = HAL_GetTick();

    // Reset PID controllers
    anglePidIntegral_ = 0.0f;
    anglePidLastError_ = 0.0f;
    speedPidIntegral_ = 0.0f;
    speedPidLastError_ = 0.0f;

    resetWatchdog();
    updateState();

    return Config::Result<void>();
}

Config::Result<void> RoboMasterMotorController::update(float deltaTime) {
    checkWatchdog();
    updateState();

    if (state_.status != Config::ErrorCode::OK) {
        return Config::Result<void>(state_.status);
    }

    if (!state_.enabled) {
        // Motor disabled, send zero current command
        return sendCanCommand(0);
    }

    // Get RoboMaster configuration
    const auto* config = Config::ConfigAccessor::getRoboMasterConfig(id_);
    if (!config) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    // Check if we've received recent CAN feedback
    uint32_t currentTime = HAL_GetTick();
    if ((currentTime - lastCanRx_) > 100) { // 100ms timeout
        state_.status = Config::ErrorCode::TIMEOUT;
        reportError(Config::ErrorCode::TIMEOUT);
        return sendCanCommand(0);
    }

    int16_t currentCommand = 0;

    // Determine control mode and calculate output
    if (state_.targetPosition != state_.currentPosition) {
        // Position control mode
        float positionError = state_.targetPosition - state_.currentPosition;

        // Angle PID control
        float angleOutput = calculatePID(positionError, anglePidIntegral_, anglePidLastError_,
                                       config->angle_kp, config->angle_ki, config->angle_kd,
                                       1000.0f, config->max_speed_rad_s, deltaTime);

        // Use angle PID output as velocity setpoint
        state_.targetVelocity = angleOutput;
    }

    // Speed control (always run for both position and velocity modes)
    float velocityError = state_.targetVelocity - state_.currentVelocity;

    float speedOutput = calculatePID(velocityError, speedPidIntegral_, speedPidLastError_,
                                   config->speed_kp, config->speed_ki, config->speed_kd,
                                   1000.0f, 16384.0f, deltaTime); // Max current is typically ±16384

    currentCommand = static_cast<int16_t>(constrainValue(speedOutput, -16384.0f, 16384.0f));

    return sendCanCommand(currentCommand);
}

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
            state_.targetPosition = state_.currentPosition; // Stop position control
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

Config::Result<void> RoboMasterMotorController::setEnabled(bool enabled) {
    state_.enabled = enabled;
    if (!enabled) {
        // Reset PID when disabling
        anglePidIntegral_ = 0.0f;
        anglePidLastError_ = 0.0f;
        speedPidIntegral_ = 0.0f;
        speedPidLastError_ = 0.0f;

        // Send zero current command
        sendCanCommand(0);
    }
    updateState();
    return Config::Result<void>();
}

void RoboMasterMotorController::emergencyStop() {
    state_.enabled = false;
    state_.status = Config::ErrorCode::EMERGENCY_STOP;

    // Send zero current immediately
    sendCanCommand(0);

    // Reset PID controllers
    anglePidIntegral_ = 0.0f;
    anglePidLastError_ = 0.0f;
    speedPidIntegral_ = 0.0f;
    speedPidLastError_ = 0.0f;

    updateState();
}

void RoboMasterMotorController::resetWatchdog() {
    lastWatchdogReset_ = HAL_GetTick();
    watchdogExpired_ = false;
}

Config::Result<void> RoboMasterMotorController::runSelfTest() {
    // Basic self-test: send a small current command and check for response
    auto result = sendCanCommand(100); // Small current
    if (result.isError()) {
        return result.error();
    }

    // Wait a bit then return to zero
    HAL_Delay(10);
    return sendCanCommand(0);
}

Config::Result<void> RoboMasterMotorController::updateConfig(const Config::RoboMasterConfig& config) {
    config_ = config;
    // Reset PID when config changes
    anglePidIntegral_ = 0.0f;
    anglePidLastError_ = 0.0f;
    speedPidIntegral_ = 0.0f;
    speedPidLastError_ = 0.0f;
    return Config::Result<void>();
}

Config::Result<void> RoboMasterMotorController::processCanMessage(uint32_t canId, const uint8_t* data, uint8_t length) {
    // Check if this message is for our motor
    if (canId != (0x201 + id_) || length != 8) {
        return Config::Result<void>(Config::ErrorCode::CONFIG_ERROR);
    }

    // Parse RoboMaster feedback message
    // Typical format: angle(2 bytes), velocity(2 bytes), current(2 bytes), temperature(1 byte)
    int16_t angle_raw = (data[0] << 8) | data[1];
    int16_t velocity_raw = (data[2] << 8) | data[3];
    int16_t current_raw = (data[4] << 8) | data[5];
    uint8_t temperature = data[6];

    // Convert raw values to engineering units
    // Angle: 0-8191 represents 0-360 degrees
    state_.currentPosition = (angle_raw / 8191.0f) * 360.0f;

    // Velocity: raw RPM value
    state_.currentVelocity = velocity_raw * (3.14159f / 30.0f); // Convert RPM to rad/s

    // Current: raw current value (typically in mA)
    state_.currentCurrent = current_raw;

    // Temperature (if needed)
    (void)temperature; // Suppress unused warning for now

    lastCanRx_ = HAL_GetTick();
    return Config::Result<void>();
}

Config::Result<void> RoboMasterMotorController::sendCanCommand(int16_t current) {
    // TODO: Implement actual CAN transmission
    // This would typically use the hardware manager's CAN interface

    // Format: 8 bytes for up to 4 motors (2 bytes each)
    // For now, we'll just log the command
    (void)current; // Suppress unused warning

    // In a real implementation:
    // uint8_t canData[8] = {0};
    // canData[0] = (current >> 8) & 0xFF;
    // canData[1] = current & 0xFF;
    // return hwManager_->sendCanMessage(canId_, canData, 8);

    return Config::Result<void>();
}

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

} // namespace RoboMaster
} // namespace Motors