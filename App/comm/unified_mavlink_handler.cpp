#include "unified_mavlink_handler.hpp"
#include "../config/robot_config.hpp"

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Communication {

// UnifiedMAVLinkHandler implementation
UnifiedMAVLinkHandler::UnifiedMAVLinkHandler(HAL::HardwareManager* hwManager, uint8_t systemId, uint8_t componentId)
    : hwManager_(hwManager), systemId_(systemId), componentId_(componentId) {

    // Initialize MAVLink status
    memset(&mavlinkStatus_, 0, sizeof(mavlinkStatus_));
    memset(&rxMessage_, 0, sizeof(rxMessage_));
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::initialize() {
    if (!hwManager_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    // Setup UART for MAVLink communication
    auto uartResult = hwManager_->getUART(HAL::UARTID::UART_2);
    if (!uartResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    // Setup UART RX callback for incoming data
    auto callbackResult = hwManager_->setUARTRxCallback(HAL::UARTID::UART_2,
        [this](uint8_t* data, size_t length) {
            this->onUartRxComplete(data, length);
        });

    if (!callbackResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    // Initialize timing
    lastHeartbeat_ = HAL_GetTick();
    lastTelemetry_ = lastHeartbeat_;

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::registerDevice(IMAVLinkDevice* device) {
    if (!device) {
        return Config::ErrorCode::CONFIG_ERROR;
    }

    if (deviceCount_ >= Config::System::MAX_MOTORS) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    // Check for duplicate device IDs
    for (size_t i = 0; i < deviceCount_; ++i) {
        if (devices_[i] && devices_[i]->getDeviceId() == device->getDeviceId()) {
            return Config::ErrorCode::CONFIG_ERROR;
        }
    }

    devices_[deviceCount_++] = device;
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::unregisterDevice(uint8_t deviceId) {
    for (size_t i = 0; i < deviceCount_; ++i) {
        if (devices_[i] && devices_[i]->getDeviceId() == deviceId) {
            // Shift remaining devices down
            for (size_t j = i; j < deviceCount_ - 1; ++j) {
                devices_[j] = devices_[j + 1];
            }
            devices_[--deviceCount_] = nullptr;
            return Config::ErrorCode::OK;
        }
    }
    return Config::ErrorCode::OUT_OF_RANGE;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::update() {
    uint32_t currentTime = HAL_GetTick();

    // Process received data
    auto rxResult = processRxBuffer();
    if (!rxResult) {
        handleError(rxResult.error());
    }

    // Send queued messages
    auto txResult = processTxBuffer();
    if (!txResult) {
        handleError(txResult.error());
    }

    // Send periodic heartbeat
    if (currentTime - lastHeartbeat_ >= Config::Communication::MAVLINK_HEARTBEAT_INTERVAL_MS) {
        auto heartbeatResult = sendHeartbeat();
        if (!heartbeatResult) {
            handleError(heartbeatResult.error());
        }
        lastHeartbeat_ = currentTime;
    }

    // Send telemetry from devices
    uint32_t telemetryInterval = 1000 / Config::Communication::TELEMETRY_RATE_HZ;
    if (currentTime - lastTelemetry_ >= telemetryInterval) {
        auto telemetryResult = sendTelemetry();
        if (!telemetryResult) {
            handleError(telemetryResult.error());
        }
        lastTelemetry_ = currentTime;
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::sendMessage(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);

    // Send directly via UART
    auto transmitResult = hwManager_->transmitUART(HAL::UARTID::UART_2, buffer, length);
    if (!transmitResult) {
        return Config::ErrorCode::COMMUNICATION_ERROR;
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::queueMessage(const mavlink_message_t& msg) {
    if (pendingMessages_.size() >= Config::Communication::MAX_PENDING_COMMANDS) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    pendingMessages_.push(msg);
    return Config::ErrorCode::OK;
}

void UnifiedMAVLinkHandler::processReceivedByte(uint8_t byte) {
    // Push byte into receive buffer
    if (!rxBuffer_.push(byte)) {
        // Buffer full - this is an error condition
        handleError(Config::ErrorCode::OUT_OF_RANGE);
    }
}

void UnifiedMAVLinkHandler::handleReceivedMessage(const mavlink_message_t& msg) {
    // Handle system-level messages
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            handleParameterRequest(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleCommandLong(msg);
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            handleManualControl(msg);
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handleRcChannelsOverride(msg);
            break;
        default:
            // Route message to appropriate device
            for (size_t i = 0; i < deviceCount_; ++i) {
                if (devices_[i]) {
                    auto result = devices_[i]->handleMessage(msg);
                    if (result) {
                        // Message was handled successfully
                        break;
                    }
                }
            }
            break;
    }
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(systemId_, componentId_, &msg,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);

    return sendMessage(msg);
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::sendTelemetry() {
    // Round-robin through devices for telemetry
    if (deviceCount_ == 0) {
        return Config::ErrorCode::OK;
    }

    // Find next device with pending telemetry
    size_t startIndex = telemetryDeviceIndex_;
    do {
        if (devices_[telemetryDeviceIndex_] && devices_[telemetryDeviceIndex_]->hasPendingTelemetry()) {
            mavlink_message_t msg;
            auto result = devices_[telemetryDeviceIndex_]->generateTelemetry(msg, systemId_, componentId_);
            if (result) {
                sendMessage(msg);
                break;
            }
        }
        telemetryDeviceIndex_ = (telemetryDeviceIndex_ + 1) % deviceCount_;
    } while (telemetryDeviceIndex_ != startIndex);

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::processRxBuffer() {
    // Process all available bytes
    while (!rxBuffer_.empty()) {
        auto byteResult = rxBuffer_.pop();
        if (!byteResult) {
            break;
        }

        uint8_t byte = byteResult.get();
        uint8_t msgReceived = mavlink_parse_char(MAVLINK_COMM_0, byte, &rxMessage_, &mavlinkStatus_);

        if (msgReceived) {
            handleReceivedMessage(rxMessage_);
        }
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::processTxBuffer() {
    // Send queued messages
    while (!pendingMessages_.empty()) {
        const mavlink_message_t& msg = pendingMessages_.front();
        auto result = sendMessage(msg);
        if (!result) {
            // Failed to send - stop processing queue
            return result.error();
        }
        pendingMessages_.pop();
    }

    return Config::ErrorCode::OK;
}

IMAVLinkDevice* UnifiedMAVLinkHandler::findDevice(uint8_t deviceId) {
    for (size_t i = 0; i < deviceCount_; ++i) {
        if (devices_[i] && devices_[i]->getDeviceId() == deviceId) {
            return devices_[i];
        }
    }
    return nullptr;
}

void UnifiedMAVLinkHandler::handleError(Config::ErrorCode error) {
    if (errorCallback_) {
        errorCallback_(systemId_, error);
    }
}

// Message handlers
void UnifiedMAVLinkHandler::handleHeartbeat(const mavlink_message_t& msg) {
    // Update heartbeat timing for watchdog
    (void)msg; // Suppress unused warning
}

void UnifiedMAVLinkHandler::handleParameterRequest(const mavlink_message_t& msg) {
    // TODO: Implement parameter handling
    (void)msg; // Suppress unused warning
}

void UnifiedMAVLinkHandler::handleCommandLong(const mavlink_message_t& msg) {
    // TODO: Implement command handling
    (void)msg; // Suppress unused warning
}

void UnifiedMAVLinkHandler::handleManualControl(const mavlink_message_t& msg) {
    // Route manual control to appropriate devices
    mavlink_manual_control_t manual_control;
    mavlink_msg_manual_control_decode(&msg, &manual_control);

    // Convert to motor commands and route to devices
    for (size_t i = 0; i < deviceCount_; ++i) {
        if (devices_[i]) {
            devices_[i]->handleMessage(msg);
        }
    }
}

void UnifiedMAVLinkHandler::handleRcChannelsOverride(const mavlink_message_t& msg) {
    // Route RC channel overrides to appropriate devices
    for (size_t i = 0; i < deviceCount_; ++i) {
        if (devices_[i]) {
            devices_[i]->handleMessage(msg);
        }
    }
}

// UART callbacks
void UnifiedMAVLinkHandler::onUartRxComplete(uint8_t* data, size_t length) {
    // Process received bytes
    for (size_t i = 0; i < length; ++i) {
        processReceivedByte(data[i]);
    }
}

void UnifiedMAVLinkHandler::onUartTxComplete() {
    // Transmission complete - could be used for flow control
}

void UnifiedMAVLinkHandler::onUartError() {
    handleError(Config::ErrorCode::COMMUNICATION_ERROR);
}

// ServoMAVLinkDevice implementation
Config::Result<Config::ErrorCode> ServoMAVLinkDevice::handleMessage(const mavlink_message_t& msg) {
    if (!controller_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            mavlink_rc_channels_override_t rc_override;
            mavlink_msg_rc_channels_override_decode(&msg, &rc_override);

            // Map channel to servo (assuming deviceId maps to channels 1-8)
            uint16_t pwm_value = 0;
            switch (deviceId_) {
                case 1: pwm_value = rc_override.chan1_raw; break;
                case 2: pwm_value = rc_override.chan2_raw; break;
                case 3: pwm_value = rc_override.chan3_raw; break;
                case 4: pwm_value = rc_override.chan4_raw; break;
                default: return Config::ErrorCode::OUT_OF_RANGE;
            }

            if (pwm_value != 0) {
                // Convert PWM to degrees (1000-2000 -> -90 to +90)
                float degrees = (pwm_value - 1500.0f) / 500.0f * 90.0f;

                Motors::MotorCommand cmd;
                cmd.motorId = deviceId_;
                cmd.mode = Motors::ControlMode::POSITION;
                cmd.targetValue = degrees;
                cmd.enable = true;
                cmd.timestamp = HAL_GetTick();

                return controller_->setCommand(cmd);
            }
            break;
        }
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> ServoMAVLinkDevice::generateTelemetry(mavlink_message_t& msg, uint8_t systemId, uint8_t componentId) {
    if (!controller_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto state = controller_->getState();

    // Generate servo output raw message
    mavlink_msg_servo_output_raw_pack(systemId, componentId, &msg,
        state.lastUpdateTime, 0, // time_usec, port
        static_cast<uint16_t>((state.currentPosition / 90.0f) * 500.0f + 1500.0f), // servo 1
        0, 0, 0, 0, 0, 0, 0 // servos 2-8
    );

    return Config::ErrorCode::OK;
}

bool ServoMAVLinkDevice::hasPendingTelemetry() const {
    return controller_ && controller_->getStatus() == Motors::MotorStatus::OK;
}

// DCMotorMAVLinkDevice implementation (simplified)
Config::Result<Config::ErrorCode> DCMotorMAVLinkDevice::handleMessage(const mavlink_message_t& msg) {
    // TODO: Implement DC motor MAVLink message handling
    (void)msg;
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> DCMotorMAVLinkDevice::generateTelemetry(mavlink_message_t& msg, uint8_t systemId, uint8_t componentId) {
    // TODO: Implement DC motor telemetry generation
    (void)msg;
    (void)systemId;
    (void)componentId;
    return Config::ErrorCode::OK;
}

bool DCMotorMAVLinkDevice::hasPendingTelemetry() const {
    return false;
}

// RoboMasterMAVLinkDevice implementation (simplified)
Config::Result<Config::ErrorCode> RoboMasterMAVLinkDevice::handleMessage(const mavlink_message_t& msg) {
    // TODO: Implement RoboMaster MAVLink message handling
    (void)msg;
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> RoboMasterMAVLinkDevice::generateTelemetry(mavlink_message_t& msg, uint8_t systemId, uint8_t componentId) {
    // TODO: Implement RoboMaster telemetry generation
    (void)msg;
    (void)systemId;
    (void)componentId;
    return Config::ErrorCode::OK;
}

bool RoboMasterMAVLinkDevice::hasPendingTelemetry() const {
    return false;
}

} // namespace Communication