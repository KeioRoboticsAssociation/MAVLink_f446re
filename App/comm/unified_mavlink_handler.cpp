#include "unified_mavlink_handler.hpp"
#include "message_dispatcher.hpp"
#include "../config/robot_config.hpp"

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Communication {

// UnifiedMAVLinkHandler implementation
UnifiedMAVLinkHandler::UnifiedMAVLinkHandler(HAL::HardwareManager* hwManager, Motors::MotorRegistry* motorRegistry, uint8_t systemId, uint8_t componentId)
    : hwManager_(hwManager), systemId_(systemId), componentId_(componentId) {

    // Initialize MAVLink status
    memset(&mavlinkStatus_, 0, sizeof(mavlinkStatus_));
    memset(&rxMessage_, 0, sizeof(rxMessage_));

    // Create message dispatcher
    messageDispatcher_ = std::make_unique<MessageDispatcher>();

    auto sendCallback = [this](const mavlink_message_t& msg) {
        return this->sendMessage(msg);
    };

    // Create and register specialized dispatchers
    motorCommandDispatcher_ = std::make_unique<MotorCommandDispatcher>(motorRegistry);
    parameterDispatcher_ = std::make_unique<ParameterDispatcher>(systemId_, componentId_, sendCallback);
    telemetryDispatcher_ = std::make_unique<TelemetryDispatcher>(motorRegistry, systemId_, componentId_, sendCallback);

    // Register copies of dispatchers with main message dispatcher
    // Note: We keep ownership of the specialized dispatchers for direct access
    messageDispatcher_->registerHandler(std::make_unique<MotorCommandDispatcher>(motorRegistry));
    messageDispatcher_->registerHandler(std::make_unique<ParameterDispatcher>(systemId_, componentId_, sendCallback));
}

UnifiedMAVLinkHandler::~UnifiedMAVLinkHandler() = default;

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

    // Timing initialization moved to TelemetryDispatcher

    return Config::ErrorCode::OK;
}

// Device registration methods removed - now handled by Message Dispatcher pattern

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::update() {
    // Process received data
    auto rxResult = processRxBuffer();
    if (!rxResult) {
        handleError(rxResult.error());
    }

    // Update telemetry dispatcher (handles periodic heartbeat and telemetry)
    if (telemetryDispatcher_) {
        auto telemetryResult = telemetryDispatcher_->update();
        if (!telemetryResult) {
            handleError(telemetryResult.error());
        }
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

// Message queueing removed - direct sending via dispatcher pattern

void UnifiedMAVLinkHandler::processReceivedByte(uint8_t byte) {
    // Push byte into receive buffer
    if (!rxBuffer_.push(byte)) {
        // Buffer full - this is an error condition
        handleError(Config::ErrorCode::OUT_OF_RANGE);
    }
}

void UnifiedMAVLinkHandler::handleReceivedMessage(const mavlink_message_t& msg) {
    // Dispatch message through the message dispatcher system
    if (messageDispatcher_) {
        auto result = messageDispatcher_->dispatchMessage(msg);
        if (!result) {
            // Log or handle dispatch failure
            handleError(result.error());
        }
    }
}

// Heartbeat and telemetry methods moved to TelemetryDispatcher

uint32_t UnifiedMAVLinkHandler::getProcessedMessageCount() const {
    return messageDispatcher_ ? messageDispatcher_->getProcessedCount() : 0;
}

uint32_t UnifiedMAVLinkHandler::getDroppedMessageCount() const {
    return messageDispatcher_ ? messageDispatcher_->getDroppedCount() : 0;
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

// Old device finding and TX buffer processing removed - handled by Message Dispatcher pattern

void UnifiedMAVLinkHandler::handleError(Config::ErrorCode error) {
    if (errorCallback_) {
        errorCallback_(systemId_, error);
    }
}

// Message handlers moved to specialized dispatchers

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

// Device adapter implementations moved to specialized dispatchers in message_dispatcher.cpp

} // namespace Communication