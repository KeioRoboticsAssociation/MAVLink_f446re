/**
 * @file unified_mavlink_handler.cpp
 * @brief Implementation of the UnifiedMAVLinkHandler class.
 */

#include "unified_mavlink_handler.hpp"
#include "message_dispatcher.hpp"
#include "../config/robot_config.hpp"

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Communication {

/**
 * @brief Construct a new UnifiedMAVLinkHandler object.
 *
 * @param hwManager Pointer to the hardware manager.
 * @param motorRegistry Pointer to the motor registry.
 * @param systemId The MAVLink system ID.
 * @param componentId The MAVLink component ID.
 */
UnifiedMAVLinkHandler::UnifiedMAVLinkHandler(HAL::HardwareManager* hwManager, Motors::MotorRegistry* motorRegistry, uint8_t systemId, uint8_t componentId)
    : hwManager_(hwManager), systemId_(systemId), componentId_(componentId) {
    memset(&mavlinkStatus_, 0, sizeof(mavlinkStatus_));
    memset(&rxMessage_, 0, sizeof(rxMessage_));

    messageDispatcher_ = std::make_unique<MessageDispatcher>();

    auto sendCallback = [this](const mavlink_message_t& msg) {
        return this->sendMessage(msg);
    };

    motorCommandDispatcher_ = std::make_unique<MotorCommandDispatcher>(motorRegistry);
    parameterDispatcher_ = std::make_unique<ParameterDispatcher>(systemId_, componentId_, sendCallback, &Storage::g_parameter_storage);
    telemetryDispatcher_ = std::make_unique<TelemetryDispatcher>(motorRegistry, systemId_, componentId_, sendCallback);

    messageDispatcher_->registerHandler(std::make_unique<MotorCommandDispatcher>(motorRegistry));
    messageDispatcher_->registerHandler(std::make_unique<ParameterDispatcher>(systemId_, componentId_, sendCallback, &Storage::g_parameter_storage));
}

/**
 * @brief Destroy the UnifiedMAVLinkHandler object.
 */
UnifiedMAVLinkHandler::~UnifiedMAVLinkHandler() = default;

/**
 * @brief Initializes the MAVLink handler and its components.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::initialize() {
    if (!hwManager_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto uartResult = hwManager_->getUART(HAL::UARTID::UART_2);
    if (!uartResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    auto callbackResult = hwManager_->setUARTRxCallback(HAL::UARTID::UART_2,
        [this](uint8_t* data, size_t length) {
            this->onUartRxComplete(data, length);
        });

    if (!callbackResult) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Updates the MAVLink handler, processing incoming and outgoing messages.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::update() {
    auto rxResult = processRxBuffer();
    if (!rxResult) {
        handleError(rxResult.error());
    }

    if (telemetryDispatcher_) {
        auto telemetryResult = telemetryDispatcher_->update();
        if (!telemetryResult) {
            handleError(telemetryResult.error());
        }
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Sends a MAVLink message.
 *
 * @param msg The MAVLink message to send.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::sendMessage(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);

    auto transmitResult = hwManager_->transmitUART(HAL::UARTID::UART_2, buffer, length);
    if (!transmitResult) {
        return Config::ErrorCode::COMMUNICATION_ERROR;
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Processes a single received byte.
 *
 * @param byte The byte to process.
 */
void UnifiedMAVLinkHandler::processReceivedByte(uint8_t byte) {
    if (!rxBuffer_.push(byte)) {
        handleError(Config::ErrorCode::OUT_OF_RANGE);
    }
}

/**
 * @brief Handles a fully parsed MAVLink message.
 *
 * @param msg The received MAVLink message.
 */
void UnifiedMAVLinkHandler::handleReceivedMessage(const mavlink_message_t& msg) {
    if (messageDispatcher_) {
        auto result = messageDispatcher_->dispatchMessage(msg);
        if (!result) {
            handleError(result.error());
        }
    }
}

/**
 * @brief Gets the number of processed messages.
 *
 * @return The number of processed messages.
 */
uint32_t UnifiedMAVLinkHandler::getProcessedMessageCount() const {
    return messageDispatcher_ ? messageDispatcher_->getProcessedCount() : 0;
}

/**
 * @brief Gets the number of dropped messages.
 *
 * @return The number of dropped messages.
 */
uint32_t UnifiedMAVLinkHandler::getDroppedMessageCount() const {
    return messageDispatcher_ ? messageDispatcher_->getDroppedCount() : 0;
}

/**
 * @brief Processes the RX buffer, parsing MAVLink messages.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::processRxBuffer() {
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

/**
 * @brief Handles an error by invoking the error callback.
 *
 * @param error The error code to handle.
 */
void UnifiedMAVLinkHandler::handleError(Config::ErrorCode error) {
    if (errorCallback_) {
        errorCallback_(systemId_, error);
    }
}

/**
 * @brief Callback for when UART RX is complete.
 *
 * @param data Pointer to the received data.
 * @param length The length of the received data.
 */
void UnifiedMAVLinkHandler::onUartRxComplete(uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        processReceivedByte(data[i]);
    }
}

/**
 * @brief Callback for when UART TX is complete.
 */
void UnifiedMAVLinkHandler::onUartTxComplete() {
    // Can be used for flow control if needed
}

/**
 * @brief Callback for UART errors.
 */
void UnifiedMAVLinkHandler::onUartError() {
    handleError(Config::ErrorCode::COMMUNICATION_ERROR);
}

} // namespace Communication