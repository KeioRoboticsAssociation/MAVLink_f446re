/**
 * @file unified_mavlink_handler.hpp
 * @brief Defines the UnifiedMAVLinkHandler class for managing MAVLink communication.
 */

#pragma once

#include "../hal/hardware_manager.hpp"
#include "../motors/base/motor_interface.hpp"
#include "../config/system_config.hpp"

extern "C" {
#include "mavlink/c_library_v2/common/mavlink.h"
}

#include <functional>
#include <array>
#include <queue>
#include <memory>

namespace Communication {

// Forward declarations
class MessageDispatcher;
class MotorCommandDispatcher;
class ParameterDispatcher;
class TelemetryDispatcher;

/**
 * @brief A simple ring buffer for UART communication.
 *
 * @tparam SIZE The size of the buffer.
 */
template<size_t SIZE>
class RingBuffer {
private:
    std::array<uint8_t, SIZE> buffer_;
    volatile size_t head_ = 0;
    volatile size_t tail_ = 0;

public:
    /**
     * @brief Pushes a byte into the buffer.
     *
     * @param byte The byte to push.
     * @return true if the byte was pushed successfully, false if the buffer is full.
     */
    bool push(uint8_t byte);

    /**
     * @brief Pops a byte from the buffer.
     *
     * @return A Result containing the byte or an error if the buffer is empty.
     */
    Config::Result<uint8_t> pop();

    /**
     * @brief Returns the number of bytes available in the buffer.
     *
     * @return The number of available bytes.
     */
    size_t available() const;

    /**
     * @brief Checks if the buffer is empty.
     *
     * @return true if the buffer is empty, false otherwise.
     */
    bool empty() const;

    /**
     * @brief Checks if the buffer is full.
     *
     * @return true if the buffer is full, false otherwise.
     */
    bool full() const;
};

/**
 * @brief Manages MAVLink communication using a message dispatcher pattern.
 *
 * This class orchestrates the entire MAVLink communication stack, including message parsing,
 * dispatching to appropriate handlers, and sending telemetry.
 */
class UnifiedMAVLinkHandler {
private:
    HAL::HardwareManager* hwManager_;
    mavlink_status_t mavlinkStatus_;
    mavlink_message_t rxMessage_;

    std::unique_ptr<MessageDispatcher> messageDispatcher_;
    std::unique_ptr<MotorCommandDispatcher> motorCommandDispatcher_;
    std::unique_ptr<ParameterDispatcher> parameterDispatcher_;
    std::unique_ptr<TelemetryDispatcher> telemetryDispatcher_;

    RingBuffer<Config::Memory::RING_BUFFER_SIZE> rxBuffer_;

    uint8_t systemId_;
    uint8_t componentId_;

    std::function<void(uint8_t, Config::ErrorCode)> errorCallback_;

public:
    /**
     * @brief Construct a new UnifiedMAVLinkHandler object.
     *
     * @param hwManager Pointer to the hardware manager.
     * @param motorRegistry Pointer to the motor registry.
     * @param systemId The MAVLink system ID.
     * @param componentId The MAVLink component ID.
     */
    UnifiedMAVLinkHandler(HAL::HardwareManager* hwManager,
                         Motors::MotorRegistry* motorRegistry,
                         uint8_t systemId = Config::System::MAVLINK_SYSTEM_ID,
                         uint8_t componentId = Config::System::MAVLINK_COMPONENT_ID);

    /**
     * @brief Destroy the UnifiedMAVLinkHandler object.
     */
    ~UnifiedMAVLinkHandler();

    /**
     * @brief Initializes the MAVLink handler and its components.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> initialize();

    /**
     * @brief Updates the MAVLink handler, processing incoming and outgoing messages.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> update();

    /**
     * @brief Sends a MAVLink message.
     *
     * @param msg The MAVLink message to send.
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> sendMessage(const mavlink_message_t& msg);

    /**
     * @brief Processes a single received byte.
     *
     * @param byte The byte to process.
     */
    void processReceivedByte(uint8_t byte);

    /**
     * @brief Handles a fully parsed MAVLink message.
     *
     * @param msg The received MAVLink message.
     */
    void handleReceivedMessage(const mavlink_message_t& msg);

    /**
     * @brief Sets the MAVLink system ID.
     *
     * @param systemId The new system ID.
     */
    void setSystemId(uint8_t systemId) { systemId_ = systemId; }

    /**
     * @brief Sets the MAVLink component ID.
     *
     * @param componentId The new component ID.
     */
    void setComponentId(uint8_t componentId) { componentId_ = componentId; }

    /**
     * @brief Gets the MAVLink system ID.
     *
     * @return The system ID.
     */
    uint8_t getSystemId() const { return systemId_; }

    /**
     * @brief Gets the MAVLink component ID.
     *
     * @return The component ID.
     */
    uint8_t getComponentId() const { return componentId_; }

    /**
     * @brief Sets the error callback function.
     *
     * @param callback The function to call on error.
     */
    void setErrorCallback(std::function<void(uint8_t, Config::ErrorCode)> callback) {
        errorCallback_ = callback;
    }

    /**
     * @brief Gets a pointer to the motor command dispatcher.
     *
     * @return A pointer to the MotorCommandDispatcher.
     */
    MotorCommandDispatcher* getMotorCommandDispatcher() const { return motorCommandDispatcher_.get(); }

    /**
     * @brief Gets a pointer to the parameter dispatcher.
     *
     * @return A pointer to the ParameterDispatcher.
     */
    ParameterDispatcher* getParameterDispatcher() const { return parameterDispatcher_.get(); }

    /**
     * @brief Gets a pointer to the telemetry dispatcher.
     *
     * @return A pointer to the TelemetryDispatcher.
     */
    TelemetryDispatcher* getTelemetryDispatcher() const { return telemetryDispatcher_.get(); }

    /**
     * @brief Gets the number of available bytes in the RX buffer.
     *
     * @return The number of available bytes.
     */
    size_t getRxBufferAvailable() const { return rxBuffer_.available(); }

    /**
     * @brief Gets the number of processed messages.
     *
     * @return The number of processed messages.
     */
    uint32_t getProcessedMessageCount() const;

    /**
     * @brief Gets the number of dropped messages.
     *
     * @return The number of dropped messages.
     */
    uint32_t getDroppedMessageCount() const;

private:
    Config::Result<Config::ErrorCode> processRxBuffer();
    void handleError(Config::ErrorCode error);
    void onUartRxComplete(uint8_t* data, size_t length);
    void onUartTxComplete();
    void onUartError();
};

} // namespace Communication