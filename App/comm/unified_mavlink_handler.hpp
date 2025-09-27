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

// Ring buffer for UART communication
template<size_t SIZE>
class RingBuffer {
private:
    std::array<uint8_t, SIZE> buffer_;
    volatile size_t head_ = 0;
    volatile size_t tail_ = 0;

public:
    bool push(uint8_t byte) {
        size_t next = (head_ + 1) % SIZE;
        if (next == tail_) {
            return false; // Buffer full
        }
        buffer_[head_] = byte;
        head_ = next;
        return true;
    }

    Config::Result<uint8_t> pop() {
        if (head_ == tail_) {
            return Config::Result<uint8_t>(Config::ErrorCode::OUT_OF_RANGE); // Buffer empty
        }
        uint8_t data = buffer_[tail_];
        tail_ = (tail_ + 1) % SIZE;
        return data;
    }

    size_t available() const {
        return (head_ >= tail_) ? (head_ - tail_) : (SIZE + head_ - tail_);
    }

    bool empty() const {
        return head_ == tail_;
    }

    bool full() const {
        return ((head_ + 1) % SIZE) == tail_;
    }
};

// Unified MAVLink communication manager using Message Dispatcher pattern
class UnifiedMAVLinkHandler {
private:
    // Communication state
    HAL::HardwareManager* hwManager_;
    mavlink_status_t mavlinkStatus_;
    mavlink_message_t rxMessage_;

    // Message dispatching system
    std::unique_ptr<MessageDispatcher> messageDispatcher_;
    std::unique_ptr<MotorCommandDispatcher> motorCommandDispatcher_;
    std::unique_ptr<ParameterDispatcher> parameterDispatcher_;
    std::unique_ptr<TelemetryDispatcher> telemetryDispatcher_;

    // Communication buffers
    RingBuffer<Config::Memory::RING_BUFFER_SIZE> rxBuffer_;

    // MAVLink parameters
    uint8_t systemId_;
    uint8_t componentId_;

    // Callbacks
    std::function<void(uint8_t, Config::ErrorCode)> errorCallback_;

public:
    UnifiedMAVLinkHandler(HAL::HardwareManager* hwManager,
                         Motors::MotorRegistry* motorRegistry,
                         uint8_t systemId = Config::System::MAVLINK_SYSTEM_ID,
                         uint8_t componentId = Config::System::MAVLINK_COMPONENT_ID);

    ~UnifiedMAVLinkHandler();

    // Initialization
    Config::Result<Config::ErrorCode> initialize();

    // Communication
    Config::Result<Config::ErrorCode> update();
    Config::Result<Config::ErrorCode> sendMessage(const mavlink_message_t& msg);

    // Protocol handling
    void processReceivedByte(uint8_t byte);
    void handleReceivedMessage(const mavlink_message_t& msg);

    // Configuration
    void setSystemId(uint8_t systemId) { systemId_ = systemId; }
    void setComponentId(uint8_t componentId) { componentId_ = componentId; }
    uint8_t getSystemId() const { return systemId_; }
    uint8_t getComponentId() const { return componentId_; }

    // Error handling
    void setErrorCallback(std::function<void(uint8_t, Config::ErrorCode)> callback) {
        errorCallback_ = callback;
    }

    // Access to dispatchers for configuration
    MotorCommandDispatcher* getMotorCommandDispatcher() const { return motorCommandDispatcher_.get(); }
    ParameterDispatcher* getParameterDispatcher() const { return parameterDispatcher_.get(); }
    TelemetryDispatcher* getTelemetryDispatcher() const { return telemetryDispatcher_.get(); }

    // Statistics
    size_t getRxBufferAvailable() const { return rxBuffer_.available(); }
    uint32_t getProcessedMessageCount() const;
    uint32_t getDroppedMessageCount() const;

private:
    // Internal helpers
    Config::Result<Config::ErrorCode> processRxBuffer();
    void handleError(Config::ErrorCode error);

    // UART callbacks
    void onUartRxComplete(uint8_t* data, size_t length);
    void onUartTxComplete();
    void onUartError();
};

} // namespace Communication