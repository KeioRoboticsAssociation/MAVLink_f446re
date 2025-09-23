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

namespace Communication {

// MAVLink device interface for different motor types
class IMAVLinkDevice {
public:
    virtual ~IMAVLinkDevice() = default;
    virtual uint8_t getDeviceId() const = 0;
    virtual Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) = 0;
    virtual Config::Result<Config::ErrorCode> generateTelemetry(mavlink_message_t& msg, uint8_t systemId, uint8_t componentId) = 0;
    virtual bool hasPendingTelemetry() const = 0;
};

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

// Unified MAVLink communication manager
class UnifiedMAVLinkHandler {
private:
    // Communication state
    HAL::HardwareManager* hwManager_;
    mavlink_status_t mavlinkStatus_;
    mavlink_message_t rxMessage_;

    // Device registry
    std::array<IMAVLinkDevice*, Config::System::MAX_MOTORS> devices_;
    size_t deviceCount_ = 0;

    // Communication buffers
    RingBuffer<Config::Memory::RING_BUFFER_SIZE> rxBuffer_;
    RingBuffer<Config::Memory::RING_BUFFER_SIZE> txBuffer_;

    // MAVLink parameters
    uint8_t systemId_;
    uint8_t componentId_;

    // Telemetry timing
    uint32_t lastHeartbeat_ = 0;
    uint32_t lastTelemetry_ = 0;
    size_t telemetryDeviceIndex_ = 0;

    // Message queues
    std::queue<mavlink_message_t> pendingMessages_;

    // Callbacks
    std::function<void(uint8_t, Config::ErrorCode)> errorCallback_;

public:
    UnifiedMAVLinkHandler(HAL::HardwareManager* hwManager, uint8_t systemId = Config::System::MAVLINK_SYSTEM_ID,
                         uint8_t componentId = Config::System::MAVLINK_COMPONENT_ID);

    // Initialization
    Config::Result<Config::ErrorCode> initialize();

    // Device registration
    Config::Result<Config::ErrorCode> registerDevice(IMAVLinkDevice* device);
    Config::Result<Config::ErrorCode> unregisterDevice(uint8_t deviceId);

    // Communication
    Config::Result<Config::ErrorCode> update();
    Config::Result<Config::ErrorCode> sendMessage(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> queueMessage(const mavlink_message_t& msg);

    // Protocol handling
    void processReceivedByte(uint8_t byte);
    void handleReceivedMessage(const mavlink_message_t& msg);

    // Telemetry
    Config::Result<Config::ErrorCode> sendHeartbeat();
    Config::Result<Config::ErrorCode> sendTelemetry();

    // Configuration
    void setSystemId(uint8_t systemId) { systemId_ = systemId; }
    void setComponentId(uint8_t componentId) { componentId_ = componentId; }
    uint8_t getSystemId() const { return systemId_; }
    uint8_t getComponentId() const { return componentId_; }

    // Error handling
    void setErrorCallback(std::function<void(uint8_t, Config::ErrorCode)> callback) {
        errorCallback_ = callback;
    }

    // Statistics
    size_t getRegisteredDeviceCount() const { return deviceCount_; }
    size_t getPendingMessageCount() const { return pendingMessages_.size(); }
    size_t getRxBufferAvailable() const { return rxBuffer_.available(); }
    size_t getTxBufferAvailable() const { return txBuffer_.available(); }

private:
    // Internal helpers
    Config::Result<Config::ErrorCode> processRxBuffer();
    Config::Result<Config::ErrorCode> processTxBuffer();
    IMAVLinkDevice* findDevice(uint8_t deviceId);
    void handleError(Config::ErrorCode error);

    // MAVLink message handlers
    void handleHeartbeat(const mavlink_message_t& msg);
    void handleParameterRequest(const mavlink_message_t& msg);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleManualControl(const mavlink_message_t& msg);
    void handleRcChannelsOverride(const mavlink_message_t& msg);

    // UART callbacks
    void onUartRxComplete(uint8_t* data, size_t length);
    void onUartTxComplete();
    void onUartError();
};

// Servo MAVLink device adapter
class ServoMAVLinkDevice : public IMAVLinkDevice {
private:
    Motors::IMotorController<Config::ServoConfig>* controller_;
    uint8_t deviceId_;

public:
    ServoMAVLinkDevice(Motors::IMotorController<Config::ServoConfig>* controller, uint8_t deviceId)
        : controller_(controller), deviceId_(deviceId) {}

    uint8_t getDeviceId() const override { return deviceId_; }

    Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) override;
    Config::Result<Config::ErrorCode> generateTelemetry(mavlink_message_t& msg, uint8_t systemId, uint8_t componentId) override;
    bool hasPendingTelemetry() const override;
};

// DC Motor MAVLink device adapter
class DCMotorMAVLinkDevice : public IMAVLinkDevice {
private:
    Motors::IMotorController<Config::DCMotorConfig>* controller_;
    uint8_t deviceId_;

public:
    DCMotorMAVLinkDevice(Motors::IMotorController<Config::DCMotorConfig>* controller, uint8_t deviceId)
        : controller_(controller), deviceId_(deviceId) {}

    uint8_t getDeviceId() const override { return deviceId_; }

    Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) override;
    Config::Result<Config::ErrorCode> generateTelemetry(mavlink_message_t& msg, uint8_t systemId, uint8_t componentId) override;
    bool hasPendingTelemetry() const override;
};

// RoboMaster MAVLink device adapter
class RoboMasterMAVLinkDevice : public IMAVLinkDevice {
private:
    Motors::IMotorController<Config::RoboMasterConfig>* controller_;
    uint8_t deviceId_;

public:
    RoboMasterMAVLinkDevice(Motors::IMotorController<Config::RoboMasterConfig>* controller, uint8_t deviceId)
        : controller_(controller), deviceId_(deviceId) {}

    uint8_t getDeviceId() const override { return deviceId_; }

    Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) override;
    Config::Result<Config::ErrorCode> generateTelemetry(mavlink_message_t& msg, uint8_t systemId, uint8_t componentId) override;
    bool hasPendingTelemetry() const override;
};

} // namespace Communication