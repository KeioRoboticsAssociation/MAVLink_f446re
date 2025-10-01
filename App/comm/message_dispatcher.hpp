#pragma once

#include "../config/system_config.hpp"
#include "../motors/base/motor_interface.hpp"

extern "C" {
#include "mavlink/c_library_v2/common/mavlink.h"
}

#include <functional>
#include <unordered_map>
#include <vector>

// Forward declarations
namespace Storage {
    class ParameterStorage;
    extern ParameterStorage g_parameter_storage;
}

namespace Communication {

// Message handler interface
class IMessageHandler {
public:
    virtual ~IMessageHandler() = default;
    virtual Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) = 0;
    virtual bool canHandle(uint32_t msgId) const = 0;
    virtual uint8_t getPriority() const { return 0; } // 0 = highest priority
};

// Motor command dispatcher - converts MAVLink messages to motor commands
class MotorCommandDispatcher : public IMessageHandler {
private:
    Motors::MotorRegistry* motorRegistry_;
    std::unordered_map<uint8_t, uint8_t> channelToMotorMap_; // MAVLink channel -> Motor ID

public:
    explicit MotorCommandDispatcher(Motors::MotorRegistry* motorRegistry);

    // Configure channel to motor mapping
    void mapChannelToMotor(uint8_t channel, uint8_t motorId);
    void clearChannelMapping(uint8_t channel);

    // IMessageHandler implementation
    Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) override;
    bool canHandle(uint32_t msgId) const override;

private:
    Config::Result<Config::ErrorCode> handleRcChannelsOverride(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleManualControl(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleSetActuatorControlTarget(const mavlink_message_t& msg);

    // Helper methods
    float convertPWMToPosition(uint16_t pwmValue, float minPos = -90.0f, float maxPos = 90.0f);
    Motors::MotorCommand createMotorCommand(uint8_t motorId, float value, Motors::ControlMode mode);
};

// Parameter dispatcher - handles parameter requests with persistent storage
class ParameterDispatcher : public IMessageHandler {
private:
    struct Parameter {
        std::string name;
        float value;
        uint8_t type;
        std::function<void(float)> setter;
        std::function<float()> getter;
        bool persistent;  // Whether this parameter should be stored persistently
    };

    std::unordered_map<std::string, Parameter> parameters_;
    uint8_t systemId_;
    uint8_t componentId_;
    std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback_;
    Storage::ParameterStorage* persistentStorage_;
    bool storageEnabled_;

public:
    ParameterDispatcher(uint8_t systemId, uint8_t componentId,
                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback,
                       Storage::ParameterStorage* storage = &Storage::g_parameter_storage);

    // Parameter management
    void registerParameter(const std::string& name, float defaultValue, uint8_t type,
                          std::function<void(float)> setter = nullptr,
                          std::function<float()> getter = nullptr,
                          bool persistent = true);

    void setParameterValue(const std::string& name, float value);
    float getParameterValue(const std::string& name) const;

    // Persistent storage operations
    Config::Result<Config::ErrorCode> saveParameters();
    Config::Result<Config::ErrorCode> loadParameters();
    Config::Result<Config::ErrorCode> factoryReset();

    // Storage status
    bool isStorageEnabled() const { return storageEnabled_; }
    uint16_t getParameterCount() const;
    float getStorageHealth() const;

    // IMessageHandler implementation
    Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) override;
    bool canHandle(uint32_t msgId) const override;

    // Update method for periodic storage maintenance
    Config::Result<Config::ErrorCode> update();

private:
    Config::Result<Config::ErrorCode> handleParameterRequestList(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleParameterRequestRead(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleParameterSet(const mavlink_message_t& msg);

    Config::Result<Config::ErrorCode> sendParameterValue(const std::string& name, uint16_t index = 0);

    // Storage integration helpers
    Config::Result<Config::ErrorCode> initializeStorage();
    Config::Result<Config::ErrorCode> syncWithStorage();
    void updateStorageParameter(const std::string& name, float value);
};

// Telemetry dispatcher - handles telemetry generation and transmission
class TelemetryDispatcher {
private:
    Motors::MotorRegistry* motorRegistry_;
    uint8_t systemId_;
    uint8_t componentId_;
    std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback_;

    uint32_t lastHeartbeat_ = 0;
    uint32_t lastServoOutput_ = 0;
    uint32_t lastSystemStatus_ = 0;

public:
    TelemetryDispatcher(Motors::MotorRegistry* motorRegistry, uint8_t systemId, uint8_t componentId,
                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback);

    // Telemetry generation
    Config::Result<Config::ErrorCode> update();
    Config::Result<Config::ErrorCode> sendHeartbeat();
    Config::Result<Config::ErrorCode> sendServoOutputRaw();
    Config::Result<Config::ErrorCode> sendSystemStatus();
    Config::Result<Config::ErrorCode> sendAutopilotVersion();

private:
    uint8_t calculateSystemStatus();
    uint32_t getSystemUptimeMs();
};

// Main message dispatcher - coordinates all message handlers
class MessageDispatcher {
private:
    std::vector<std::unique_ptr<IMessageHandler>> handlers_;
    std::unordered_map<uint32_t, std::vector<IMessageHandler*>> messageRoutes_;

    // Statistics
    uint32_t messagesProcessed_ = 0;
    uint32_t messagesDropped_ = 0;
    uint32_t lastStatsTime_ = 0;

public:
    MessageDispatcher() = default;

    // Handler management
    void registerHandler(std::unique_ptr<IMessageHandler> handler);
    void unregisterHandler(IMessageHandler* handler);

    // Message processing
    Config::Result<Config::ErrorCode> dispatchMessage(const mavlink_message_t& msg);

    // Statistics
    uint32_t getProcessedCount() const { return messagesProcessed_; }
    uint32_t getDroppedCount() const { return messagesDropped_; }
    void resetStatistics();

private:
    void updateRoutes();
    void updateStatistics();
};

} // namespace Communication