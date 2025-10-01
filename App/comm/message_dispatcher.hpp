#pragma once

#include "../config/system_config.hpp"
#include "../motors/base/motor_interface.hpp"
#include "../storage/parameter_storage.hpp"

extern "C" {
#include "mavlink/c_library_v2/common/mavlink.h"
}

#include <functional>
#include <unordered_map>
#include <vector>


namespace Communication {

/**
 * @brief Interface for handling MAVLink messages.
 *
 * This class defines the contract for any class that wants to process incoming MAVLink messages.
 */
class IMessageHandler {
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IMessageHandler() = default;

    /**
     * @brief Handles an incoming MAVLink message.
     *
     * @param msg The MAVLink message to handle.
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    virtual Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) = 0;

    /**
     * @brief Checks if this handler can process a given message ID.
     *
     * @param msgId The ID of the MAVLink message.
     * @return true if the handler can process the message, false otherwise.
     */
    virtual bool canHandle(uint32_t msgId) const = 0;

    /**
     * @brief Gets the priority of the handler.
     *
     * Lower values indicate higher priority.
     *
     * @return The priority of the handler (0 is highest).
     */
    virtual uint8_t getPriority() const { return 0; }
};

/**
 * @brief Dispatches motor commands from MAVLink messages.
 *
 * This class converts MAVLink messages into motor commands and sends them to the motor registry.
 */
class MotorCommandDispatcher : public IMessageHandler {
private:
    Motors::MotorRegistry* motorRegistry_;
    std::unordered_map<uint8_t, uint8_t> channelToMotorMap_; // MAVLink channel -> Motor ID

public:
    /**
     * @brief Construct a new Motor Command Dispatcher object.
     *
     * @param motorRegistry Pointer to the motor registry.
     */
    explicit MotorCommandDispatcher(Motors::MotorRegistry* motorRegistry);

    /**
     * @brief Maps a MAVLink channel to a motor ID.
     *
     * @param channel The MAVLink channel.
     * @param motorId The ID of the motor.
     */
    void mapChannelToMotor(uint8_t channel, uint8_t motorId);

    /**
     * @brief Clears the mapping for a MAVLink channel.
     *
     * @param channel The MAVLink channel to clear.
     */
    void clearChannelMapping(uint8_t channel);

    /**
     * @brief Handles an incoming MAVLink message.
     *
     * @param msg The MAVLink message to handle.
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) override;

    /**
     * @brief Checks if this handler can process a given message ID.
     *
     * @param msgId The ID of the MAVLink message.
     * @return true if the handler can process the message, false otherwise.
     */
    bool canHandle(uint32_t msgId) const override;

private:
    Config::Result<Config::ErrorCode> handleRcChannelsOverride(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleManualControl(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleSetActuatorControlTarget(const mavlink_message_t& msg);

    float convertPWMToPosition(uint16_t pwmValue, float minPos = -90.0f, float maxPos = 90.0f);
    Motors::MotorCommand createMotorCommand(uint8_t motorId, float value, Motors::ControlMode mode);
};

/**
 * @brief Handles parameter requests and persistent storage.
 *
 * This class manages system parameters, including getting, setting, and saving them to persistent storage.
 */
class ParameterDispatcher : public IMessageHandler {
private:
    struct Parameter {
        std::string name;
        float value;
        uint8_t type;
        std::function<void(float)> setter;
        std::function<float()> getter;
        bool persistent;
        Storage::AuthorizationLevel required_auth_level;
        bool safety_critical;
        float min_value;
        float max_value;
    };

    std::unordered_map<std::string, Parameter> parameters_;
    uint8_t systemId_;
    uint8_t componentId_;
    std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback_;
    Storage::ParameterStorage* persistentStorage_;
    bool storageEnabled_;
    Storage::AuthorizationLevel current_auth_level_;
    bool safety_critical_enabled_;

public:
    /**
     * @brief Construct a new Parameter Dispatcher object.
     *
     * @param systemId The MAVLink system ID.
     * @param componentId The MAVLink component ID.
     * @param sendCallback Callback function to send a MAVLink message.
     * @param storage Pointer to the parameter storage object.
     */
    ParameterDispatcher(uint8_t systemId, uint8_t componentId,
                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback,
                       Storage::ParameterStorage* storage = &Storage::g_parameter_storage);

    /**
     * @brief Registers a new parameter.
     *
     * @param name The name of the parameter.
     * @param defaultValue The default value of the parameter.
     * @param type The MAVLink parameter type.
     * @param setter Optional setter function.
     * @param getter Optional getter function.
     * @param persistent Whether the parameter should be saved to persistent storage.
     * @param auth_level The required authorization level to modify the parameter.
     * @param safety_critical Whether the parameter is safety-critical.
     * @param min_value The minimum allowed value for the parameter.
     * @param max_value The maximum allowed value for the parameter.
     */
    void registerParameter(const std::string& name, float defaultValue, uint8_t type,
                          std::function<void(float)> setter = nullptr,
                          std::function<float()> getter = nullptr,
                          bool persistent = true,
                          Storage::AuthorizationLevel auth_level = Storage::AuthorizationLevel::USER,
                          bool safety_critical = false,
                          float min_value = -1000000.0f,
                          float max_value = 1000000.0f);

    /**
     * @brief Sets the value of a parameter.
     *
     * @param name The name of the parameter.
     * @param value The new value.
     */
    void setParameterValue(const std::string& name, float value);

    /**
     * @brief Gets the value of a parameter.
     *
     * @param name The name of the parameter.
     * @return The value of the parameter.
     */
    float getParameterValue(const std::string& name) const;

    /**
     * @brief Sets the current authorization level.
     *
     * @param level The new authorization level.
     */
    void setAuthorizationLevel(Storage::AuthorizationLevel level) { current_auth_level_ = level; }

    /**
     * @brief Gets the current authorization level.
     *
     * @return The current authorization level.
     */
    Storage::AuthorizationLevel getAuthorizationLevel() const { return current_auth_level_; }

    /**
     * @brief Enables or disables modification of safety-critical parameters.
     *
     * @param enabled true to enable, false to disable.
     */
    void setSafetyCriticalEnabled(bool enabled) { safety_critical_enabled_ = enabled; }

    /**
     * @brief Saves all persistent parameters to storage.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> saveParameters();

    /**
     * @brief Loads all persistent parameters from storage.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> loadParameters();

    /**
     * @brief Resets all parameters to their default values.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> factoryReset();

    /**
     * @brief Checks if persistent storage is enabled.
     *
     * @return true if storage is enabled, false otherwise.
     */
    bool isStorageEnabled() const { return storageEnabled_; }

    /**
     * @brief Gets the total number of registered parameters.
     *
     * @return The number of parameters.
     */
    uint16_t getParameterCount() const;

    /**
     * @brief Gets the health of the persistent storage.
     *
     * @return A value indicating storage health.
     */
    float getStorageHealth() const;

    /**
     * @brief Handles an incoming MAVLink message.
     *
     * @param msg The MAVLink message to handle.
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> handleMessage(const mavlink_message_t& msg) override;

    /**
     * @brief Checks if this handler can process a given message ID.
     *
     * @param msgId The ID of the MAVLink message.
     * @return true if the handler can process the message, false otherwise.
     */
    bool canHandle(uint32_t msgId) const override;

    /**
     * @brief Updates the parameter dispatcher, performing periodic maintenance.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> update();

private:
    Config::Result<Config::ErrorCode> handleParameterRequestList(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleParameterRequestRead(const mavlink_message_t& msg);
    Config::Result<Config::ErrorCode> handleParameterSet(const mavlink_message_t& msg);

    Config::Result<Config::ErrorCode> sendParameterValue(const std::string& name, uint16_t index = 0);

    Config::Result<Config::ErrorCode> initializeStorage();
    Config::Result<Config::ErrorCode> syncWithStorage();
    void updateStorageParameter(const std::string& name, float value);
};

/**
 * @brief Handles telemetry generation and transmission.
 *
 * This class is responsible for sending telemetry data, such as heartbeats and sensor data, over MAVLink.
 */
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
    /**
     * @brief Construct a new Telemetry Dispatcher object.
     *
     * @param motorRegistry Pointer to the motor registry.
     * @param systemId The MAVLink system ID.
     * @param componentId The MAVLink component ID.
     * @param sendCallback Callback function to send a MAVLink message.
     */
    TelemetryDispatcher(Motors::MotorRegistry* motorRegistry, uint8_t systemId, uint8_t componentId,
                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback);

    /**
     * @brief Updates the telemetry dispatcher, sending periodic messages.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> update();

    /**
     * @brief Sends a heartbeat message.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> sendHeartbeat();

    /**
     * @brief Sends raw servo output data.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> sendServoOutputRaw();

    /**
     * @brief Sends system status information.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> sendSystemStatus();

    /**
     * @brief Sends the autopilot version.
     *
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> sendAutopilotVersion();

private:
    uint8_t calculateSystemStatus();
    uint32_t getSystemUptimeMs();
};

/**
 * @brief Main message dispatcher that coordinates all message handlers.
 *
 * This class receives all incoming MAVLink messages and routes them to the appropriate handlers.
 */
class MessageDispatcher {
private:
    std::vector<std::unique_ptr<IMessageHandler>> handlers_;
    std::unordered_map<uint32_t, std::vector<IMessageHandler*>> messageRoutes_;

    uint32_t messagesProcessed_ = 0;
    uint32_t messagesDropped_ = 0;
    uint32_t lastStatsTime_ = 0;

public:
    /**
     * @brief Construct a new Message Dispatcher object.
     */
    MessageDispatcher() = default;

    /**
     * @brief Registers a new message handler.
     *
     * @param handler A unique pointer to the handler to register.
     */
    void registerHandler(std::unique_ptr<IMessageHandler> handler);

    /**
     * @brief Unregisters a message handler.
     *
     * @param handler A pointer to the handler to unregister.
     */
    void unregisterHandler(IMessageHandler* handler);

    /**
     * @brief Dispatches a MAVLink message to the appropriate handler.
     *
     * @param msg The MAVLink message to dispatch.
     * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
     */
    Config::Result<Config::ErrorCode> dispatchMessage(const mavlink_message_t& msg);

    /**
     * @brief Gets the number of processed messages.
     *
     * @return The number of processed messages.
     */
    uint32_t getProcessedCount() const { return messagesProcessed_; }

    /**
     * @brief Gets the number of dropped messages.
     *
     * @return The number of dropped messages.
     */
    uint32_t getDroppedCount() const { return messagesDropped_; }

    /**
     * @brief Resets the message statistics.
     */
    void resetStatistics();

private:
    void updateRoutes();
    void updateStatistics();
};

} // namespace Communication