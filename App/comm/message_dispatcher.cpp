#include "message_dispatcher.hpp"
#include "../storage/parameter_storage.hpp"
#include "../config/robot_config.hpp"

extern "C" {
#include "main.h" // For HAL_GetTick()
}

#include <algorithm>
#include <cstring>

namespace Communication {

// MotorCommandDispatcher implementation
MotorCommandDispatcher::MotorCommandDispatcher(Motors::MotorRegistry* motorRegistry)
    : motorRegistry_(motorRegistry) {

    // Set up default channel mappings (can be reconfigured)
    mapChannelToMotor(1, 1); // RC Channel 1 -> Motor ID 1
    mapChannelToMotor(2, 2); // RC Channel 2 -> Motor ID 2
    mapChannelToMotor(3, 3); // RC Channel 3 -> Motor ID 3
    mapChannelToMotor(4, 4); // RC Channel 4 -> Motor ID 4
}

void MotorCommandDispatcher::mapChannelToMotor(uint8_t channel, uint8_t motorId) {
    channelToMotorMap_[channel] = motorId;
}

void MotorCommandDispatcher::clearChannelMapping(uint8_t channel) {
    channelToMotorMap_.erase(channel);
}

Config::Result<Config::ErrorCode> MotorCommandDispatcher::handleMessage(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            return handleRcChannelsOverride(msg);
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            return handleManualControl(msg);
        case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
            return handleSetActuatorControlTarget(msg);
        default:
            return Config::ErrorCode::CONFIG_ERROR; // Unknown message
    }
}

bool MotorCommandDispatcher::canHandle(uint32_t msgId) const {
    return msgId == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE ||
           msgId == MAVLINK_MSG_ID_MANUAL_CONTROL ||
           msgId == MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET;
}

Config::Result<Config::ErrorCode> MotorCommandDispatcher::handleRcChannelsOverride(const mavlink_message_t& msg) {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_rc_channels_override_t rc_override;
    mavlink_msg_rc_channels_override_decode(&msg, &rc_override);

    // Array of channel values for easier processing
    uint16_t channels[] = {
        rc_override.chan1_raw, rc_override.chan2_raw, rc_override.chan3_raw, rc_override.chan4_raw,
        rc_override.chan5_raw, rc_override.chan6_raw, rc_override.chan7_raw, rc_override.chan8_raw
    };

    // Process each channel
    for (uint8_t channel = 1; channel <= 8; ++channel) {
        uint16_t pwmValue = channels[channel - 1];

        // Skip disabled channels (65535 = disabled, 0 = no change)
        if (pwmValue == 65535 || pwmValue == 0) {
            continue;
        }

        // Check if this channel is mapped to a motor
        auto it = channelToMotorMap_.find(channel);
        if (it == channelToMotorMap_.end()) {
            continue; // Channel not mapped
        }

        uint8_t motorId = it->second;

        // Convert PWM to position and create motor command
        float position = convertPWMToPosition(pwmValue);
        auto command = createMotorCommand(motorId, position, Motors::ControlMode::POSITION);

        // Send command to motor
        auto result = motorRegistry_->sendCommand(motorId, command);
        if (!result) {
            // Continue processing other channels even if one fails
            // Log error or handle as needed
        }
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> MotorCommandDispatcher::handleManualControl(const mavlink_message_t& msg) {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_manual_control_t manual_control;
    mavlink_msg_manual_control_decode(&msg, &manual_control);

    // Map manual control axes to motors
    // X axis -> Motor 1 (typically yaw or steering)
    // Y axis -> Motor 2 (typically pitch or throttle)
    // Z axis -> Motor 3 (typically roll)
    // R axis -> Motor 4 (typically additional control)

    struct AxisMapping {
        int16_t value;
        uint8_t motorId;
    } axes[] = {
        {manual_control.x, 1},
        {manual_control.y, 2},
        {manual_control.z, 3},
        {manual_control.r, 4}
    };

    for (const auto& axis : axes) {
        // Check if motor exists in mapping
        if (channelToMotorMap_.find(axis.motorId) == channelToMotorMap_.end()) {
            continue;
        }

        // Convert from [-1000, 1000] to position range
        float normalizedValue = axis.value / 1000.0f; // Range: [-1, 1]
        float position = normalizedValue * 90.0f; // Convert to degrees [-90, 90]

        auto command = createMotorCommand(axis.motorId, position, Motors::ControlMode::POSITION);

        auto result = motorRegistry_->sendCommand(axis.motorId, command);
        if (!result) {
            // Handle error as needed
        }
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> MotorCommandDispatcher::handleSetActuatorControlTarget(const mavlink_message_t& msg) {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_set_actuator_control_target_t actuator_target;
    mavlink_msg_set_actuator_control_target_decode(&msg, &actuator_target);

    // Process each actuator control (up to 8 controls)
    for (uint8_t i = 0; i < 8; ++i) {
        float controlValue = actuator_target.controls[i];

        // Skip unused controls (NaN or out of range)
        if (controlValue != controlValue || // Check for NaN
            controlValue < -1.0f || controlValue > 1.0f) {
            continue;
        }

        uint8_t motorId = i + 1; // Map control index to motor ID

        // Check if this motor is configured
        if (channelToMotorMap_.find(motorId) == channelToMotorMap_.end()) {
            continue;
        }

        // Convert normalized control [-1, 1] to position
        float position = controlValue * 90.0f; // Convert to degrees [-90, 90]

        auto command = createMotorCommand(motorId, position, Motors::ControlMode::POSITION);

        auto result = motorRegistry_->sendCommand(motorId, command);
        if (!result) {
            // Handle error as needed
        }
    }

    return Config::ErrorCode::OK;
}

float MotorCommandDispatcher::convertPWMToPosition(uint16_t pwmValue, float minPos, float maxPos) {
    // Standard RC PWM: 1000-2000 microseconds, center at 1500
    constexpr float PWM_MIN = 1000.0f;
    constexpr float PWM_MAX = 2000.0f;
    constexpr float PWM_CENTER = 1500.0f;

    // Clamp PWM value to valid range
    float clampedPWM = std::max(PWM_MIN, std::min(PWM_MAX, static_cast<float>(pwmValue)));

    // Normalize to [-1, 1] range (center = 0)
    float normalized = (clampedPWM - PWM_CENTER) / (PWM_MAX - PWM_CENTER) * 2.0f;

    // Scale to position range
    float range = (maxPos - minPos) / 2.0f;
    float center = (maxPos + minPos) / 2.0f;

    return center + normalized * range;
}

Motors::MotorCommand MotorCommandDispatcher::createMotorCommand(uint8_t motorId, float value, Motors::ControlMode mode) {
    Motors::MotorCommand cmd;
    cmd.motorId = motorId;
    cmd.mode = mode;
    cmd.targetValue = value;
    cmd.enable = true;
    cmd.timestamp = HAL_GetTick();
    return cmd;
}

// ParameterDispatcher implementation
ParameterDispatcher::ParameterDispatcher(uint8_t systemId, uint8_t componentId,
                                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback,
                                       Storage::ParameterStorage* storage)
    : systemId_(systemId), componentId_(componentId), sendCallback_(sendCallback),
      persistentStorage_(storage), storageEnabled_(false) {

    // Initialize persistent storage
    auto storage_result = initializeStorage();
    storageEnabled_ = storage_result.isOk();

    // Register default system parameters
    registerParameter("SYS_ID", systemId, MAV_PARAM_TYPE_UINT8);
    registerParameter("COMP_ID", componentId, MAV_PARAM_TYPE_UINT8);
    registerParameter("HEARTBEAT_RATE", 1.0f, MAV_PARAM_TYPE_REAL32);
    registerParameter("TELEMETRY_RATE", 10.0f, MAV_PARAM_TYPE_REAL32);
    registerParameter("AUTO_SAVE", 1.0f, MAV_PARAM_TYPE_UINT8);

    // Load saved parameters if storage is available
    if (storageEnabled_) {
        loadParameters();
    }
}

void ParameterDispatcher::registerParameter(const std::string& name, float defaultValue, uint8_t type,
                                          std::function<void(float)> setter,
                                          std::function<float()> getter,
                                          bool persistent) {
    Parameter param;
    param.name = name;
    param.value = defaultValue;
    param.type = type;
    param.setter = setter;
    param.getter = getter;
    param.persistent = persistent;

    parameters_[name] = param;

    // Register with persistent storage if enabled and parameter is persistent
    if (storageEnabled_ && persistent && persistentStorage_) {
        auto storage_result = persistentStorage_->registerParameterWithCallbacks(
            name.c_str(), defaultValue, setter, getter, type
        );
        if (storage_result.isError()) {
            // Fallback to non-persistent if storage registration fails
            param.persistent = false;
            parameters_[name] = param;
        }
    }
}

void ParameterDispatcher::setParameterValue(const std::string& name, float value) {
    auto it = parameters_.find(name);
    if (it != parameters_.end()) {
        it->second.value = value;
        if (it->second.setter) {
            it->second.setter(value);
        }

        // Update persistent storage if enabled and parameter is persistent
        if (storageEnabled_ && it->second.persistent && persistentStorage_) {
            updateStorageParameter(name, value);
        }
    }
}

float ParameterDispatcher::getParameterValue(const std::string& name) const {
    auto it = parameters_.find(name);
    if (it != parameters_.end()) {
        if (it->second.getter) {
            return it->second.getter();
        }

        // Try to get from persistent storage if parameter is persistent
        if (storageEnabled_ && it->second.persistent && persistentStorage_) {
            auto storage_result = persistentStorage_->getParameter(name.c_str());
            if (storage_result.isSuccess()) {
                return storage_result.value;
            }
        }

        return it->second.value;
    }
    return 0.0f;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::handleMessage(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            return handleParameterRequestList(msg);
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            return handleParameterRequestRead(msg);
        case MAVLINK_MSG_ID_PARAM_SET:
            return handleParameterSet(msg);
        default:
            return Config::ErrorCode::CONFIG_ERROR;
    }
}

bool ParameterDispatcher::canHandle(uint32_t msgId) const {
    return msgId == MAVLINK_MSG_ID_PARAM_REQUEST_LIST ||
           msgId == MAVLINK_MSG_ID_PARAM_REQUEST_READ ||
           msgId == MAVLINK_MSG_ID_PARAM_SET;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::handleParameterRequestList(const mavlink_message_t& msg) {
    mavlink_param_request_list_t request;
    mavlink_msg_param_request_list_decode(&msg, &request);

    // Send all parameters
    uint16_t index = 0;
    for (const auto& pair : parameters_) {
        auto result = sendParameterValue(pair.first, index++);
        if (!result) {
            return result.error();
        }
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::handleParameterRequestRead(const mavlink_message_t& msg) {
    mavlink_param_request_read_t request;
    mavlink_msg_param_request_read_decode(&msg, &request);

    std::string paramName(request.param_id);
    return sendParameterValue(paramName, request.param_index);
}

Config::Result<Config::ErrorCode> ParameterDispatcher::handleParameterSet(const mavlink_message_t& msg) {
    mavlink_param_set_t paramSet;
    mavlink_msg_param_set_decode(&msg, &paramSet);

    std::string paramName(paramSet.param_id);
    setParameterValue(paramName, paramSet.param_value);

    // Send confirmation
    return sendParameterValue(paramName);
}

Config::Result<Config::ErrorCode> ParameterDispatcher::sendParameterValue(const std::string& name, uint16_t index) {
    auto it = parameters_.find(name);
    if (it == parameters_.end()) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    const Parameter& param = it->second;

    mavlink_message_t msg;
    char paramId[17] = {0}; // MAVLink param_id is 16 chars + null terminator
    strncpy(paramId, param.name.c_str(), 16);

    mavlink_msg_param_value_pack(systemId_, componentId_, &msg,
                                paramId,
                                param.getter ? param.getter() : param.value,
                                param.type,
                                static_cast<uint16_t>(parameters_.size()),
                                index);

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

// New storage-related methods for ParameterDispatcher

Config::Result<Config::ErrorCode> ParameterDispatcher::saveParameters() {
    if (!storageEnabled_ || !persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto storage_result = persistentStorage_->saveParameters();
    return storage_result.isSuccess() ? Config::ErrorCode::OK : Config::ErrorCode::CONFIG_ERROR;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::loadParameters() {
    if (!storageEnabled_ || !persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto storage_result = persistentStorage_->loadParameters();
    if (storage_result.isSuccess()) {
        syncWithStorage();
        return Config::ErrorCode::OK;
    }
    return Config::ErrorCode::CONFIG_ERROR;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::factoryReset() {
    if (!storageEnabled_ || !persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto storage_result = persistentStorage_->factoryReset();
    if (storage_result.isSuccess()) {
        // Reset local parameters to defaults
        for (auto& pair : parameters_) {
            if (pair.second.persistent) {
                auto storage_result = persistentStorage_->getParameter(pair.first.c_str());
                if (storage_result.isSuccess()) {
                    pair.second.value = storage_result.value;
                }
            }
        }
        return Config::ErrorCode::OK;
    }
    return Config::ErrorCode::CONFIG_ERROR;
}

uint16_t ParameterDispatcher::getParameterCount() const {
    if (storageEnabled_ && persistentStorage_) {
        auto storage_result = persistentStorage_->getParameterCount();
        return storage_result.isSuccess() ? storage_result.value : static_cast<uint16_t>(parameters_.size());
    }
    return static_cast<uint16_t>(parameters_.size());
}

float ParameterDispatcher::getStorageHealth() const {
    if (storageEnabled_ && persistentStorage_) {
        auto storage_result = persistentStorage_->getStorageHealth();
        return storage_result.isSuccess() ? storage_result.value : -1.0f;
    }
    return -1.0f;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::update() {
    if (storageEnabled_ && persistentStorage_) {
        auto storage_result = persistentStorage_->update();
        return storage_result.isSuccess() ? Config::ErrorCode::OK : Config::ErrorCode::CONFIG_ERROR;
    }
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::initializeStorage() {
    if (!persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto storage_result = persistentStorage_->initialize();
    return storage_result.isSuccess() ? Config::ErrorCode::OK : Config::ErrorCode::CONFIG_ERROR;
}

Config::Result<Config::ErrorCode> ParameterDispatcher::syncWithStorage() {
    if (!storageEnabled_ || !persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    // Sync all persistent parameters from storage to local cache
    for (auto& pair : parameters_) {
        if (pair.second.persistent) {
            auto storage_result = persistentStorage_->getParameter(pair.first.c_str());
            if (storage_result.isSuccess()) {
                pair.second.value = storage_result.value;
            }
        }
    }

    return Config::ErrorCode::OK;
}

void ParameterDispatcher::updateStorageParameter(const std::string& name, float value) {
    if (storageEnabled_ && persistentStorage_) {
        auto storage_result = persistentStorage_->setParameter(name.c_str(), value);
        if (storage_result.isError()) {
            // Handle error silently or log it
        }
    }
}

// TelemetryDispatcher implementation
TelemetryDispatcher::TelemetryDispatcher(Motors::MotorRegistry* motorRegistry, uint8_t systemId, uint8_t componentId,
                                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback)
    : motorRegistry_(motorRegistry), systemId_(systemId), componentId_(componentId), sendCallback_(sendCallback) {
}

Config::Result<Config::ErrorCode> TelemetryDispatcher::update() {
    uint32_t currentTime = HAL_GetTick();

    // Send heartbeat every 1000ms
    if (currentTime - lastHeartbeat_ >= 1000) {
        auto result = sendHeartbeat();
        if (!result) {
            return result.error();
        }
        lastHeartbeat_ = currentTime;
    }

    // Send servo output every 100ms (10Hz)
    if (currentTime - lastServoOutput_ >= 100) {
        auto result = sendServoOutputRaw();
        if (!result) {
            return result.error();
        }
        lastServoOutput_ = currentTime;
    }

    // Send system status every 1000ms
    if (currentTime - lastSystemStatus_ >= 1000) {
        auto result = sendSystemStatus();
        if (!result) {
            return result.error();
        }
        lastSystemStatus_ = currentTime;
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> TelemetryDispatcher::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(systemId_, componentId_, &msg,
                              MAV_TYPE_GENERIC,
                              MAV_AUTOPILOT_GENERIC,
                              MAV_MODE_MANUAL_ARMED,
                              0, // custom_mode
                              calculateSystemStatus());

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> TelemetryDispatcher::sendServoOutputRaw() {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_message_t msg;

    // Get motor states and convert to servo values
    uint16_t servo_raw[8] = {0};

    for (uint8_t i = 1; i <= 8; ++i) {
        auto state = motorRegistry_->getMotorState(i);
        if (state.status == Motors::MotorStatus::OK) {
            // Convert position to PWM (assuming servo, -90 to +90 degrees -> 1000 to 2000 us)
            float normalizedPos = (state.currentPosition + 90.0f) / 180.0f; // [0, 1]
            servo_raw[i-1] = static_cast<uint16_t>(1000 + normalizedPos * 1000); // [1000, 2000]
        }
    }

    mavlink_msg_servo_output_raw_pack(systemId_, componentId_, &msg,
                                     getSystemUptimeMs(),
                                     0, // port
                                     servo_raw[0], servo_raw[1], servo_raw[2], servo_raw[3],
                                     servo_raw[4], servo_raw[5], servo_raw[6], servo_raw[7],
                                     0, 0, 0, 0, 0, 0, 0, 0); // servo9-16

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> TelemetryDispatcher::sendSystemStatus() {
    mavlink_message_t msg;

    mavlink_msg_sys_status_pack(systemId_, componentId_, &msg,
                               0, // onboard_control_sensors_present
                               0, // onboard_control_sensors_enabled
                               0, // onboard_control_sensors_health
                               0, // load (%)
                               12000, // voltage_battery (mV)
                               -1, // current_battery (cA)
                               -1, // battery_remaining (%)
                               0, // drop_rate_comm
                               0, // errors_comm
                               0, 0, 0, 0, // errors_count1-4
                               0, // onboard_control_sensors_present_extended
                               0, // onboard_control_sensors_enabled_extended
                               0); // onboard_control_sensors_health_extended

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> TelemetryDispatcher::sendAutopilotVersion() {
    mavlink_message_t msg;

    uint64_t capabilities = 0; // No special capabilities
    uint32_t flight_sw_version = 0x01000000; // Version 1.0.0
    uint32_t middleware_sw_version = 0x01000000;
    uint32_t os_sw_version = 0x01000000;
    uint32_t board_version = 1;
    uint8_t flight_custom_version[8] = {0}; // Empty custom version
    uint8_t middleware_custom_version[8] = {0}; // Empty custom version
    uint8_t os_custom_version[8] = {0}; // Empty custom version
    uint16_t vendor_id = 0; // No vendor ID
    uint16_t product_id = 0; // No product ID
    uint64_t uid = 0; // No unique ID
    uint8_t uid2[18] = {0}; // No extended UID

    mavlink_msg_autopilot_version_pack(systemId_, componentId_, &msg,
                                      capabilities,
                                      flight_sw_version,
                                      middleware_sw_version,
                                      os_sw_version,
                                      board_version,
                                      flight_custom_version,
                                      middleware_custom_version,
                                      os_custom_version,
                                      vendor_id,
                                      product_id,
                                      uid,
                                      uid2);

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

uint8_t TelemetryDispatcher::calculateSystemStatus() {
    // Simple status calculation - could be more sophisticated
    if (motorRegistry_) {
        // Check if any motors have errors
        for (uint8_t i = 1; i <= 8; ++i) {
            auto state = motorRegistry_->getMotorState(i);
            if (state.status == Motors::MotorStatus::EMERGENCY_STOP) {
                return MAV_STATE_EMERGENCY;
            }
            if (state.status != Motors::MotorStatus::OK &&
                state.status != Motors::MotorStatus::NOT_INITIALIZED) {
                return MAV_STATE_CRITICAL;
            }
        }
    }

    return MAV_STATE_ACTIVE;
}

uint32_t TelemetryDispatcher::getSystemUptimeMs() {
    return HAL_GetTick();
}

// MessageDispatcher implementation
void MessageDispatcher::registerHandler(std::unique_ptr<IMessageHandler> handler) {
    handlers_.push_back(std::move(handler));
    updateRoutes();
}

void MessageDispatcher::unregisterHandler(IMessageHandler* handler) {
    handlers_.erase(
        std::remove_if(handlers_.begin(), handlers_.end(),
                      [handler](const std::unique_ptr<IMessageHandler>& h) {
                          return h.get() == handler;
                      }),
        handlers_.end());
    updateRoutes();
}

Config::Result<Config::ErrorCode> MessageDispatcher::dispatchMessage(const mavlink_message_t& msg) {
    messagesProcessed_++;
    updateStatistics();

    auto it = messageRoutes_.find(msg.msgid);
    if (it == messageRoutes_.end()) {
        messagesDropped_++;
        return Config::ErrorCode::OUT_OF_RANGE; // No handler for this message
    }

    // Process message with all registered handlers for this message type
    // Handlers are sorted by priority (lowest number = highest priority)
    for (auto* handler : it->second) {
        auto result = handler->handleMessage(msg);
        if (result) {
            return Config::ErrorCode::OK; // Message handled successfully
        }
        // Continue to next handler if this one failed
    }

    messagesDropped_++;
    return Config::ErrorCode::COMMUNICATION_ERROR; // All handlers failed
}

void MessageDispatcher::resetStatistics() {
    messagesProcessed_ = 0;
    messagesDropped_ = 0;
    lastStatsTime_ = HAL_GetTick();
}

void MessageDispatcher::updateRoutes() {
    messageRoutes_.clear();

    // Build routing table for all message types
    for (auto& handler : handlers_) {
        // Check common MAVLink message IDs (could be expanded)
        uint32_t commonMsgIds[] = {
            MAVLINK_MSG_ID_HEARTBEAT,
            MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE,
            MAVLINK_MSG_ID_MANUAL_CONTROL,
            MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET,
            MAVLINK_MSG_ID_PARAM_REQUEST_LIST,
            MAVLINK_MSG_ID_PARAM_REQUEST_READ,
            MAVLINK_MSG_ID_PARAM_SET,
            MAVLINK_MSG_ID_COMMAND_LONG,
            MAVLINK_MSG_ID_COMMAND_INT
        };

        for (uint32_t msgId : commonMsgIds) {
            if (handler->canHandle(msgId)) {
                messageRoutes_[msgId].push_back(handler.get());
            }
        }
    }

    // Sort handlers by priority for each message type
    for (auto& route : messageRoutes_) {
        std::sort(route.second.begin(), route.second.end(),
                 [](const IMessageHandler* a, const IMessageHandler* b) {
                     return a->getPriority() < b->getPriority();
                 });
    }
}

void MessageDispatcher::updateStatistics() {
    uint32_t currentTime = HAL_GetTick();

    // Update statistics every 10 seconds
    if (currentTime - lastStatsTime_ >= 10000) {
        // Could log statistics here or update telemetry
        lastStatsTime_ = currentTime;
    }
}

} // namespace Communication