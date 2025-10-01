/**
 * @file message_dispatcher.cpp
 * @brief Implementation of the MAVLink message dispatcher and handlers.
 */

#include "message_dispatcher.hpp"
#include "../storage/parameter_storage.hpp"
#include "../config/robot_config.hpp"

extern "C" {
#include "main.h" // For HAL_GetTick()
}

#include <algorithm>
#include <cstring>

namespace Communication {

/**
 * @brief Construct a new MotorCommandDispatcher object.
 *
 * @param motorRegistry Pointer to the motor registry.
 */
MotorCommandDispatcher::MotorCommandDispatcher(Motors::MotorRegistry* motorRegistry)
    : motorRegistry_(motorRegistry) {
    // Set up default channel mappings (can be reconfigured)
    mapChannelToMotor(1, 1); // RC Channel 1 -> Motor ID 1
    mapChannelToMotor(2, 2); // RC Channel 2 -> Motor ID 2
    mapChannelToMotor(3, 3); // RC Channel 3 -> Motor ID 3
    mapChannelToMotor(4, 4); // RC Channel 4 -> Motor ID 4
}

/**
 * @brief Maps a MAVLink channel to a motor ID.
 *
 * @param channel The MAVLink channel.
 * @param motorId The ID of the motor.
 */
void MotorCommandDispatcher::mapChannelToMotor(uint8_t channel, uint8_t motorId) {
    channelToMotorMap_[channel] = motorId;
}

/**
 * @brief Clears the mapping for a MAVLink channel.
 *
 * @param channel The MAVLink channel to clear.
 */
void MotorCommandDispatcher::clearChannelMapping(uint8_t channel) {
    channelToMotorMap_.erase(channel);
}

/**
 * @brief Handles an incoming MAVLink message.
 *
 * @param msg The MAVLink message to handle.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
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

/**
 * @brief Checks if this handler can process a given message ID.
 *
 * @param msgId The ID of the MAVLink message.
 * @return true if the handler can process the message, false otherwise.
 */
bool MotorCommandDispatcher::canHandle(uint32_t msgId) const {
    return msgId == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE ||
           msgId == MAVLINK_MSG_ID_MANUAL_CONTROL ||
           msgId == MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET;
}

/**
 * @brief Handles the RC_CHANNELS_OVERRIDE MAVLink message.
 *
 * @param msg The MAVLink message.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> MotorCommandDispatcher::handleRcChannelsOverride(const mavlink_message_t& msg) {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_rc_channels_override_t rc_override;
    mavlink_msg_rc_channels_override_decode(&msg, &rc_override);

    uint16_t channels[] = {
        rc_override.chan1_raw, rc_override.chan2_raw, rc_override.chan3_raw, rc_override.chan4_raw,
        rc_override.chan5_raw, rc_override.chan6_raw, rc_override.chan7_raw, rc_override.chan8_raw
    };

    for (uint8_t channel = 1; channel <= 8; ++channel) {
        uint16_t pwmValue = channels[channel - 1];
        if (pwmValue == 65535 || pwmValue == 0) {
            continue;
        }

        auto it = channelToMotorMap_.find(channel);
        if (it == channelToMotorMap_.end()) {
            continue;
        }

        uint8_t motorId = it->second;
        float position = convertPWMToPosition(pwmValue);
        auto command = createMotorCommand(motorId, position, Motors::ControlMode::POSITION);
        motorRegistry_->sendCommand(motorId, command);
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Handles the MANUAL_CONTROL MAVLink message.
 *
 * @param msg The MAVLink message.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> MotorCommandDispatcher::handleManualControl(const mavlink_message_t& msg) {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_manual_control_t manual_control;
    mavlink_msg_manual_control_decode(&msg, &manual_control);

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
        if (channelToMotorMap_.find(axis.motorId) == channelToMotorMap_.end()) {
            continue;
        }

        float normalizedValue = axis.value / 1000.0f;
        float position = normalizedValue * 90.0f;
        auto command = createMotorCommand(axis.motorId, position, Motors::ControlMode::POSITION);
        motorRegistry_->sendCommand(axis.motorId, command);
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Handles the SET_ACTUATOR_CONTROL_TARGET MAVLink message.
 *
 * @param msg The MAVLink message.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> MotorCommandDispatcher::handleSetActuatorControlTarget(const mavlink_message_t& msg) {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_set_actuator_control_target_t actuator_target;
    mavlink_msg_set_actuator_control_target_decode(&msg, &actuator_target);

    for (uint8_t i = 0; i < 8; ++i) {
        float controlValue = actuator_target.controls[i];
        if (controlValue != controlValue || controlValue < -1.0f || controlValue > 1.0f) {
            continue;
        }

        uint8_t motorId = i + 1;
        if (channelToMotorMap_.find(motorId) == channelToMotorMap_.end()) {
            continue;
        }

        float position = controlValue * 90.0f;
        auto command = createMotorCommand(motorId, position, Motors::ControlMode::POSITION);
        motorRegistry_->sendCommand(motorId, command);
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Converts a PWM value to a position.
 *
 * @param pwmValue The PWM value (1000-2000).
 * @param minPos The minimum position.
 * @param maxPos The maximum position.
 * @return The converted position.
 */
float MotorCommandDispatcher::convertPWMToPosition(uint16_t pwmValue, float minPos, float maxPos) {
    constexpr float PWM_MIN = 1000.0f;
    constexpr float PWM_MAX = 2000.0f;
    constexpr float PWM_CENTER = 1500.0f;

    float clampedPWM = std::max(PWM_MIN, std::min(PWM_MAX, static_cast<float>(pwmValue)));
    float normalized = (clampedPWM - PWM_CENTER) / (PWM_MAX - PWM_CENTER) * 2.0f;
    float range = (maxPos - minPos) / 2.0f;
    float center = (maxPos + minPos) / 2.0f;

    return center + normalized * range;
}

/**
 * @brief Creates a new motor command.
 *
 * @param motorId The ID of the motor.
 * @param value The target value for the motor.
 * @param mode The control mode.
 * @return A new MotorCommand object.
 */
Motors::MotorCommand MotorCommandDispatcher::createMotorCommand(uint8_t motorId, float value, Motors::ControlMode mode) {
    Motors::MotorCommand cmd;
    cmd.motorId = motorId;
    cmd.mode = mode;
    cmd.targetValue = value;
    cmd.enable = true;
    cmd.timestamp = HAL_GetTick();
    return cmd;
}

/**
 * @brief Construct a new ParameterDispatcher object.
 *
 * @param systemId The MAVLink system ID.
 * @param componentId The MAVLink component ID.
 * @param sendCallback Callback function to send a MAVLink message.
 * @param storage Pointer to the parameter storage object.
 */
ParameterDispatcher::ParameterDispatcher(uint8_t systemId, uint8_t componentId,
                                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback,
                                       Storage::ParameterStorage* storage)
    : systemId_(systemId), componentId_(componentId), sendCallback_(sendCallback),
      persistentStorage_(storage), storageEnabled_(false),
      current_auth_level_(Storage::AuthorizationLevel::USER), safety_critical_enabled_(false) {
    auto storage_result = initializeStorage();
    storageEnabled_ = storage_result.isOk();

    registerParameter("SYS_ID", systemId, MAV_PARAM_TYPE_UINT8, nullptr, nullptr, true,
                     Storage::AuthorizationLevel::ADMIN, false, 1.0f, 255.0f);
    registerParameter("COMP_ID", componentId, MAV_PARAM_TYPE_UINT8, nullptr, nullptr, true,
                     Storage::AuthorizationLevel::ADMIN, false, 1.0f, 255.0f);
    registerParameter("HEARTBEAT_RATE", 1.0f, MAV_PARAM_TYPE_REAL32, nullptr, nullptr, true,
                     Storage::AuthorizationLevel::USER, false, 0.1f, 10.0f);
    registerParameter("TELEMETRY_RATE", 10.0f, MAV_PARAM_TYPE_REAL32, nullptr, nullptr, true,
                     Storage::AuthorizationLevel::USER, false, 1.0f, 100.0f);
    registerParameter("AUTO_SAVE", 1.0f, MAV_PARAM_TYPE_UINT8, nullptr, nullptr, true,
                     Storage::AuthorizationLevel::ADMIN, false, 0.0f, 1.0f);

    if (storageEnabled_) {
        loadParameters();
    }
}

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
void ParameterDispatcher::registerParameter(const std::string& name, float defaultValue, uint8_t type,
                                          std::function<void(float)> setter,
                                          std::function<float()> getter,
                                          bool persistent,
                                          Storage::AuthorizationLevel auth_level,
                                          bool safety_critical,
                                          float min_value,
                                          float max_value) {
    Parameter param;
    param.name = name;
    param.value = defaultValue;
    param.type = type;
    param.setter = setter;
    param.getter = getter;
    param.persistent = persistent;
    param.required_auth_level = auth_level;
    param.safety_critical = safety_critical;
    param.min_value = min_value;
    param.max_value = max_value;

    parameters_[name] = param;

    if (storageEnabled_ && persistent && persistentStorage_) {
        auto storage_result = persistentStorage_->registerParameterWithCallbacks(
            name.c_str(), defaultValue, setter, getter, type
        );
        if (storage_result.isError()) {
            param.persistent = false;
            parameters_[name] = param;
        }
    }
}

/**
 * @brief Sets the value of a parameter.
 *
 * @param name The name of the parameter.
 * @param value The new value.
 */
void ParameterDispatcher::setParameterValue(const std::string& name, float value) {
    auto it = parameters_.find(name);
    if (it != parameters_.end()) {
        const Parameter& param = it->second;

        if (static_cast<uint8_t>(current_auth_level_) < static_cast<uint8_t>(param.required_auth_level)) {
            return;
        }

        if (param.safety_critical && !safety_critical_enabled_) {
            return;
        }

        if (value < param.min_value || value > param.max_value) {
            return;
        }

        it->second.value = value;
        if (it->second.setter) {
            it->second.setter(value);
        }

        if (storageEnabled_ && it->second.persistent && persistentStorage_) {
            updateStorageParameter(name, value);
        }
    }
}

/**
 * @brief Gets the value of a parameter.
 *
 * @param name The name of the parameter.
 * @return The value of the parameter.
 */
float ParameterDispatcher::getParameterValue(const std::string& name) const {
    auto it = parameters_.find(name);
    if (it != parameters_.end()) {
        if (it->second.getter) {
            return it->second.getter();
        }

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

/**
 * @brief Handles an incoming MAVLink message.
 *
 * @param msg The MAVLink message to handle.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
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

/**
 * @brief Checks if this handler can process a given message ID.
 *
 * @param msgId The ID of the MAVLink message.
 * @return true if the handler can process the message, false otherwise.
 */
bool ParameterDispatcher::canHandle(uint32_t msgId) const {
    return msgId == MAVLINK_MSG_ID_PARAM_REQUEST_LIST ||
           msgId == MAVLINK_MSG_ID_PARAM_REQUEST_READ ||
           msgId == MAVLINK_MSG_ID_PARAM_SET;
}

/**
 * @brief Handles a parameter request list message.
 *
 * @param msg The MAVLink message.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::handleParameterRequestList(const mavlink_message_t& msg) {
    mavlink_param_request_list_t request;
    mavlink_msg_param_request_list_decode(&msg, &request);

    uint16_t index = 0;
    for (const auto& pair : parameters_) {
        auto result = sendParameterValue(pair.first, index++);
        if (!result) {
            return result.error();
        }
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Handles a parameter request read message.
 *
 * @param msg The MAVLink message.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::handleParameterRequestRead(const mavlink_message_t& msg) {
    mavlink_param_request_read_t request;
    mavlink_msg_param_request_read_decode(&msg, &request);

    std::string paramName(request.param_id);
    return sendParameterValue(paramName, request.param_index);
}

/**
 * @brief Handles a parameter set message.
 *
 * @param msg The MAVLink message.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::handleParameterSet(const mavlink_message_t& msg) {
    mavlink_param_set_t paramSet;
    mavlink_msg_param_set_decode(&msg, &paramSet);

    std::string paramName(paramSet.param_id);
    setParameterValue(paramName, paramSet.param_value);

    return sendParameterValue(paramName);
}

/**
 * @brief Sends the value of a parameter over MAVLink.
 *
 * @param name The name of the parameter.
 * @param index The index of the parameter.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::sendParameterValue(const std::string& name, uint16_t index) {
    auto it = parameters_.find(name);
    if (it == parameters_.end()) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    const Parameter& param = it->second;

    mavlink_message_t msg;
    char paramId[17] = {0};
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

/**
 * @brief Saves all persistent parameters to storage.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::saveParameters() {
    if (!storageEnabled_ || !persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto storage_result = persistentStorage_->saveParameters();
    return storage_result.isSuccess() ? Config::ErrorCode::OK : Config::ErrorCode::CONFIG_ERROR;
}

/**
 * @brief Loads all persistent parameters from storage.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
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

/**
 * @brief Resets all parameters to their default values.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::factoryReset() {
    if (!storageEnabled_ || !persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto storage_result = persistentStorage_->factoryReset();
    if (storage_result.isSuccess()) {
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

/**
 * @brief Gets the total number of registered parameters.
 *
 * @return The number of parameters.
 */
uint16_t ParameterDispatcher::getParameterCount() const {
    if (storageEnabled_ && persistentStorage_) {
        auto storage_result = persistentStorage_->getParameterCount();
        return storage_result.isSuccess() ? storage_result.value : static_cast<uint16_t>(parameters_.size());
    }
    return static_cast<uint16_t>(parameters_.size());
}

/**
 * @brief Gets the health of the persistent storage.
 *
 * @return A value indicating storage health.
 */
float ParameterDispatcher::getStorageHealth() const {
    if (storageEnabled_ && persistentStorage_) {
        auto storage_result = persistentStorage_->getStorageHealth();
        return storage_result.isSuccess() ? storage_result.value : -1.0f;
    }
    return -1.0f;
}

/**
 * @brief Updates the parameter dispatcher, performing periodic maintenance.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::update() {
    if (storageEnabled_ && persistentStorage_) {
        auto storage_result = persistentStorage_->update();
        return storage_result.isSuccess() ? Config::ErrorCode::OK : Config::ErrorCode::CONFIG_ERROR;
    }
    return Config::ErrorCode::OK;
}

/**
 * @brief Initializes the persistent storage.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::initializeStorage() {
    if (!persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto storage_result = persistentStorage_->initialize();
    return storage_result.isSuccess() ? Config::ErrorCode::OK : Config::ErrorCode::CONFIG_ERROR;
}

/**
 * @brief Synchronizes the local parameter cache with the persistent storage.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> ParameterDispatcher::syncWithStorage() {
    if (!storageEnabled_ || !persistentStorage_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

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

/**
 * @brief Updates a parameter in the persistent storage.
 *
 * @param name The name of the parameter.
 * @param value The new value of the parameter.
 */
void ParameterDispatcher::updateStorageParameter(const std::string& name, float value) {
    if (storageEnabled_ && persistentStorage_) {
        persistentStorage_->setParameter(name.c_str(), value);
    }
}

/**
 * @brief Construct a new TelemetryDispatcher object.
 *
 * @param motorRegistry Pointer to the motor registry.
 * @param systemId The MAVLink system ID.
 * @param componentId The MAVLink component ID.
 * @param sendCallback Callback function to send a MAVLink message.
 */
TelemetryDispatcher::TelemetryDispatcher(Motors::MotorRegistry* motorRegistry, uint8_t systemId, uint8_t componentId,
                                       std::function<Config::Result<Config::ErrorCode>(const mavlink_message_t&)> sendCallback)
    : motorRegistry_(motorRegistry), systemId_(systemId), componentId_(componentId), sendCallback_(sendCallback) {
}

/**
 * @brief Updates the telemetry dispatcher, sending periodic messages.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> TelemetryDispatcher::update() {
    uint32_t currentTime = HAL_GetTick();

    if (currentTime - lastHeartbeat_ >= 1000) {
        auto result = sendHeartbeat();
        if (!result) {
            return result.error();
        }
        lastHeartbeat_ = currentTime;
    }

    if (currentTime - lastServoOutput_ >= 100) {
        auto result = sendServoOutputRaw();
        if (!result) {
            return result.error();
        }
        lastServoOutput_ = currentTime;
    }

    if (currentTime - lastSystemStatus_ >= 1000) {
        auto result = sendSystemStatus();
        if (!result) {
            return result.error();
        }
        lastSystemStatus_ = currentTime;
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Sends a heartbeat message.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> TelemetryDispatcher::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(systemId_, componentId_, &msg,
                              MAV_TYPE_GENERIC,
                              MAV_AUTOPILOT_GENERIC,
                              MAV_MODE_MANUAL_ARMED,
                              0,
                              calculateSystemStatus());

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Sends raw servo output data.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> TelemetryDispatcher::sendServoOutputRaw() {
    if (!motorRegistry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    mavlink_message_t msg;
    uint16_t servo_raw[8] = {0};

    for (uint8_t i = 1; i <= 8; ++i) {
        auto state = motorRegistry_->getMotorState(i);
        if (state.status == Motors::MotorStatus::OK) {
            float normalizedPos = (state.currentPosition + 90.0f) / 180.0f;
            servo_raw[i-1] = static_cast<uint16_t>(1000 + normalizedPos * 1000);
        }
    }

    mavlink_msg_servo_output_raw_pack(systemId_, componentId_, &msg,
                                     getSystemUptimeMs(),
                                     0,
                                     servo_raw[0], servo_raw[1], servo_raw[2], servo_raw[3],
                                     servo_raw[4], servo_raw[5], servo_raw[6], servo_raw[7],
                                     0, 0, 0, 0, 0, 0, 0, 0);

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

/**
_raw
 * @brief Sends system status information.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> TelemetryDispatcher::sendSystemStatus() {
    mavlink_message_t msg;

    mavlink_msg_sys_status_pack(systemId_, componentId_, &msg,
                               0, 0, 0, 0, 12000, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    if (sendCallback_) {
        return sendCallback_(msg);
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Sends the autopilot version.
 *
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> TelemetryDispatcher::sendAutopilotVersion() {
    mavlink_message_t msg;
    uint64_t capabilities = 0;
    uint32_t flight_sw_version = 0x01000000;
    uint32_t middleware_sw_version = 0x01000000;
    uint32_t os_sw_version = 0x01000000;
    uint32_t board_version = 1;
    uint8_t flight_custom_version[8] = {0};
    uint8_t middleware_custom_version[8] = {0};
    uint8_t os_custom_version[8] = {0};
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    uint64_t uid = 0;
    uint8_t uid2[18] = {0};

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

/**
 * @brief Calculates the current system status.
 *
 * @return The system status as a MAV_STATE enum value.
 */
uint8_t TelemetryDispatcher::calculateSystemStatus() {
    if (motorRegistry_) {
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

/**
 * @brief Gets the system uptime in milliseconds.
 *
 * @return The system uptime.
 */
uint32_t TelemetryDispatcher::getSystemUptimeMs() {
    return HAL_GetTick();
}

/**
 * @brief Registers a new message handler.
 *
 * @param handler A unique pointer to the handler to register.
 */
void MessageDispatcher::registerHandler(std::unique_ptr<IMessageHandler> handler) {
    handlers_.push_back(std::move(handler));
    updateRoutes();
}

/**
 * @brief Unregisters a message handler.
 *
 * @param handler A pointer to the handler to unregister.
 */
void MessageDispatcher::unregisterHandler(IMessageHandler* handler) {
    handlers_.erase(
        std::remove_if(handlers_.begin(), handlers_.end(),
                      [handler](const std::unique_ptr<IMessageHandler>& h) {
                          return h.get() == handler;
                      }),
        handlers_.end());
    updateRoutes();
}

/**
 * @brief Dispatches a MAVLink message to the appropriate handler.
 *
 * @param msg The MAVLink message to dispatch.
 * @return Config::Result<Config::ErrorCode> indicating the result of the operation.
 */
Config::Result<Config::ErrorCode> MessageDispatcher::dispatchMessage(const mavlink_message_t& msg) {
    messagesProcessed_++;
    updateStatistics();

    auto it = messageRoutes_.find(msg.msgid);
    if (it == messageRoutes_.end()) {
        messagesDropped_++;
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    for (auto* handler : it->second) {
        auto result = handler->handleMessage(msg);
        if (result) {
            return Config::ErrorCode::OK;
        }
    }

    messagesDropped_++;
    return Config::ErrorCode::COMMUNICATION_ERROR;
}

/**
 * @brief Resets the message statistics.
 */
void MessageDispatcher::resetStatistics() {
    messagesProcessed_ = 0;
    messagesDropped_ = 0;
    lastStatsTime_ = HAL_GetTick();
}

/**
 * @brief Updates the message routing table.
 */
void MessageDispatcher::updateRoutes() {
    messageRoutes_.clear();

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

    for (auto& handler : handlers_) {
        for (uint32_t msgId : commonMsgIds) {
            if (handler->canHandle(msgId)) {
                messageRoutes_[msgId].push_back(handler.get());
            }
        }
    }

    for (auto& route : messageRoutes_) {
        std::sort(route.second.begin(), route.second.end(),
                 [](const IMessageHandler* a, const IMessageHandler* b) {
                     return a->getPriority() < b->getPriority();
                 });
    }
}

/**
 * @brief Updates the message statistics.
 */
void MessageDispatcher::updateStatistics() {
    uint32_t currentTime = HAL_GetTick();
    if (currentTime - lastStatsTime_ >= 10000) {
        lastStatsTime_ = currentTime;
    }
}

} // namespace Communication