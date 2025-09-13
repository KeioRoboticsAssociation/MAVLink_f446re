#include "MAVLinkRoboMasterController.hpp"
#include <cstring>
#include <cstdio>

MAVLinkRoboMasterController::MAVLinkRoboMasterController()
    : uart_(nullptr), system_id_(1), can_manager_(nullptr),
      last_heartbeat_(0), last_telemetry_(0), telemetry_rate_ms_(TELEMETRY_RATE_MS),
      telemetry_enabled_(true), parameter_count_(0) {

    // Initialize motor registry
    for (int i = 0; i < MAX_MOTORS; i++) {
        motors_[i] = nullptr;
        motor_registered_[i] = false;
    }

    // Initialize parameters
    initializeParameters();
}

MAVLinkRoboMasterController::~MAVLinkRoboMasterController() {
    // Clean up allocated parameter names
    for (uint16_t i = 0; i < parameter_count_; i++) {
        delete[] parameters_[i].name;
    }
}

void MAVLinkRoboMasterController::init(UART_HandleTypeDef* uart, uint8_t system_id) {
    uart_ = uart;
    system_id_ = system_id;
}

void MAVLinkRoboMasterController::addMotor(RoboMasterMotor* motor, uint8_t motor_id) {
    if (motor_id < 1 || motor_id > MAX_MOTORS || motor == nullptr) {
        return;
    }
    
    uint8_t index = motor_id - 1;
    motors_[index] = motor;
    motor_registered_[index] = true;
    
    // Register parameters for this motor
    registerMotorParameters(motor_id);
}

void MAVLinkRoboMasterController::setCANManager(RoboMasterCANManager* can_manager) {
    can_manager_ = can_manager;
}

void MAVLinkRoboMasterController::update() {
    uint32_t now = getCurrentTimeMs();
    
    // Send heartbeat
    if (now - last_heartbeat_ >= HEARTBEAT_RATE_MS) {
        sendHeartbeat();
        last_heartbeat_ = now;
    }
    
    // Send telemetry if enabled
    if (telemetry_enabled_ && (now - last_telemetry_ >= telemetry_rate_ms_)) {
        sendMotorTelemetry();
        last_telemetry_ = now;
    }
}

void MAVLinkRoboMasterController::processReceivedByte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg_, &rx_status_)) {
        handleMessage(&rx_msg_);
    }
}

void MAVLinkRoboMasterController::handleMessage(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg);
            break;
            
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            handleParamRequestRead(msg);
            break;
            
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            handleParamRequestList(msg);
            break;
            
        case MAVLINK_MSG_ID_PARAM_SET:
            handleParamSet(msg);
            break;
            
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleCommandLong(msg);
            break;
            
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            handleManualControl(msg);
            break;
            
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handleRCChannelsOverride(msg);
            break;
            
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
            handleRequestDataStream(msg);
            break;
            
        // Custom RoboMaster messages
        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL:
            handleMotorControl(msg);
            break;
            
        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG:
            handleMotorConfigSet(msg);
            break;

        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS:
            handleMotorStatusRequest(msg);
            break;

        case MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY:
            handleMotorConfigGet(msg);
            break;
            
        default:
            // Unknown message
            break;
    }
}

void MAVLinkRoboMasterController::handleParamRequestRead(mavlink_message_t* msg) {
    mavlink_param_request_read_t request;
    mavlink_msg_param_request_read_decode(msg, &request);
    
    if (request.target_system != system_id_) {
        return;
    }
    
    // Find parameter by name or index
    int16_t param_index = -1;
    if (request.param_id[0] != '\0') {
        param_index = findParameterIndex(request.param_id);
    } else {
        param_index = findParameterIndex(request.param_index);
    }
    
    if (param_index >= 0) {
        sendParameterValue(param_index);
    }
}

void MAVLinkRoboMasterController::handleParamRequestList(mavlink_message_t* msg) {
    mavlink_param_request_list_t request;
    mavlink_msg_param_request_list_decode(msg, &request);
    
    if (request.target_system != system_id_) {
        return;
    }
    
    // Send all parameters
    for (uint16_t i = 0; i < parameter_count_; i++) {
        sendParameterValue(i);
        HAL_Delay(5);  // Small delay to avoid overwhelming the bus
    }
}

void MAVLinkRoboMasterController::handleParamSet(mavlink_message_t* msg) {
    mavlink_param_set_t param_set;
    mavlink_msg_param_set_decode(msg, &param_set);
    
    if (param_set.target_system != system_id_) {
        return;
    }
    
    // Find and set parameter
    int16_t param_index = findParameterIndex(param_set.param_id);
    if (param_index >= 0) {
        bool success = setParameterValue(param_index, param_set.param_value);
        if (success) {
            sendParameterValue(param_index);
            
            // Auto-save critical parameters
            if (strstr(param_set.param_id, "SAVE") || strstr(param_set.param_id, "KP") || 
                strstr(param_set.param_id, "KI") || strstr(param_set.param_id, "KD")) {
                saveParametersToFlash();
            }
        }
    }
}

void MAVLinkRoboMasterController::handleCommandLong(mavlink_message_t* msg) {
    mavlink_command_long_t command;
    mavlink_msg_command_long_decode(msg, &command);
    
    if (command.target_system != system_id_) {
        return;
    }
    
    uint8_t result = MAV_RESULT_ACCEPTED;
    
    switch (static_cast<uint16_t>(command.command)) {
        case MAV_CMD_PREFLIGHT_STORAGE: {
            // param1: 0=read, 1=write, 2=clear
            if (command.param1 == 0) {
                loadParametersFromFlash();
            } else if (command.param1 == 1) {
                saveParametersToFlash();
            } else if (command.param1 == 2) {
                // Reset to defaults
                initializeParameters();
            }
            break;
        }
        
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            // param1: 1=arm, 0=disarm
            bool enable = (command.param1 > 0.5f);
            uint8_t motor_id = static_cast<uint8_t>(command.param2);
            
            if (motor_id == 0) {
                // Enable/disable all motors
                for (int i = 0; i < MAX_MOTORS; i++) {
                    if (motor_registered_[i] && motors_[i] != nullptr) {
                        motors_[i]->setEnabled(enable);
                    }
                }
            } else {
                RoboMasterMotor* motor = findMotor(motor_id);
                if (motor != nullptr) {
                    motor->setEnabled(enable);
                }
            }
            break;
        }
        
        case MAV_CMD_DO_MOTOR_TEST: {
            // Motor test command
            uint8_t motor_id = static_cast<uint8_t>(command.param1);
            float test_value = command.param2;
            uint8_t test_type = static_cast<uint8_t>(command.param3);
            
            RoboMasterMotor* motor = findMotor(motor_id);
            if (motor != nullptr) {
                switch (test_type) {
                    case 0: // Current test
                        motor->setCurrent(static_cast<int16_t>(test_value));
                        break;
                    case 1: // Velocity test
                        motor->setVelocityRPS(test_value);
                        break;
                    case 2: // Position test
                        motor->setPositionRad(test_value);
                        break;
                }
            }
            break;
        }
        
        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
    }
    
    sendCommandAck(static_cast<uint16_t>(command.command), result);
}

void MAVLinkRoboMasterController::handleManualControl(mavlink_message_t* msg) {
    mavlink_manual_control_t manual_control;
    mavlink_msg_manual_control_decode(msg, &manual_control);
    
    if (manual_control.target != system_id_) {
        return;
    }
    
    // Map manual control to motor commands
    // X/Y/Z/R axes to motors 1-4
    float scale = 50.0f / 1000.0f;  // Scale to ±50 RPS
    
    if (motor_registered_[0] && motors_[0] != nullptr) {
        motors_[0]->setVelocityRPS(manual_control.x * scale);
    }
    if (motor_registered_[1] && motors_[1] != nullptr) {
        motors_[1]->setVelocityRPS(manual_control.y * scale);
    }
    if (motor_registered_[2] && motors_[2] != nullptr) {
        motors_[2]->setVelocityRPS(manual_control.z * scale);
    }
    if (motor_registered_[3] && motors_[3] != nullptr) {
        motors_[3]->setVelocityRPS(manual_control.r * scale);
    }
}

void MAVLinkRoboMasterController::sendHeartbeat() {
    mavlink_message_t msg;
    
    mavlink_msg_heartbeat_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                              MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
                              MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
    
    sendMessage(&msg);
}

void MAVLinkRoboMasterController::sendMotorTelemetry() {
    for (uint8_t motor_id = 1; motor_id <= MAX_MOTORS; motor_id++) {
        if (motor_registered_[motor_id - 1] && motors_[motor_id - 1] != nullptr) {
            sendMotorStatus(motor_id);
            HAL_Delay(1);  // Small delay between motors
        }
    }
}

void MAVLinkRoboMasterController::sendMotorStatus(uint8_t motor_id) {
    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor == nullptr) {
        return;
    }

    const RoboMasterState& state = motor->getState();

    // Send comprehensive motor status as custom MAVLink message
    mavlink_message_t msg;
    uint8_t payload[64] = {0};
    uint16_t payload_len = 0;

    // Motor ID
    payload[payload_len++] = motor_id;

    // Current measurements (position, velocity, current, temperature)
    memcpy(&payload[payload_len], &state.currentPositionRad, sizeof(float));
    payload_len += sizeof(float);

    memcpy(&payload[payload_len], &state.currentVelocityRPS, sizeof(float));
    payload_len += sizeof(float);

    memcpy(&payload[payload_len], &state.currentMilliamps, sizeof(int16_t));
    payload_len += sizeof(int16_t);

    payload[payload_len++] = state.temperatureCelsius;

    // Target values
    memcpy(&payload[payload_len], &state.targetPositionRad, sizeof(float));
    payload_len += sizeof(float);

    memcpy(&payload[payload_len], &state.targetVelocityRPS, sizeof(float));
    payload_len += sizeof(float);

    memcpy(&payload[payload_len], &state.targetCurrent, sizeof(int16_t));
    payload_len += sizeof(int16_t);

    // Status flags
    payload[payload_len++] = static_cast<uint8_t>(state.controlMode);
    payload[payload_len++] = state.enabled ? 1 : 0;
    payload[payload_len++] = static_cast<uint8_t>(state.status);

    // Statistics (pack as uint16_t to save space)
    uint16_t error_count = static_cast<uint16_t>(state.errorCount > 65535 ? 65535 : state.errorCount);
    uint16_t timeout_count = static_cast<uint16_t>(state.timeoutCount > 65535 ? 65535 : state.timeoutCount);
    uint16_t overheat_count = static_cast<uint16_t>(state.overHeatCount > 65535 ? 65535 : state.overHeatCount);

    memcpy(&payload[payload_len], &error_count, sizeof(uint16_t));
    payload_len += sizeof(uint16_t);

    memcpy(&payload[payload_len], &timeout_count, sizeof(uint16_t));
    payload_len += sizeof(uint16_t);

    memcpy(&payload[payload_len], &overheat_count, sizeof(uint16_t));
    payload_len += sizeof(uint16_t);

    // Timestamps (use relative time in ms, pack as uint32_t)
    uint32_t current_time = getCurrentTimeMs();
    uint32_t last_command_age = current_time - state.lastCommandTime;
    uint32_t last_feedback_age = current_time - state.lastFeedbackTime;

    memcpy(&payload[payload_len], &last_command_age, sizeof(uint32_t));
    payload_len += sizeof(uint32_t);

    memcpy(&payload[payload_len], &last_feedback_age, sizeof(uint32_t));
    payload_len += sizeof(uint32_t);

    // Create custom status message
    msg.msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS;
    msg.len = payload_len;
    msg.sysid = system_id_;
    msg.compid = MAV_COMP_ID_AUTOPILOT1;
    memcpy(msg.payload64, payload, payload_len);

    sendMessage(&msg);

    // Also send as status text for human readability
    char status_text[100];
    snprintf(status_text, sizeof(status_text),
             "M%d: pos=%.1f vel=%.1f cur=%d temp=%d %s %s",
             motor_id,
             state.currentPositionRad,
             state.currentVelocityRPS,
             state.currentMilliamps,
             state.temperatureCelsius,
             state.enabled ? "EN" : "DIS",
             state.status == RoboMasterStatus::OK ? "OK" : "ERR");

    sendStatusText(MAV_SEVERITY_INFO, status_text);
}

void MAVLinkRoboMasterController::sendParameterValue(uint16_t param_index) {
    if (param_index >= parameter_count_) {
        return;
    }

    const ParameterInfo& param = parameters_[param_index];

    // Get parameter value based on type
    float param_value;
    switch (param.type) {
        case ParamType::FLOAT:
            param_value = *static_cast<float*>(param.value_ptr);
            break;
        case ParamType::INT16:
            param_value = static_cast<float>(*static_cast<int16_t*>(param.value_ptr));
            break;
        case ParamType::UINT8:
            param_value = static_cast<float>(*static_cast<uint8_t*>(param.value_ptr));
            break;
        case ParamType::UINT32:
            param_value = static_cast<float>(*static_cast<uint32_t*>(param.value_ptr));
            break;
        default:
            param_value = 0.0f;
            break;
    }

    mavlink_message_t msg;
    mavlink_msg_param_value_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                                param.name, param_value, MAVLINK_TYPE_FLOAT,
                                parameter_count_, param_index);

    sendMessage(&msg);
}

void MAVLinkRoboMasterController::sendCommandAck(uint16_t command, uint8_t result) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                                command, result, 0, 0, 0, 0);
    sendMessage(&msg);
}

void MAVLinkRoboMasterController::initializeParameters() {
    parameter_count_ = 0;
    
    // System parameters will be added when motors are registered
}

void MAVLinkRoboMasterController::registerMotorParameters(uint8_t motor_id) {
    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor == nullptr || parameter_count_ >= MAX_PARAMETERS - 20) {
        return;
    }
    
    char param_name[17];
    
    // Get reference to motor configuration 
    RoboMasterConfig& config = motor->getConfigReference();
    
    // Control parameters
    snprintf(param_name, sizeof(param_name), "M%d_POS_KP", motor_id);
    char* name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.positionKp, ParamType::FLOAT, 0.0f, 1000.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_POS_KI", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.positionKi, ParamType::FLOAT, 0.0f, 100.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_POS_KD", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.positionKd, ParamType::FLOAT, 0.0f, 100.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_VEL_KP", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.velocityKp, ParamType::FLOAT, 0.0f, 1000.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_VEL_KI", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.velocityKi, ParamType::FLOAT, 0.0f, 100.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_VEL_KD", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.velocityKd, ParamType::FLOAT, 0.0f, 100.0f, motor_id};
    parameter_count_++;

    // Limit parameters
    snprintf(param_name, sizeof(param_name), "M%d_MAX_VEL", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.maxVelocityRPS, ParamType::FLOAT, 1.0f, 200.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_MAX_CUR", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.maxCurrent, ParamType::INT16, 100.0f, 20000.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_MAX_TEMP", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.maxTemperature, ParamType::UINT8, 40.0f, 100.0f, motor_id};
    parameter_count_++;

    snprintf(param_name, sizeof(param_name), "M%d_TIMEOUT", motor_id);
    name_copy = new char[strlen(param_name) + 1];
    strcpy(name_copy, param_name);
    parameters_[parameter_count_] = {name_copy, &config.watchdogTimeoutMs, ParamType::UINT32, 50.0f, 5000.0f, motor_id};
    parameter_count_++;
}

int16_t MAVLinkRoboMasterController::findParameterIndex(const char* param_name) const {
    for (uint16_t i = 0; i < parameter_count_; i++) {
        if (strcmp(parameters_[i].name, param_name) == 0) {
            return i;
        }
    }
    return -1;
}

int16_t MAVLinkRoboMasterController::findParameterIndex(uint16_t param_index) const {
    if (param_index < parameter_count_) {
        return param_index;
    }
    return -1;
}

bool MAVLinkRoboMasterController::setParameterValue(uint16_t param_index, float value) {
    if (param_index >= parameter_count_) {
        return false;
    }

    const ParameterInfo& param = parameters_[param_index];

    // Validate range
    if (value < param.min_value || value > param.max_value) {
        return false;
    }

    // Set the value based on type
    switch (param.type) {
        case ParamType::FLOAT:
            *static_cast<float*>(param.value_ptr) = value;
            break;
        case ParamType::INT16:
            *static_cast<int16_t*>(param.value_ptr) = static_cast<int16_t>(value);
            break;
        case ParamType::UINT8:
            *static_cast<uint8_t*>(param.value_ptr) = static_cast<uint8_t>(value);
            break;
        case ParamType::UINT32:
            *static_cast<uint32_t*>(param.value_ptr) = static_cast<uint32_t>(value);
            break;
        default:
            return false;
    }

    // Apply to motor if needed
    RoboMasterMotor* motor = findMotor(param.motor_id);
    if (motor != nullptr) {
        // This would require a method to update motor configuration
        // motor->updateParameter(param.name, value);
    }

    return true;
}

void MAVLinkRoboMasterController::emergencyStop() {
    if (can_manager_ != nullptr) {
        can_manager_->emergencyStop();
    }
    
    sendStatusText(MAV_SEVERITY_WARNING, "Emergency stop activated");
}

RoboMasterMotor* MAVLinkRoboMasterController::findMotor(uint8_t motor_id) {
    if (motor_id < 1 || motor_id > MAX_MOTORS) {
        return nullptr;
    }
    
    uint8_t index = motor_id - 1;
    if (motor_registered_[index]) {
        return motors_[index];
    }
    
    return nullptr;
}

void MAVLinkRoboMasterController::sendMessage(mavlink_message_t* msg) {
    if (uart_ == nullptr) return;
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    
    HAL_UART_Transmit(uart_, buf, len, 100);
}

void MAVLinkRoboMasterController::sendStatusText(uint8_t severity, const char* text) {
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                               severity, text, 0, 0);
    sendMessage(&msg);
}

uint32_t MAVLinkRoboMasterController::getCurrentTimeMs() const {
    return HAL_GetTick();
}

// Placeholder implementations for save/load (would need flash/EEPROM integration)
void MAVLinkRoboMasterController::saveParametersToFlash() {
    // TODO: Implement flash storage
    sendStatusText(MAV_SEVERITY_INFO, "Parameters saved");
}

void MAVLinkRoboMasterController::loadParametersFromFlash() {
    // TODO: Implement flash loading  
    sendStatusText(MAV_SEVERITY_INFO, "Parameters loaded");
}

// Missing method implementations

void MAVLinkRoboMasterController::handleMotorConfigGet(mavlink_message_t* msg) {
    // Handle custom RoboMaster motor configuration request

    // Validate message length
    if (msg->len < 1) {
        sendStatusText(MAV_SEVERITY_ERROR, "Config get msg too short");
        return;
    }

    const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg->payload64);
    uint8_t motor_id = payload[0];

    // Validate motor ID
    if (motor_id < 1 || motor_id > MAX_MOTORS) {
        sendStatusText(MAV_SEVERITY_ERROR, "Invalid motor ID for config get");
        return;
    }

    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor == nullptr) {
        char error_msg[50];
        snprintf(error_msg, sizeof(error_msg), "Motor %d not found for config get", motor_id);
        sendStatusText(MAV_SEVERITY_ERROR, error_msg);
        return;
    }

    // Send motor configuration back
    const RoboMasterConfig& config = motor->getConfig();
    sendMotorConfig(motor_id, config);
}

void MAVLinkRoboMasterController::handleMotorStatusRequest(mavlink_message_t* msg) {
    // Handle motor status request

    // Validate message length
    if (msg->len < 1) {
        sendStatusText(MAV_SEVERITY_ERROR, "Status request msg too short");
        return;
    }

    const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg->payload64);
    uint8_t motor_id = payload[0];

    if (motor_id == 0) {
        // Send status for all registered motors
        uint8_t motors_sent = 0;
        for (uint8_t i = 1; i <= MAX_MOTORS; i++) {
            if (motor_registered_[i - 1] && motors_[i - 1] != nullptr) {
                sendMotorStatus(i);
                motors_sent++;
                HAL_Delay(5); // Small delay to prevent bus saturation
            }
        }

        if (motors_sent == 0) {
            sendStatusText(MAV_SEVERITY_INFO, "No motors registered");
        }
    } else {
        // Validate specific motor ID
        if (motor_id > MAX_MOTORS) {
            sendStatusText(MAV_SEVERITY_ERROR, "Invalid motor ID for status");
            return;
        }

        if (!motor_registered_[motor_id - 1] || motors_[motor_id - 1] == nullptr) {
            char error_msg[50];
            snprintf(error_msg, sizeof(error_msg), "Motor %d not registered", motor_id);
            sendStatusText(MAV_SEVERITY_ERROR, error_msg);
            return;
        }

        sendMotorStatus(motor_id);
    }
}

void MAVLinkRoboMasterController::sendParameterValue(const char* param_name, float value) {
    int16_t param_index = findParameterIndex(param_name);
    if (param_index >= 0) {
        sendParameterValue(static_cast<uint16_t>(param_index));
    }
}

bool MAVLinkRoboMasterController::setParameterValue(const char* param_name, float value) {
    int16_t param_index = findParameterIndex(param_name);
    if (param_index >= 0) {
        return setParameterValue(static_cast<uint16_t>(param_index), value);
    }
    return false;
}

void MAVLinkRoboMasterController::saveMotorConfig(uint8_t motor_id) {
    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor != nullptr) {
        // TODO: Implement motor-specific configuration saving
        char message[50];
        snprintf(message, sizeof(message), "Motor %d config saved", motor_id);
        sendStatusText(MAV_SEVERITY_INFO, message);
    }
}

void MAVLinkRoboMasterController::loadMotorConfig(uint8_t motor_id) {
    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor != nullptr) {
        // TODO: Implement motor-specific configuration loading
        char message[50];
        snprintf(message, sizeof(message), "Motor %d config loaded", motor_id);
        sendStatusText(MAV_SEVERITY_INFO, message);
    }
}

void MAVLinkRoboMasterController::resetMotorConfig(uint8_t motor_id) {
    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor != nullptr) {
        RoboMasterConfig default_config;
        motor->setConfig(default_config);
        char message[50];
        snprintf(message, sizeof(message), "Motor %d config reset", motor_id);
        sendStatusText(MAV_SEVERITY_INFO, message);
    }
}

void MAVLinkRoboMasterController::emergencyStopMotor(uint8_t motor_id) {
    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor != nullptr) {
        motor->setEnabled(false);
        motor->setCurrent(0);
        char message[50];
        snprintf(message, sizeof(message), "Motor %d emergency stop", motor_id);
        sendStatusText(MAV_SEVERITY_WARNING, message);
    }
}

bool MAVLinkRoboMasterController::validateMotorCommand(uint8_t motor_id, float value, const char* param_type) {
    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor == nullptr) {
        return false;
    }

    const RoboMasterConfig& config = motor->getConfig();

    if (strcmp(param_type, "velocity") == 0) {
        return (value >= -config.maxVelocityRPS && value <= config.maxVelocityRPS);
    } else if (strcmp(param_type, "current") == 0) {
        return (value >= config.minCurrent && value <= config.maxCurrent);
    } else if (strcmp(param_type, "position") == 0) {
        if (config.positionLimitsEnabled) {
            return (value >= config.minPositionRad && value <= config.maxPositionRad);
        }
        return true; // No limits enabled
    }

    return false;
}

void MAVLinkRoboMasterController::sendMotorConfig(uint8_t motor_id, const RoboMasterConfig& config) {
    // Send motor configuration as custom MAVLink message
    mavlink_message_t msg;

    // Create a custom message with motor config data
    uint8_t payload[64] = {0};
    uint16_t payload_len = 0;

    // Validate motor ID before sending
    if (motor_id < 1 || motor_id > MAX_MOTORS) {
        sendStatusText(MAV_SEVERITY_ERROR, "Invalid motor ID for config send");
        return;
    }

    payload[payload_len++] = motor_id;

    // Pack essential config parameters with bounds checking
    if (payload_len + sizeof(float) > sizeof(payload)) {
        sendStatusText(MAV_SEVERITY_ERROR, "Config payload too large");
        return;
    }
    memcpy(&payload[payload_len], &config.maxVelocityRPS, sizeof(float));
    payload_len += sizeof(float);

    if (payload_len + sizeof(int16_t) > sizeof(payload)) {
        sendStatusText(MAV_SEVERITY_ERROR, "Config payload too large");
        return;
    }
    memcpy(&payload[payload_len], &config.maxCurrent, sizeof(int16_t));
    payload_len += sizeof(int16_t);

    if (payload_len + sizeof(float) > sizeof(payload)) {
        sendStatusText(MAV_SEVERITY_ERROR, "Config payload too large");
        return;
    }
    memcpy(&payload[payload_len], &config.positionKp, sizeof(float));
    payload_len += sizeof(float);

    if (payload_len + sizeof(float) > sizeof(payload)) {
        sendStatusText(MAV_SEVERITY_ERROR, "Config payload too large");
        return;
    }
    memcpy(&payload[payload_len], &config.velocityKp, sizeof(float));
    payload_len += sizeof(float);

    // Ensure payload length doesn't exceed MAVLink limits
    if (payload_len > MAVLINK_MAX_PAYLOAD_LEN) {
        sendStatusText(MAV_SEVERITY_ERROR, "Config payload exceeds MAVLink limit");
        return;
    }

    // Create message with custom ID
    msg.msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG;
    msg.len = payload_len;
    msg.sysid = system_id_;
    msg.compid = MAV_COMP_ID_AUTOPILOT1;
    memcpy(msg.payload64, payload, payload_len);

    sendMessage(&msg);
}

void MAVLinkRoboMasterController::handleHeartbeat(mavlink_message_t* msg) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);
    
    // Process heartbeat from ground station
    // Could track connection status, etc.
    (void)heartbeat; // Suppress unused parameter warning
}

void MAVLinkRoboMasterController::handleRCChannelsOverride(mavlink_message_t* msg) {
    mavlink_rc_channels_override_t rc_override;
    mavlink_msg_rc_channels_override_decode(msg, &rc_override);
    
    if (rc_override.target_system != system_id_) {
        return;
    }
    
    // Map RC channels to motors (scaled to ±50 RPS)
    float scale = 50.0f / 1000.0f;
    
    if (motor_registered_[0] && motors_[0] != nullptr && rc_override.chan1_raw != UINT16_MAX) {
        float velocity = (static_cast<float>(rc_override.chan1_raw) - 1500.0f) * scale;
        motors_[0]->setVelocityRPS(velocity);
    }
    
    if (motor_registered_[1] && motors_[1] != nullptr && rc_override.chan2_raw != UINT16_MAX) {
        float velocity = (static_cast<float>(rc_override.chan2_raw) - 1500.0f) * scale;
        motors_[1]->setVelocityRPS(velocity);
    }
    
    if (motor_registered_[2] && motors_[2] != nullptr && rc_override.chan3_raw != UINT16_MAX) {
        float velocity = (static_cast<float>(rc_override.chan3_raw) - 1500.0f) * scale;
        motors_[2]->setVelocityRPS(velocity);
    }
    
    if (motor_registered_[3] && motors_[3] != nullptr && rc_override.chan4_raw != UINT16_MAX) {
        float velocity = (static_cast<float>(rc_override.chan4_raw) - 1500.0f) * scale;
        motors_[3]->setVelocityRPS(velocity);
    }
}

void MAVLinkRoboMasterController::handleRequestDataStream(mavlink_message_t* msg) {
    mavlink_request_data_stream_t request;
    mavlink_msg_request_data_stream_decode(msg, &request);
    
    if (request.target_system != system_id_) {
        return;
    }
    
    // Update telemetry rate based on request
    if (request.req_stream_id == MAV_DATA_STREAM_ALL || 
        request.req_stream_id == MAV_DATA_STREAM_RC_CHANNELS) {
        
        telemetry_enabled_ = (request.start_stop == 1);
        if (request.req_message_rate > 0) {
            telemetry_rate_ms_ = 1000 / request.req_message_rate;
        }
    }
}

void MAVLinkRoboMasterController::handleMotorControl(mavlink_message_t* msg) {
    // Handle custom RoboMaster motor control message
    // This would need custom MAVLink message definitions

    // Validate message length first
    if (msg->len < 6) {
        sendStatusText(MAV_SEVERITY_ERROR, "Motor control msg too short");
        return;
    }

    const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg->payload64);

    uint8_t motor_id = payload[0];
    uint8_t control_mode = payload[1]; // 0=current, 1=velocity, 2=position

    // Validate motor ID
    if (motor_id < 1 || motor_id > MAX_MOTORS) {
        sendStatusText(MAV_SEVERITY_ERROR, "Invalid motor ID");
        return;
    }

    // Validate control mode
    if (control_mode > 2) {
        sendStatusText(MAV_SEVERITY_ERROR, "Invalid control mode");
        return;
    }

    // Extract control value (little endian) - ensure we have enough bytes
    if (msg->len < 6) {
        sendStatusText(MAV_SEVERITY_ERROR, "Insufficient data for control value");
        return;
    }

    float control_value;
    memcpy(&control_value, &payload[2], sizeof(float));

    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor == nullptr) {
        char error_msg[50];
        snprintf(error_msg, sizeof(error_msg), "Motor %d not found", motor_id);
        sendStatusText(MAV_SEVERITY_ERROR, error_msg);
        return;
    }

    // Validate command values before applying
    bool valid_command = false;
    switch (control_mode) {
        case 0: // Current control
            valid_command = validateMotorCommand(motor_id, control_value, "current");
            if (valid_command) {
                motor->setCurrent(static_cast<int16_t>(control_value));
            }
            break;
        case 1: // Velocity control
            valid_command = validateMotorCommand(motor_id, control_value, "velocity");
            if (valid_command) {
                motor->setVelocityRPS(control_value);
            }
            break;
        case 2: // Position control
            valid_command = validateMotorCommand(motor_id, control_value, "position");
            if (valid_command) {
                motor->setPositionRad(control_value);
            }
            break;
    }

    if (!valid_command) {
        char error_msg[80];
        snprintf(error_msg, sizeof(error_msg), "Motor %d command out of range: %.2f", motor_id, control_value);
        sendStatusText(MAV_SEVERITY_WARNING, error_msg);
    }
}

void MAVLinkRoboMasterController::handleMotorConfigSet(mavlink_message_t* msg) {
    // Handle custom RoboMaster motor configuration message
    // This would need custom MAVLink message definitions

    // Validate message length
    if (msg->len < 6) {
        sendStatusText(MAV_SEVERITY_ERROR, "Config msg too short");
        return;
    }

    const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg->payload64);

    uint8_t motor_id = payload[0];
    uint8_t param_id = payload[1];

    // Validate motor ID
    if (motor_id < 1 || motor_id > MAX_MOTORS) {
        sendStatusText(MAV_SEVERITY_ERROR, "Invalid motor ID for config");
        return;
    }

    // Extract parameter value (little endian) - ensure sufficient length
    if (msg->len < 6) {
        sendStatusText(MAV_SEVERITY_ERROR, "Insufficient config data");
        return;
    }

    float param_value;
    memcpy(&param_value, &payload[2], sizeof(float));

    RoboMasterMotor* motor = findMotor(motor_id);
    if (motor == nullptr) {
        char error_msg[50];
        snprintf(error_msg, sizeof(error_msg), "Motor %d not found for config", motor_id);
        sendStatusText(MAV_SEVERITY_ERROR, error_msg);
        return;
    }

    // Map param_id to actual parameters with bounds checking
    const char* param_names[] = {
        "positionKp", "positionKi", "positionKd",
        "velocityKp", "velocityKi", "velocityKd",
        "maxVelocityRPS", "maxCurrent", "maxTemperature"
    };

    const size_t num_params = sizeof(param_names) / sizeof(param_names[0]);

    if (param_id >= num_params) {
        char error_msg[60];
        snprintf(error_msg, sizeof(error_msg), "Invalid param ID %d (max %zu)", param_id, num_params - 1);
        sendStatusText(MAV_SEVERITY_ERROR, error_msg);
        return;
    }

    // Update parameter with validation
    RoboMasterStatus result = motor->updateParameter(param_names[param_id], param_value);
    if (result == RoboMasterStatus::OK) {
        char success_msg[80];
        snprintf(success_msg, sizeof(success_msg), "Motor %d %s set to %.2f", motor_id, param_names[param_id], param_value);
        sendStatusText(MAV_SEVERITY_INFO, success_msg);
    } else {
        char error_msg[80];
        snprintf(error_msg, sizeof(error_msg), "Failed to set Motor %d %s", motor_id, param_names[param_id]);
        sendStatusText(MAV_SEVERITY_ERROR, error_msg);
    }
}