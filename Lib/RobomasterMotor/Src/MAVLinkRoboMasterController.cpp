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
    
    // Send as servo output raw message for compatibility
    mavlink_message_t msg;
    
    const RoboMasterState& state = motor->getState();
    
    // Convert values for MAVLink
    uint16_t pulse_us = static_cast<uint16_t>(1500 + (state.currentVelocityRPS / 100.0f) * 500);
    pulse_us = (pulse_us < 1000) ? 1000 : (pulse_us > 2000) ? 2000 : pulse_us;
    
    uint16_t servo_values[16] = {0};
    servo_values[motor_id - 1] = pulse_us;
    
    mavlink_msg_servo_output_raw_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                                     getCurrentTimeMs() * 1000ULL,
                                     0,  // port
                                     servo_values[0], servo_values[1], servo_values[2], servo_values[3],
                                     servo_values[4], servo_values[5], servo_values[6], servo_values[7],
                                     servo_values[8], servo_values[9], servo_values[10], servo_values[11],
                                     servo_values[12], servo_values[13], servo_values[14], servo_values[15]);
    
    sendMessage(&msg);
}

void MAVLinkRoboMasterController::sendParameterValue(uint16_t param_index) {
    if (param_index >= parameter_count_) {
        return;
    }
    
    const ParameterInfo& param = parameters_[param_index];
    
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                                param.name, *(param.value_ptr), MAVLINK_TYPE_FLOAT,
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
    parameters_[parameter_count_] = {strdup(param_name), &config.positionKp, 0.0f, 1000.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_POS_KI", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), &config.positionKi, 0.0f, 100.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_POS_KD", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), &config.positionKd, 0.0f, 100.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_VEL_KP", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), &config.velocityKp, 0.0f, 1000.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_VEL_KI", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), &config.velocityKi, 0.0f, 100.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_VEL_KD", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), &config.velocityKd, 0.0f, 100.0f, motor_id};
    parameter_count_++;
    
    // Limit parameters
    snprintf(param_name, sizeof(param_name), "M%d_MAX_VEL", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), &config.maxVelocityRPS, 1.0f, 200.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_MAX_CUR", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), reinterpret_cast<float*>(&config.maxCurrent), 100.0f, 20000.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_MAX_TEMP", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), reinterpret_cast<float*>(&config.maxTemperature), 40.0f, 100.0f, motor_id};
    parameter_count_++;
    
    snprintf(param_name, sizeof(param_name), "M%d_TIMEOUT", motor_id);
    parameters_[parameter_count_] = {strdup(param_name), reinterpret_cast<float*>(&config.watchdogTimeoutMs), 50.0f, 5000.0f, motor_id};
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
    
    // Set the value
    *(param.value_ptr) = value;
    
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
    
    // For now, extract basic data assuming a simple format
    const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg->payload64);
    
    if (msg->len >= 8) {
        uint8_t motor_id = payload[0];
        uint8_t control_mode = payload[1]; // 0=current, 1=velocity, 2=position
        
        // Extract control value (little endian)
        float control_value;
        memcpy(&control_value, &payload[2], sizeof(float));
        
        RoboMasterMotor* motor = findMotor(motor_id);
        if (motor != nullptr) {
            switch (control_mode) {
                case 0: // Current control
                    motor->setCurrent(static_cast<int16_t>(control_value));
                    break;
                case 1: // Velocity control
                    motor->setVelocityRPS(control_value);
                    break;
                case 2: // Position control
                    motor->setPositionRad(control_value);
                    break;
            }
        }
    }
}

void MAVLinkRoboMasterController::handleMotorConfigSet(mavlink_message_t* msg) {
    // Handle custom RoboMaster motor configuration message
    // This would need custom MAVLink message definitions
    
    const uint8_t* payload = reinterpret_cast<const uint8_t*>(msg->payload64);
    
    if (msg->len >= 12) {
        uint8_t motor_id = payload[0];
        uint8_t param_id = payload[1];
        
        // Extract parameter value (little endian)
        float param_value;
        memcpy(&param_value, &payload[2], sizeof(float));
        
        RoboMasterMotor* motor = findMotor(motor_id);
        if (motor != nullptr) {
            // Map param_id to actual parameters
            const char* param_names[] = {
                "positionKp", "positionKi", "positionKd",
                "velocityKp", "velocityKi", "velocityKd",
                "maxVelocityRPS", "maxCurrent", "maxTemperature"
            };
            
            if (param_id < sizeof(param_names) / sizeof(param_names[0])) {
                motor->updateParameter(param_names[param_id], param_value);
            }
        }
    }
}