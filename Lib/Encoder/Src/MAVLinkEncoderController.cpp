#include "MAVLinkEncoderController.hpp"

MAVLinkEncoderController::MAVLinkEncoderController() 
    : uart_(nullptr), system_id_(1), encoder_count_(0), 
      last_status_send_(0) {
    for (int i = 0; i < MAX_ENCODERS; i++) {
        encoders_[i] = nullptr;
    }
}

void MAVLinkEncoderController::init(UART_HandleTypeDef* uart, uint8_t system_id) {
    uart_ = uart;
    system_id_ = system_id;
}

void MAVLinkEncoderController::addEncoder(Encoder* encoder) {
    if (encoder_count_ < MAX_ENCODERS && encoder != nullptr) {
        encoders_[encoder_count_++] = encoder;
    }
}

void MAVLinkEncoderController::update() {
    uint32_t now = HAL_GetTick();
    
    // Send encoder status every 100ms (10Hz)
    if (now - last_status_send_ >= STATUS_SEND_RATE_MS) {
        sendMotorStatus();
        last_status_send_ = now;
    }
    
    // Update all encoders (handles watchdog and position updates)
    for (uint8_t i = 0; i < encoder_count_; i++) {
        if (encoders_[i] != nullptr) {
            encoders_[i]->update();
        }
    }
}

void MAVLinkEncoderController::processReceivedByte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg_, &rx_status_)) {
        handleMessage(&rx_msg_);
    }
}

void MAVLinkEncoderController::handleMessage(mavlink_message_t* msg) {
    switch (msg->msgid) {
        // Using existing MAVLink motor messages for encoder configuration
        // These would be custom message IDs in a real implementation
        case MAVLINK_MSG_ID_COMMAND_LONG:
            // Handle MOTOR_CONFIG_SET, MOTOR_CONFIG_GET, MOTOR_CONFIG_SAVE via command_long
            {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(msg, &cmd);
                
                if (cmd.target_system == system_id_) {
                    switch ((uint16_t)cmd.command) {
                        case 31000: // MOTOR_CONFIG_SET
                            // param1 = motor_id, param2 = config_type, param3 = value
                            {
                                uint8_t motor_id = (uint8_t)cmd.param1;
                                uint8_t config_type = (uint8_t)cmd.param2;
                                float value = cmd.param3;
                                
                                Encoder* encoder = findEncoderById(motor_id);
                                if (encoder != nullptr) {
                                    EncoderConfig config = encoder->getConfig();
                                    bool config_changed = false;
                                    
                                    switch (config_type) {
                                        case 1: // cpr
                                            if (value > 0 && value <= 65535) {
                                                config.cpr = (uint16_t)value;
                                                config_changed = true;
                                            }
                                            break;
                                        case 2: // invertA
                                            config.invertA = (value != 0.0f);
                                            config_changed = true;
                                            break;
                                        case 3: // invertB
                                            config.invertB = (value != 0.0f);
                                            config_changed = true;
                                            break;
                                        case 4: // useZ
                                            config.useZ = (value != 0.0f);
                                            config_changed = true;
                                            break;
                                    }
                                    
                                    if (config_changed) {
                                        encoder->setConfig(config);
                                    }
                                }
                            }
                            break;
                            
                        case 31001: // MOTOR_CONFIG_GET
                            {
                                uint8_t motor_id = (uint8_t)cmd.param1;
                                Encoder* encoder = findEncoderById(motor_id);
                                if (encoder != nullptr) {
                                    sendMotorConfigResponse(motor_id, encoder->getConfig());
                                }
                            }
                            break;
                            
                        case 31002: // MOTOR_CONFIG_SAVE
                            {
                                uint8_t motor_id = (uint8_t)cmd.param1;
                                Encoder* encoder = findEncoderById(motor_id);
                                if (encoder != nullptr) {
                                    // In a real implementation, this would save to file system
                                    // For now, we just acknowledge the command
                                    mavlink_message_t ack_msg;
                                    mavlink_msg_command_ack_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &ack_msg,
                                                               31002, MAV_RESULT_ACCEPTED, 0, 0,
                                                               msg->sysid, msg->compid);
                                    sendMessage(&ack_msg);
                                }
                            }
                            break;
                    }
                }
            }
            break;
            
        default:
            // Unknown message
            break;
    }
}

void MAVLinkEncoderController::sendMotorStatus() {
    // Send encoder position data using ATTITUDE message (reusing existing message)
    // In a real implementation, custom messages would be defined
    for (uint8_t i = 0; i < encoder_count_; i++) {
        if (encoders_[i] != nullptr) {
            mavlink_message_t msg;
            const EncoderState& state = encoders_[i]->getState();
            
            // Pack encoder data into attitude message
            // roll = encoder angle in radians
            // pitch = encoder position (normalized)
            // yaw = revolutions
            mavlink_msg_attitude_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                                    HAL_GetTick() * 1000, // time_usec
                                    state.currentAngleRad, // roll (encoder angle)
                                    (float)state.currentPosition / encoders_[i]->getConfig().cpr, // pitch (normalized position)
                                    (float)state.revolutions, // yaw (revolutions)
                                    0.0f, 0.0f, 0.0f); // rollspeed, pitchspeed, yawspeed
            
            sendMessage(&msg);
        }
    }
}

void MAVLinkEncoderController::sendMotorConfigResponse(uint8_t motor_id, const EncoderConfig& config) {
    // Send config data using PARAM_VALUE message
    mavlink_message_t msg;
    char param_id[16];
    
    // Send CPR
    snprintf(param_id, sizeof(param_id), "ENC%d_CPR", motor_id);
    mavlink_msg_param_value_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                               param_id, (float)config.cpr, MAV_PARAM_TYPE_UINT16, 0, 0);
    sendMessage(&msg);
    
    // Send invertA
    snprintf(param_id, sizeof(param_id), "ENC%d_INVA", motor_id);
    mavlink_msg_param_value_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                               param_id, config.invertA ? 1.0f : 0.0f, MAV_PARAM_TYPE_UINT8, 0, 0);
    sendMessage(&msg);
    
    // Send invertB
    snprintf(param_id, sizeof(param_id), "ENC%d_INVB", motor_id);
    mavlink_msg_param_value_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                               param_id, config.invertB ? 1.0f : 0.0f, MAV_PARAM_TYPE_UINT8, 0, 0);
    sendMessage(&msg);
    
    // Send useZ
    snprintf(param_id, sizeof(param_id), "ENC%d_USEZ", motor_id);
    mavlink_msg_param_value_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                               param_id, config.useZ ? 1.0f : 0.0f, MAV_PARAM_TYPE_UINT8, 0, 0);
    sendMessage(&msg);
}

void MAVLinkEncoderController::sendMessage(mavlink_message_t* msg) {
    if (uart_ == nullptr) return;
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    HAL_UART_Transmit(uart_, buf, len, HAL_MAX_DELAY);
}

Encoder* MAVLinkEncoderController::findEncoderById(uint8_t id) {
    for (uint8_t i = 0; i < encoder_count_; i++) {
        if (encoders_[i] != nullptr && encoders_[i]->getId() == id) {
            return encoders_[i];
        }
    }
    return nullptr;
}