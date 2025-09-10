#include "MAVLinkServoController.hpp"

MAVLinkServoController::MAVLinkServoController() 
    : uart_(nullptr), system_id_(1), servo_count_(0), 
      last_heartbeat_(0), last_telemetry_(0) {
    for (int i = 0; i < MAX_SERVOS; i++) {
        servos_[i] = nullptr;
    }
}

void MAVLinkServoController::init(UART_HandleTypeDef* uart, uint8_t system_id) {
    uart_ = uart;
    system_id_ = system_id;
}

void MAVLinkServoController::addServo(ServoMotor* servo) {
    if (servo_count_ < MAX_SERVOS && servo != nullptr) {
        servos_[servo_count_++] = servo;
    }
}

void MAVLinkServoController::update() {
    uint32_t now = HAL_GetTick();
    
    // Send heartbeat every 1000ms (1Hz as required by MAVLink protocol)
    if (now - last_heartbeat_ >= 1000) {
        sendHeartbeat();
        last_heartbeat_ = now;
    }
    
    // Passive mode: Only send servo status when explicitly requested
    // (Remove automatic 20Hz transmission for ROS compatibility)
    
    // Update all servos (handles rate limiting and safety)
    for (uint8_t i = 0; i < servo_count_; i++) {
        if (servos_[i] != nullptr) {
            servos_[i]->update();
        }
    }
}

void MAVLinkServoController::processReceivedByte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg_, &rx_status_)) {
        handleMessage(&rx_msg_);
    }
}

void MAVLinkServoController::handleMessage(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            sendAutopilotVersion();
            break;
        
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Handle incoming heartbeats if needed
            break;
            
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            handleManualControl(msg);
            break;
            
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handleRcChannelsOverride(msg);
            break;
            
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
            handleRequestDataStream(msg);
            break;
            
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            // Send servo status when parameter read is requested
            sendServoOutputRaw();
            break;
            
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            // Send servo status when parameter list is requested
            sendServoOutputRaw();
            break;
            
        default:
            // Unknown message
            break;
    }
}

void MAVLinkServoController::sendHeartbeat() {
    mavlink_message_t msg;
    
    mavlink_msg_heartbeat_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                              MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 
                              MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
    
    sendMessage(&msg);
}

void MAVLinkServoController::sendServoOutputRaw() {
    mavlink_message_t msg;
    
    // Get servo pulse values (up to 16 servos), with validation
    uint16_t servo_values[16] = {0};
    for (uint8_t i = 0; i < servo_count_ && i < 16; i++) {
        if (servos_[i] != nullptr && servos_[i]->getStatus() == ServoStatus::OK) {
            uint16_t pulse = servos_[i]->getPulseUs();
            // Ensure pulse is within valid MAVLink range (0-65535, typically 1000-2000)
            servo_values[i] = (pulse >= 500 && pulse <= 2500) ? pulse : 0;
        } else {
            servo_values[i] = 0;  // Invalid/disconnected servo
        }
    }
    
    mavlink_msg_servo_output_raw_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                                     HAL_GetTick() * 1000ULL,  // time_usec (ensure 64-bit)
                                     0,  // port (MAIN)
                                     servo_values[0], servo_values[1], servo_values[2], servo_values[3],
                                     servo_values[4], servo_values[5], servo_values[6], servo_values[7],
                                     servo_values[8], servo_values[9], servo_values[10], servo_values[11],
                                     servo_values[12], servo_values[13], servo_values[14], servo_values[15]);
    
    sendMessage(&msg);
}

void MAVLinkServoController::sendAutopilotVersion() {
    mavlink_message_t msg;
    
    uint8_t flight_custom_version[8] = {0};
    uint8_t middleware_custom_version[8] = {0};
    uint8_t os_custom_version[8] = {0};
    uint8_t uid2[18] = {0};
    
    mavlink_msg_autopilot_version_pack(system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
                                      0,  // capabilities (minimal)
                                      0,  // flight_sw_version
                                      0,  // middleware_sw_version  
                                      0,  // os_sw_version
                                      0,  // board_version
                                      flight_custom_version,
                                      middleware_custom_version,
                                      os_custom_version,
                                      0,    // vendor_id
                                      0,    // product_id
                                      0,    // uid
                                      uid2);
    
    sendMessage(&msg);
}

void MAVLinkServoController::handleManualControl(mavlink_message_t* msg) {
    mavlink_manual_control_t manual_control;
    mavlink_msg_manual_control_decode(msg, &manual_control);
    
    // Check if message is for us
    if (manual_control.target != system_id_) {
        return;
    }
    
    // Convert MAVLink control inputs (-1000 to +1000) to servo angles (-90 to +90 degrees)
    // Map to first 4 servos: X, Y, Z, R axes
    if (servo_count_ > 0 && servos_[0] != nullptr) {
        float angle = (manual_control.x / 1000.0f) * 90.0f;
        servos_[0]->setAngleDeg(angle);
        servos_[0]->resetWatchdog();
    }
    
    if (servo_count_ > 1 && servos_[1] != nullptr) {
        float angle = (manual_control.y / 1000.0f) * 90.0f;
        servos_[1]->setAngleDeg(angle);
        servos_[1]->resetWatchdog();
    }
    
    if (servo_count_ > 2 && servos_[2] != nullptr) {
        float angle = (manual_control.z / 1000.0f) * 90.0f;
        servos_[2]->setAngleDeg(angle);
        servos_[2]->resetWatchdog();
    }
    
    if (servo_count_ > 3 && servos_[3] != nullptr) {
        float angle = (manual_control.r / 1000.0f) * 90.0f;
        servos_[3]->setAngleDeg(angle);
        servos_[3]->resetWatchdog();
    }
}

void MAVLinkServoController::handleRcChannelsOverride(mavlink_message_t* msg) {
    mavlink_rc_channels_override_t rc_override;
    mavlink_msg_rc_channels_override_decode(msg, &rc_override);
    
    // Check if message is for us
    if (rc_override.target_system != system_id_) {
        return;
    }
    
    // Convert PWM to angle for channels 1-8 (safer than direct PWM control)
    uint16_t channels[8] = {
        rc_override.chan1_raw, rc_override.chan2_raw, rc_override.chan3_raw, rc_override.chan4_raw,
        rc_override.chan5_raw, rc_override.chan6_raw, rc_override.chan7_raw, rc_override.chan8_raw
    };
    
    for (uint8_t i = 0; i < 8 && i < servo_count_; i++) {
        if (servos_[i] != nullptr && channels[i] != UINT16_MAX && 
            channels[i] >= 1000 && channels[i] <= 2000) {
            // Convert PWM (1000-2000us) to angle (-90 to +90 degrees)
            float angle = ((float)(channels[i] - 1500) / 500.0f) * 90.0f;
            servos_[i]->setAngleDeg(angle);
            servos_[i]->resetWatchdog();
        }
    }
}

void MAVLinkServoController::handleRequestDataStream(mavlink_message_t* msg) {
    mavlink_request_data_stream_t request;
    mavlink_msg_request_data_stream_decode(msg, &request);
    
    // Check if message is for us
    if (request.target_system != system_id_) {
        return;
    }
    
    // Handle request for servo output stream (MAV_DATA_STREAM_RAW_CONTROLLER = 3)
    if (request.req_stream_id == MAV_DATA_STREAM_RAW_CONTROLLER || 
        request.req_stream_id == MAV_DATA_STREAM_ALL) {
        if (request.start_stop == 1) {
            // Send immediate servo status response (passive mode - only on request)
            sendServoOutputRaw();
        }
        // Passive mode: We don't start continuous streaming, only respond to requests
    }
}

void MAVLinkServoController::sendMessage(mavlink_message_t* msg) {
    if (uart_ == nullptr) return;
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    
    // Use non-blocking transmission with reasonable timeout (100ms)
    HAL_StatusTypeDef status = HAL_UART_Transmit(uart_, buf, len, 100);
    
    // Optional: Toggle LED or set flag for debugging transmission status
    if (status != HAL_OK) {
        // Transmission failed - could log error or increment error counter
        // For now, we'll just continue (non-blocking behavior)
    }
}