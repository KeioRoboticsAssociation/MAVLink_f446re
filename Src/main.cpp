#include "main.hpp"
#include "../Lib/mavlink/c_library_v2/common/mavlink.h"

// ---- HALのオブジェクト ----
extern UART_HandleTypeDef huart2;

// MAVLink settings
#define MAV_SYSTEM_ID 1
#define MAV_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1
#define MAV_TARGET_SYSTEM 1
#define MAV_TARGET_COMPONENT 191

// UART receive buffer for incoming messages
uint8_t rx_buffer[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t rx_status;
mavlink_message_t rx_msg;

void mavlink_send_heartbeat() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack heartbeat message
    mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,
                              MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 
                              MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
    
    // Serialize message to buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send via UART
    HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

void mavlink_send_autopilot_version() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    uint8_t flight_custom_version[8] = {0};
    uint8_t middleware_custom_version[8] = {0};
    uint8_t os_custom_version[8] = {0};
    uint8_t uid2[18] = {0};
    
    // Pack autopilot version message
    mavlink_msg_autopilot_version_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg,
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
    
    // Serialize message to buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send via UART
    HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

void mavlink_handle_message(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            // Respond to version requests
            mavlink_send_autopilot_version();
            break;
        
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Handle incoming heartbeats if needed
            break;
            
        default:
            // Unknown message
            break;
    }
}

void mavlink_receive_byte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
        // Complete message received
        mavlink_handle_message(&rx_msg);
    }
}

void setup() {
    // Enable UART receive interrupt for MAVLink message parsing
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

void loop() {
    // Send heartbeat every 1000ms (1Hz as required by MAVLink protocol)
    static uint32_t last_heartbeat = 0;
    uint32_t now = HAL_GetTick();
    
    if (now - last_heartbeat >= 1000) {
        mavlink_send_heartbeat();
        last_heartbeat = now;
    }
    
    HAL_Delay(10); // Small delay to prevent overwhelming the system
}

// UART receive complete interrupt callback
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Process received byte through MAVLink parser
        mavlink_receive_byte(rx_buffer[0]);
        
        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
    }
}