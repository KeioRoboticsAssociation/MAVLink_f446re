#pragma once

#include "stm32f4xx_hal.h"
#include "Encoder.hpp"
#include "../mavlink/c_library_v2/common/mavlink.h"

class MAVLinkEncoderController {
public:
    MAVLinkEncoderController();
    ~MAVLinkEncoderController() = default;

    void init(UART_HandleTypeDef* uart, uint8_t system_id = 1);
    void addEncoder(Encoder* encoder);
    void update();
    void handleMessage(mavlink_message_t* msg);
    void processReceivedByte(uint8_t byte);

private:
    static constexpr uint8_t MAX_ENCODERS = 16;
    static constexpr uint32_t STATUS_SEND_RATE_MS = 100;  // 10Hz status updates
    
    UART_HandleTypeDef* uart_;
    uint8_t system_id_;
    
    Encoder* encoders_[MAX_ENCODERS];
    uint8_t encoder_count_;
    
    // MAVLink parsing
    mavlink_status_t rx_status_;
    mavlink_message_t rx_msg_;
    
    // Timing
    uint32_t last_status_send_;
    
    // Message handlers for motor commands (reusing existing MAVLink motor messages)
    void handleMotorConfigSet(mavlink_message_t* msg);
    void handleMotorConfigGet(mavlink_message_t* msg);
    void handleMotorConfigSave(mavlink_message_t* msg);
    void handleMotorStatusRequest(mavlink_message_t* msg);
    
    // Status sending
    void sendMotorStatus();
    void sendMotorConfigResponse(uint8_t motor_id, const EncoderConfig& config);
    
    // Utility
    void sendMessage(mavlink_message_t* msg);
    Encoder* findEncoderById(uint8_t id);
};