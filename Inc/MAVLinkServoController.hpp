#pragma once

#include "stm32f4xx_hal.h"
#include "ServoMotor.hpp"
#include "../Lib/mavlink/c_library_v2/common/mavlink.h"

class MAVLinkServoController {
public:
    MAVLinkServoController();
    ~MAVLinkServoController() = default;

    void init(UART_HandleTypeDef* uart, uint8_t system_id = 1);
    void addServo(ServoMotor* servo);
    void update();
    void handleMessage(mavlink_message_t* msg);
    void processReceivedByte(uint8_t byte);

private:
    static constexpr uint8_t MAX_SERVOS = 16;
    
    UART_HandleTypeDef* uart_;
    uint8_t system_id_;
    
    ServoMotor* servos_[MAX_SERVOS];
    uint8_t servo_count_;
    
    // MAVLink parsing
    mavlink_status_t rx_status_;
    mavlink_message_t rx_msg_;
    
    // Timing
    uint32_t last_heartbeat_;
    uint32_t last_telemetry_;
    
    void sendHeartbeat();
    void sendServoOutputRaw();
    void sendAutopilotVersion();
    void handleManualControl(mavlink_message_t* msg);
    void handleRcChannelsOverride(mavlink_message_t* msg);
    void sendMessage(mavlink_message_t* msg);
};