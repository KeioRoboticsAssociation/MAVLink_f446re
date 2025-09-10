#include "main.hpp"
#include "ServoMotor.hpp"
#include "MAVLinkServoController.hpp"

// HAL objects
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim12;

// MAVLink communication
MAVLinkServoController mavlink_controller;
uint8_t rx_buffer[1];

// Servo instances
ServoMotor servo1;
ServoMotor servo2;
ServoMotor servo3;

void setup() {
    // Initialize servos
    servo1.create(1, &htim2, TIM_CHANNEL_1);
    servo1.init();
    servo1.setEnabled(true);
    servo1.setAngleDeg(0);
    
    servo2.create(2, &htim2, TIM_CHANNEL_2);
    servo2.init();
    servo2.setEnabled(true);
    servo2.setAngleDeg(0);
    
    servo3.create(3, &htim12, TIM_CHANNEL_1);
    servo3.init();
    servo3.setEnabled(true);
    servo3.setAngleDeg(0);
    
    // Initialize MAVLink controller
    mavlink_controller.init(&huart2, 1);  // UART2, System ID 1
    mavlink_controller.addServo(&servo1);
    mavlink_controller.addServo(&servo2);
    mavlink_controller.addServo(&servo3);
    
    // Enable UART receive interrupt
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

void loop() {
    // Update MAVLink controller (handles heartbeat, telemetry, and servo updates)
    mavlink_controller.update();
    
    HAL_Delay(10);
}

// UART receive complete interrupt callback
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        mavlink_controller.processReceivedByte(rx_buffer[0]);
        HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
    }
}