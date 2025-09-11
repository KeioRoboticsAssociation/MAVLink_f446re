#include "main.hpp"
#include "ServoMotor.hpp"
#include "MAVLinkServoController.hpp"
#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"
#include "MAVLinkRoboMasterController.hpp"

// HAL objects
//uart
extern UART_HandleTypeDef huart2;
//can
extern CAN_HandleTypeDef hcan1;
//timers
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim12;

// MAVLink communication
MAVLinkServoController mavlink_controller;
MAVLinkRoboMasterController mavlink_robomaster_controller;
uint8_t rx_buffer[1];

// Interrupt-safe circular buffer for UART data
#define UART_BUFFER_SIZE 256
static volatile uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

// Servo instances
ServoMotor servo1;
ServoMotor servo2;
ServoMotor servo3;

// RoboMaster CAN manager and GM6020 motor instances
RoboMasterCANManager can_manager;
RoboMasterMotor gm6020_1;
// RoboMasterMotor gm6020_2;


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
    
    // Initialize CAN manager and GM6020 motors
    can_manager.init(&hcan1);
    can_manager.start();
    
    gm6020_1.create(5, &can_manager);
    gm6020_1.init();
    gm6020_1.setEnabled(true);
    
    // gm6020_2.create(6, &can_manager);
    // gm6020_2.init();
    // gm6020_2.setEnabled(true);
    
    // Initialize MAVLink controllers
    mavlink_controller.init(&huart2, 1);  // UART2, System ID 1
    mavlink_controller.addServo(&servo1);
    mavlink_controller.addServo(&servo2);
    mavlink_controller.addServo(&servo3);
    
    // Initialize MAVLink RoboMaster controller
    mavlink_robomaster_controller.init(&huart2, 1);  // UART2, System ID 1
    mavlink_robomaster_controller.setCANManager(&can_manager);
    mavlink_robomaster_controller.addMotor(&gm6020_1, 5);
    // mavlink_robomaster_controller.addMotor(&gm6020_2, 6);
    
    // Enable UART receive interrupt
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

void loop() {
    // Process any received UART data from interrupt buffer
    while (uart_rx_tail != uart_rx_head) {
        // Disable interrupts temporarily to ensure atomic access
        __disable_irq();
        uint16_t local_head = uart_rx_head;
        uint16_t local_tail = uart_rx_tail;
        __enable_irq();
        
        if (local_tail != local_head) {
            uint8_t byte = uart_rx_buffer[local_tail];
            local_tail = (local_tail + 1) % UART_BUFFER_SIZE;
            
            // Update tail atomically
            __disable_irq();
            uart_rx_tail = local_tail;
            __enable_irq();
            
            mavlink_controller.processReceivedByte(byte);
            mavlink_robomaster_controller.processReceivedByte(byte);
        } else {
            break; // Buffer is empty
        }
    }
    
    // Update MAVLink controllers
    mavlink_controller.update();
    mavlink_robomaster_controller.update();
    
    // Update CAN manager
    can_manager.update();
    
    // Short delay to prevent tight loop
    HAL_Delay(10);
}

// UART receive complete interrupt callback
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Store byte in circular buffer (interrupt safe)
        uint16_t next_head = (uart_rx_head + 1) % UART_BUFFER_SIZE;
        if (next_head != uart_rx_tail) {  // Buffer not full
            uart_rx_buffer[uart_rx_head] = rx_buffer[0];
            uart_rx_head = next_head;
        }
        
        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
        
        // Toggle LED to indicate activity
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        can_manager.handleCANReceive();
    }
}