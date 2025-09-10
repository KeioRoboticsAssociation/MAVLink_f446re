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

// Interrupt-safe circular buffer for UART data
#define UART_BUFFER_SIZE 256
static volatile uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

// Servo instances
ServoMotor servo1;
ServoMotor servo2;
ServoMotor servo3;

int setup_count = 0;

void setup() {
    setup_count++;
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
    // Process any received UART data from interrupt buffer
    while (uart_rx_tail != uart_rx_head) {
        uint8_t byte = uart_rx_buffer[uart_rx_tail];
        uart_rx_tail = (uart_rx_tail + 1) % UART_BUFFER_SIZE;
        mavlink_controller.processReceivedByte(byte);
    }
    
    // Update MAVLink controller (handles heartbeat, telemetry, and servo updates)
    mavlink_controller.update();
    
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