#include "main.hpp"
#include "ServoMotor.hpp"
#include "MAVLinkServoController.hpp"
#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"
#include "MAVLinkRoboMasterController.hpp"
#include "Encoder.hpp"
#include "EncoderConfig.hpp"
#include "DCMotor.hpp"
#include "MAVLinkDCMotorController.hpp"
#include <cmath>

// HAL objects
//uart
extern UART_HandleTypeDef huart2;
//can
extern CAN_HandleTypeDef hcan1;
//timers
extern TIM_HandleTypeDef htim1;// encoder
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;// pwm
extern TIM_HandleTypeDef htim4;// encoder
extern TIM_HandleTypeDef htim12;

// MAVLink communication
MAVLinkServoController mavlink_controller;
MAVLinkRoboMasterController mavlink_robomaster_controller;
MAVLinkDCMotorController mavlink_dcmotor_controller;
uint8_t rx_buffer[1];

// Interrupt-safe circular buffer for UART data
#define UART_BUFFER_SIZE 256
static volatile uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

// Encoders
Encoder encoder_dcmotor;

// Limit switch state tracking
uint8_t limitSwitchState = 0;

// Servo instances
ServoMotor servo1;
ServoMotor servo2;
ServoMotor servo3;
ServoMotor servo4;

// DC Motor instances - PWM resolution automatically determined from timer period
DCMotor dcmotor1(&htim3, TIM_CHANNEL_1, GPIOB, GPIO_PIN_8, true);

// RoboMaster CAN manager and GM6020 motor instances
// RoboMasterCANManager can_manager;
// RoboMasterMotor gm6020_1;
// // RoboMasterMotor gm6020_2;
// RoboMasterConfig motor_config;


void setup() {
    // Initialize encoders
    encoder_dcmotor.create(1, &htim1);

    // Load encoder config for cumulative position tracking (embedded JSON - file I/O not available on embedded STM32)
    const char* encoder_config_json = R"({
  "encoders": {
    "1": {
      "cpr": 8192,
      "invertA": true,
      "invertB": false,
      "useZ": false,
      "mode": "TIM_ENCODER_MODE",
      "watchdogTimeoutMs": 500,
      "offsetCounts": 0,
      "wrapAround": false
    }
  }
})";

    // Parse config for encoder ID 1
    EncoderConfig encoder_config;
    if (EncoderConfigParser::parseFromJsonForId(encoder_config_json, 1, encoder_config) == EncoderStatus::OK) {
        encoder_dcmotor.init(encoder_config);
    } else {
        encoder_dcmotor.init();  // Fallback to default config
    }

    // Initialize servos with configuration from JSON
    servo1.create(1, &htim2, TIM_CHANNEL_1);
    servo1.loadConfigFromFileForId("Lib/ServoMotor/Inc/servo_config.json");
    servo1.init();
    servo1.setEnabled(true);

    servo2.create(2, &htim2, TIM_CHANNEL_2);
    servo2.loadConfigFromFileForId("Lib/ServoMotor/Inc/servo_config.json");
    servo2.init();
    servo2.setEnabled(true);

    servo3.create(3, &htim12, TIM_CHANNEL_1);
    servo3.loadConfigFromFileForId("Lib/ServoMotor/Inc/servo_config.json");
    servo3.init();
    servo3.setEnabled(true);

    servo4.create(4, &htim12, TIM_CHANNEL_2);
    servo4.loadConfigFromFileForId("Lib/ServoMotor/Inc/servo_config.json");
    servo4.init();
    servo4.setEnabled(true);


    // Initialize DC Motor
    dcmotor1.setMotorId(10);  // Motor ID 10 for MAVLink
    dcmotor1.start();

    // Initialize DC Motor MAVLink controller
    mavlink_dcmotor_controller.init(10, &dcmotor1, &encoder_dcmotor, &huart2, 1);

    // Configure DC Motor with reasonable PID parameters
    MotorConfig dc_motor_config;
    dc_motor_config.motor_id = 10;
    dc_motor_config.mode = MotorControlMode::DUTY_TO_POSITION;

    // Speed control PID (inner loop) - Tuned for smoother operation
    dc_motor_config.speed_kp = 0.1f;    // Increased for better response
    dc_motor_config.speed_ki = 0.1f;    // Small integral to eliminate steady-state error
    dc_motor_config.speed_kd = 0.0f;   // Small derivative for damping
    dc_motor_config.speed_max_integral = 0.3f;  // Limit integral windup
    dc_motor_config.speed_max_output = 0.5f;

    // Position control PID (outer loop)
    dc_motor_config.position_kp = 0.1f;
    dc_motor_config.position_ki = 0.0f;
    dc_motor_config.position_kd = 0.0f;
    dc_motor_config.position_max_integral = 100.0f;
    dc_motor_config.position_max_output = 10.0f;  // max speed rad/s

    // Physical limits
    dc_motor_config.max_speed_rad_s = 15.0f;
    dc_motor_config.max_acceleration_rad_s2 = 50.0f;
    dc_motor_config.use_position_limits = true;
    dc_motor_config.position_limit_min_rad = -M_PI * 100.0f;  // -10 revolutions
    dc_motor_config.position_limit_max_rad = M_PI * 100.0f;   // +10 revolutions

    // Safety settings
    dc_motor_config.watchdog_timeout_ms = 2000;  // 2 seconds timeout
    dc_motor_config.control_period_ms = 10;      // 100Hz control

    mavlink_dcmotor_controller.setConfig(dc_motor_config);
    mavlink_dcmotor_controller.setMode(MotorControlMode::DISABLED);
    mavlink_dcmotor_controller.enable();

    // Initialize CAN manager and GM6020 motors
    // can_manager.init(&hcan1);
    // can_manager.start();
    
    // gm6020_1.create(5, &can_manager);
    // gm6020_1.init(motor_config);
    // gm6020_1.setControlMode(RoboMasterControlMode::POSITION);
    // gm6020_1.setEnabled(true);
    // gm6020_1.setInitialPosition(30.0*M_PI/180.0);
    // gm6020_1.setPositionRad(0.0f);
    
    // gm6020_2.create(6, &can_manager);
    // gm6020_2.init();
    // gm6020_2.setEnabled(true);
    
    // Initialize MAVLink controllers
    mavlink_controller.init(&huart2, 1);  // UART2, System ID 1
    mavlink_controller.addServo(&servo1);
    mavlink_controller.addServo(&servo2);
    mavlink_controller.addServo(&servo3);
    mavlink_controller.addServo(&servo4);
    
    // Initialize MAVLink RoboMaster controller
    // mavlink_robomaster_controller.init(&huart2, 1);  // UART2, System ID 1
    // mavlink_robomaster_controller.setCANManager(&can_manager);
    // mavlink_robomaster_controller.addMotor(&gm6020_1, 5);
    // mavlink_robomaster_controller.addMotor(&gm6020_2, 6);
    
    // Enable UART receive interrupt
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

void loop() {
    // Process any received UART data from interrupt buffer (limit iterations to prevent infinite loop)
    uint8_t max_iterations = 32;  // Process max 32 bytes per loop
    while (uart_rx_tail != uart_rx_head && max_iterations-- > 0) {
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
            // mavlink_robomaster_controller.processReceivedByte(byte);
            mavlink_dcmotor_controller.processReceivedByte(byte);
        } else {
            break; // Buffer is empty
        }
    }
    
    // Update MAVLink controllers
    mavlink_controller.update();
    // mavlink_robomaster_controller.update();
    mavlink_dcmotor_controller.update();
    
    // Update CAN manager (this also updates all registered motors)
    // can_manager.update();
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
        if (limitSwitchState == 0) {
            limitSwitchState = 1;
            // Reset encoder position to zero (origin)
            encoder_dcmotor.setZeroPosition();
            // Reset MAVLink controller target position to zero
            mavlink_dcmotor_controller.setTargetPosition(0.0f);
            // Stop the motor briefly for safety
            dcmotor1.brake();
            HAL_Delay(10);  // Brief delay to ensure motor stops
            dcmotor1.start();
            // Toggle LED to indicate limit switch activation
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    } else {
        limitSwitchState = 0;
    }

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

// extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//     if (hcan->Instance == CAN1) {
//         can_manager.handleCANReceive();
//     }
// }