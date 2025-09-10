/**
 * @file ros_integration_example.cpp
 * @brief Complete ROS integration example for RoboMaster motors with MAVLink parameter management
 * 
 * This example shows how to:
 * - Set up RoboMaster motors with MAVLink communication
 * - Read/write parameters from ROS via MAVLink
 * - Control motors and monitor status
 * - Save/load configurations
 */

#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"
#include "MAVLinkRoboMasterController.hpp"
#include "stm32f4xx_hal.h"

// External HAL handles (configured in main.c)
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

// Global objects
RoboMasterCANManager can_manager;
MAVLinkRoboMasterController mavlink_controller;

// Motor instances (up to 8 motors supported)
RoboMasterMotor motor1;
RoboMasterMotor motor2;
RoboMasterMotor motor3;
RoboMasterMotor motor4;

/**
 * @brief Initialize the complete RoboMaster system with MAVLink
 */
void robomaster_system_init() {
    // 1. Initialize CAN manager
    if (can_manager.init(&hcan1) != CANManagerStatus::OK) {
        printf("Failed to initialize CAN manager\n");
        return;
    }
    
    // 2. Initialize MAVLink controller
    mavlink_controller.init(&huart2, 1);  // System ID = 1
    mavlink_controller.setCANManager(&can_manager);
    
    // 3. Create and configure motors
    setupMotors();
    
    // 4. Register motors with MAVLink controller
    mavlink_controller.addMotor(&motor1, 1);
    mavlink_controller.addMotor(&motor2, 2);
    mavlink_controller.addMotor(&motor3, 3);
    mavlink_controller.addMotor(&motor4, 4);
    
    // 5. Start CAN communication
    if (can_manager.start() != CANManagerStatus::OK) {
        printf("Failed to start CAN manager\n");
        return;
    }
    
    // 6. Enable motors
    motor1.setEnabled(true);
    motor2.setEnabled(true);
    motor3.setEnabled(true);
    motor4.setEnabled(true);
    
    printf("RoboMaster system initialized successfully\n");
}

/**
 * @brief Configure motors with different control strategies
 */
void setupMotors() {
    // Motor 1: High-precision position control
    RoboMasterConfig config1;
    config1.maxVelocityRPS = 30.0f;
    config1.maxCurrent = 8000;
    config1.positionKp = 25.0f;
    config1.positionKi = 0.5f;
    config1.positionKd = 0.2f;
    config1.velocityKp = 50.0f;
    config1.velocityKi = 0.3f;
    config1.positionLimitsEnabled = true;
    config1.minPositionRad = -5.0f * M_PI;
    config1.maxPositionRad = 5.0f * M_PI;
    config1.startupMode = RoboMasterControlMode::POSITION;
    
    motor1.create(1, &can_manager);
    motor1.init(config1);
    
    // Motor 2: High-speed velocity control
    RoboMasterConfig config2;
    config2.maxVelocityRPS = 80.0f;
    config2.maxCurrent = 12000;
    config2.velocityKp = 40.0f;
    config2.velocityKi = 0.4f;
    config2.maxAccelerationRPS2 = 300.0f;
    config2.startupMode = RoboMasterControlMode::VELOCITY;
    
    motor2.create(2, &can_manager);
    motor2.init(config2);
    
    // Motor 3: Torque control for force feedback
    RoboMasterConfig config3;
    config3.maxCurrent = 15000;
    config3.startupMode = RoboMasterControlMode::CURRENT;
    config3.watchdogTimeoutMs = 200;  // Faster timeout for safety
    
    motor3.create(3, &can_manager);
    motor3.init(config3);
    
    // Motor 4: Default configuration
    motor4.create(4, &can_manager);
    motor4.init();
}

/**
 * @brief Main control loop - call this periodically (100Hz recommended)
 */
void robomaster_control_loop() {
    // Update CAN manager (handles all motor communication)
    can_manager.update();
    
    // Update MAVLink controller (handles ROS communication)
    mavlink_controller.update();
    
    // Example control logic based on some external input
    static uint32_t loop_counter = 0;
    loop_counter++;
    
    // Motor control examples
    if (loop_counter % 100 == 0) {  // Every 1 second at 100Hz
        demonstrateMotorControl();
    }
    
    // Safety monitoring
    monitorMotorSafety();
}

/**
 * @brief Demonstrate different motor control modes
 */
void demonstrateMotorControl() {
    static int demo_phase = 0;
    demo_phase = (demo_phase + 1) % 4;
    
    switch (demo_phase) {
        case 0:
            // Position control demo
            motor1.setPositionRad(2.0f * M_PI);  // 2 full rotations
            printf("Motor 1: Position control - 2 rotations\n");
            break;
            
        case 1:
            // Velocity control demo
            motor2.setVelocityRPS(20.0f);
            printf("Motor 2: Velocity control - 20 RPS\n");
            break;
            
        case 2:
            // Current control demo
            motor3.setCurrent(3000);  // 3A
            printf("Motor 3: Current control - 3A\n");
            break;
            
        case 3:
            // Return to neutral
            motor1.setPositionRad(0.0f);
            motor2.setVelocityRPS(0.0f);
            motor3.setCurrent(0);
            printf("All motors: Return to neutral\n");
            break;
    }
}

/**
 * @brief Monitor motor safety and status
 */
void monitorMotorSafety() {
    RoboMasterMotor* motors[] = {&motor1, &motor2, &motor3, &motor4};
    
    for (int i = 0; i < 4; i++) {
        const RoboMasterState& state = motors[i]->getState();
        
        // Check for error conditions
        if (state.status != RoboMasterStatus::OK) {
            printf("Motor %d Error: Status %d\n", i+1, static_cast<int>(state.status));
            
            switch (state.status) {
                case RoboMasterStatus::TIMEOUT:
                    motors[i]->resetWatchdog();
                    break;
                    
                case RoboMasterStatus::OVERHEAT:
                    motors[i]->setEnabled(false);
                    printf("Motor %d disabled due to overheating (%d°C)\n", 
                           i+1, state.temperatureCelsius);
                    break;
                    
                case RoboMasterStatus::OVERCURRENT:
                    motors[i]->setCurrent(0);
                    printf("Motor %d overcurrent detected (%d mA)\n", 
                           i+1, state.currentMilliamps);
                    break;
                    
                default:
                    break;
            }
        }
    }
}

/**
 * @brief UART interrupt handler for MAVLink communication
 * Call this from HAL_UART_RxCpltCallback() or equivalent
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t rx_byte;
    
    if (huart->Instance == USART2) {
        // Process received MAVLink byte
        mavlink_controller.processReceivedByte(rx_byte);
        
        // Re-enable interrupt for next byte
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/**
 * @brief CAN interrupt handler
 * Call this from HAL_CAN_RxFifo0MsgPendingCallback()
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        can_manager.handleCANReceive();
    }
}

/**
 * @brief Example ROS parameter operations via MAVLink
 * These would be initiated from ROS side, but shown here for reference
 */
void exampleParameterOperations() {
    // These operations would normally come from ROS via MAVLink messages
    
    // Reading parameters (ROS → STM32)
    printf("Current PID gains for Motor 1:\n");
    printf("Position Kp: %.3f\n", motor1.getParameter("positionKp"));
    printf("Position Ki: %.3f\n", motor1.getParameter("positionKi"));
    printf("Velocity Kp: %.3f\n", motor1.getParameter("velocityKp"));
    
    // Writing parameters (ROS → STM32)
    if (motor1.updateParameter("positionKp", 30.0f) == RoboMasterStatus::OK) {
        printf("Updated Motor 1 position Kp to 30.0\n");
    }
    
    if (motor2.updateParameter("maxVelocityRPS", 60.0f) == RoboMasterStatus::OK) {
        printf("Updated Motor 2 max velocity to 60 RPS\n");
    }
}

/**
 * @brief ROS MAVLink parameter commands that can be sent to this system
 * 
 * From ROS, you can use these MAVLink commands:
 * 
 * 1. Read all parameters:
 *    rostopic pub /mavlink/to mavros_msgs/Mavlink "header: {...}
 *    msgid: 21  # PARAM_REQUEST_LIST
 *    payload64: [1, 0, 0, 0]"  # target_system=1
 * 
 * 2. Read specific parameter:
 *    rostopic pub /mavlink/to mavros_msgs/Mavlink "header: {...}
 *    msgid: 20  # PARAM_REQUEST_READ
 *    payload64: [1, 0, 'M1_VEL_KP', 0]"
 * 
 * 3. Set parameter:
 *    rostopic pub /mavlink/to mavros_msgs/Mavlink "header: {...}
 *    msgid: 23  # PARAM_SET
 *    payload64: [1, 0, 'M1_VEL_KP', 45.0]"
 * 
 * 4. Save parameters to flash:
 *    rostopic pub /mavlink/to mavros_msgs/Mavlink "header: {...}
 *    msgid: 76  # COMMAND_LONG
 *    payload64: [245, 1, 0, 0, 0, 0, 0, 0, 1, 0]"  # MAV_CMD_PREFLIGHT_STORAGE
 * 
 * 5. Motor control via manual control:
 *    rostopic pub /mavlink/to mavros_msgs/Mavlink "header: {...}
 *    msgid: 69  # MANUAL_CONTROL
 *    payload64: [1000, 500, 0, 0, 0, 1]"  # x=1000, y=500 for motors 1&2
 * 
 * 6. Emergency stop:
 *    rostopic pub /mavlink/to mavros_msgs/Mavlink "header: {...}
 *    msgid: 76  # COMMAND_LONG
 *    payload64: [400, 0, 0, 0, 0, 0, 0, 0, 1, 0]"  # MAV_CMD_COMPONENT_ARM_DISARM
 */

/**
 * @brief Available parameters that can be read/written from ROS:
 * 
 * Per-Motor Parameters (replace %d with motor ID 1-4):
 * - M%d_POS_KP    : Position control P gain (0-1000)
 * - M%d_POS_KI    : Position control I gain (0-100)
 * - M%d_POS_KD    : Position control D gain (0-100)
 * - M%d_VEL_KP    : Velocity control P gain (0-1000)
 * - M%d_VEL_KI    : Velocity control I gain (0-100)
 * - M%d_VEL_KD    : Velocity control D gain (0-100)
 * - M%d_MAX_VEL   : Maximum velocity in RPS (1-200)
 * - M%d_MAX_CUR   : Maximum current in mA (100-20000)
 * - M%d_MAX_TEMP  : Maximum temperature in °C (40-100)
 * - M%d_TIMEOUT   : Watchdog timeout in ms (50-5000)
 * 
 * Example ROS Python script to read/write parameters:
 * 
 * ```python
 * import rospy
 * from mavros_msgs.msg import Mavlink
 * from mavros_msgs.srv import ParamGet, ParamSet
 * 
 * # Read parameter
 * param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
 * response = param_get('M1_VEL_KP')
 * print(f"Motor 1 Velocity Kp: {response.value.real}")
 * 
 * # Write parameter  
 * param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
 * response = param_set('M1_VEL_KP', 50.0)
 * print(f"Parameter set success: {response.success}")
 * ```
 */

/**
 * @brief System status reporting
 */
void printSystemStatus() {
    printf("\n=== RoboMaster System Status ===\n");
    
    // CAN Manager status
    printf("CAN Manager: %s\n", can_manager.isStarted() ? "Running" : "Stopped");
    printf("CAN TX Errors: %u\n", can_manager.getTransmissionErrors());
    printf("CAN RX Errors: %u\n", can_manager.getReceptionErrors());
    
    // Motor status
    RoboMasterMotor* motors[] = {&motor1, &motor2, &motor3, &motor4};
    const char* motor_names[] = {"Motor 1", "Motor 2", "Motor 3", "Motor 4"};
    
    for (int i = 0; i < 4; i++) {
        const RoboMasterState& state = motors[i]->getState();
        const RoboMasterConfig& config = motors[i]->getConfig();
        
        printf("\n%s (ID %d):\n", motor_names[i], motors[i]->getId());
        printf("  Status: %d, Enabled: %s\n", 
               static_cast<int>(state.status), state.enabled ? "Yes" : "No");
        printf("  Position: %.2f rad, Velocity: %.2f RPS\n", 
               state.currentPositionRad, state.currentVelocityRPS);
        printf("  Current: %d mA, Temperature: %d°C\n", 
               state.currentMilliamps, state.temperatureCelsius);
        printf("  Control Mode: %d, Max Velocity: %.1f RPS\n", 
               static_cast<int>(state.controlMode), config.maxVelocityRPS);
        printf("  Errors: Timeout=%u, Saturation=%u\n", 
               state.timeoutCount, state.saturationCount);
    }
    
    printf("\n===============================\n");
}