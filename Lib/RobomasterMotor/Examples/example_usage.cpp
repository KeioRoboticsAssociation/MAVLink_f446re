/**
 * @file example_usage.cpp
 * @brief Example usage of the new RoboMaster Motor Library
 * 
 * This example demonstrates how to use the redesigned RoboMaster library
 * which follows the ServoMotor design philosophy with individual motor control.
 */

#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"
#include "stm32f4xx_hal.h"

// External CAN handle (configured in main.c)
extern CAN_HandleTypeDef hcan1;

// Global CAN manager and motors
RoboMasterCANManager can_manager;
RoboMasterMotor motor1;
RoboMasterMotor motor2;
RoboMasterMotor motor3;
RoboMasterMotor motor4;

/**
 * @brief Initialize RoboMaster motors
 */
void robomaster_init() {
    // Initialize CAN manager
    if (can_manager.init(&hcan1) != CANManagerStatus::OK) {
        // Handle error
        return;
    }
    
    // Create and configure motor 1
    if (motor1.create(1, &can_manager) != RoboMasterStatus::OK) {
        // Handle error
        return;
    }
    
    // Configure motor 1 with custom settings
    RoboMasterConfig config1;
    config1.maxVelocityRPS = 50.0f;           // 50 RPS max velocity
    config1.maxCurrent = 10000;               // 10A max current
    config1.positionKp = 15.0f;               // Higher position gain
    config1.velocityKp = 40.0f;               // Higher velocity gain
    config1.velocityKi = 0.2f;                // Some integral gain
    config1.startupMode = RoboMasterControlMode::VELOCITY;
    config1.watchdogTimeoutMs = 500;          // 500ms timeout
    
    if (motor1.init(config1) != RoboMasterStatus::OK) {
        // Handle error
        return;
    }
    
    // Create and configure motor 2 with position limits
    if (motor2.create(2, &can_manager) != RoboMasterStatus::OK) {
        return;
    }
    
    RoboMasterConfig config2;
    config2.positionLimitsEnabled = true;
    config2.minPositionRad = -10.0f * M_PI;   // -10 revolutions
    config2.maxPositionRad = 10.0f * M_PI;    // +10 revolutions
    config2.startupMode = RoboMasterControlMode::POSITION;
    config2.failSafeBehavior = FailSafeBehavior::HOLD_POSITION;
    
    if (motor2.init(config2) != RoboMasterStatus::OK) {
        return;
    }
    
    // Create motors 3 and 4 with default settings
    motor3.create(3, &can_manager);
    motor3.init();  // Use default configuration
    
    motor4.create(4, &can_manager);
    motor4.init();
    
    // Start the CAN manager
    if (can_manager.start() != CANManagerStatus::OK) {
        // Handle error
        return;
    }
    
    // Enable all motors
    motor1.setEnabled(true);
    motor2.setEnabled(true);
    motor3.setEnabled(true);
    motor4.setEnabled(true);
}

/**
 * @brief Control loop - call this periodically (e.g., in timer interrupt)
 */
void robomaster_control_loop() {
    // Update CAN manager (handles communication and motor updates)
    can_manager.update();
    
    // Example control logic
    static uint32_t counter = 0;
    counter++;
    
    // Motor 1: Velocity control with sine wave
    float velocity_cmd = 20.0f * sinf(counter * 0.01f);
    motor1.setVelocityRPS(velocity_cmd);
    
    // Motor 2: Position control
    float position_cmd = 5.0f * sinf(counter * 0.005f);
    motor2.setPositionRad(position_cmd);
    
    // Motor 3: Direct current control
    int16_t current_cmd = static_cast<int16_t>(5000.0f * sinf(counter * 0.02f));
    motor3.setCurrent(current_cmd);
    
    // Motor 4: Step response test
    if ((counter % 1000) < 500) {
        motor4.setVelocityRPS(10.0f);
    } else {
        motor4.setVelocityRPS(-10.0f);
    }
}

/**
 * @brief Get motor status information
 */
void print_motor_status() {
    // Motor 1 status
    if (motor1.getStatus() == RoboMasterStatus::OK) {
        float pos = motor1.getCurrentPosition();
        float vel = motor1.getCurrentVelocity();
        int16_t current = motor1.getCurrentMilliamps();
        uint8_t temp = motor1.getTemperature();
        
        printf("Motor 1: Pos=%.2f rad, Vel=%.2f RPS, I=%d mA, T=%d°C\n", 
               pos, vel, current, temp);
    } else {
        printf("Motor 1: Error status %d\n", static_cast<int>(motor1.getStatus()));
    }
    
    // Motor 2 status with limits
    RoboMasterLimits limits = motor2.getLimits();
    printf("Motor 2 limits: Vel=±%.1f RPS, I=%d to %d mA\n", 
           limits.maxVelocityRPS, limits.minCurrent, limits.maxCurrent);
}

/**
 * @brief Emergency stop all motors
 */
void emergency_stop() {
    can_manager.emergencyStop();
    
    // Or stop individual motors
    // motor1.setEnabled(false);
    // motor2.setEnabled(false);
    // motor3.setEnabled(false);
    // motor4.setEnabled(false);
}

/**
 * @brief Switch control modes dynamically
 */
void switch_control_modes() {
    // Switch motor 1 to position control
    motor1.setControlMode(RoboMasterControlMode::POSITION);
    motor1.setPositionRad(0.0f);  // Go to zero position
    
    // Switch motor 2 to velocity control
    motor2.setControlMode(RoboMasterControlMode::VELOCITY);
    motor2.setVelocityRPS(5.0f);  // 5 RPS constant velocity
}

/**
 * @brief CAN receive interrupt handler
 * Call this from HAL_CAN_RxFifo0MsgPendingCallback()
 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        can_manager.handleCANReceive();
    }
}

/**
 * @brief Example of configuration loading (if JSON support is added)
 */
void load_motor_configs() {
    // Example JSON configuration string
    const char* motor1_config_json = R"({
        "maxVelocityRPS": 60.0,
        "maxCurrent": 12000,
        "positionKp": 20.0,
        "velocityKp": 45.0,
        "velocityKi": 0.25,
        "startupMode": "VELOCITY",
        "watchdogTimeoutMs": 300
    })";
    
    // Load configuration (if JSON parser is implemented)
    // motor1.loadConfigFromJson(motor1_config_json);
    
    // Or load from file
    // motor1.loadConfigFromFile("/config/motor1.json");
}

/**
 * @brief Advanced control with rate limiting and safety
 */
void advanced_control_example() {
    // Get current motor state
    const RoboMasterState& state = motor1.getState();
    
    // Check if motor is operating normally
    if (state.status == RoboMasterStatus::OK && state.enabled) {
        // Apply smooth velocity changes with built-in rate limiting
        motor1.setVelocityRPS(50.0f);  // Library will apply acceleration limits
        
        // Monitor statistics
        if (state.saturationCount > 10) {
            printf("Motor 1: High saturation count, consider reducing commands\n");
        }
        
        if (state.timeoutCount > 0) {
            printf("Motor 1: Had %u timeouts, check communication\n", state.timeoutCount);
        }
    } else {
        // Handle error conditions
        switch (state.status) {
            case RoboMasterStatus::TIMEOUT:
                printf("Motor 1: Communication timeout\n");
                motor1.resetWatchdog();  // Try to recover
                break;
                
            case RoboMasterStatus::OVERHEAT:
                printf("Motor 1: Overheating at %d°C\n", state.temperatureCelsius);
                motor1.setEnabled(false);  // Protect motor
                break;
                
            case RoboMasterStatus::OVERCURRENT:
                printf("Motor 1: Overcurrent detected\n");
                motor1.setCurrent(0);  // Reduce current
                break;
                
            default:
                printf("Motor 1: Unknown error\n");
                break;
        }
    }
}