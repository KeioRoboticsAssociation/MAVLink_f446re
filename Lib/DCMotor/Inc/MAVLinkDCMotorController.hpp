#pragma once

#include "stm32f4xx_hal.h"
#include "DCMotor.hpp"
#include "Encoder.hpp"
#include "../../mavlink/c_library_v2_robomaster/robomaster/mavlink.h"
#include <cmath>

enum class MotorControlMode {
    OPEN_LOOP = 0,
    SPEED_CONTROL = 1,
    POSITION_CONTROL = 2,
    DISABLED = 3,
    DUTY_TO_POSITION = 4  // Apply duty cycle until target position is reached
};

enum class MotorStatus {
    OK = 0,
    NOT_INITIALIZED,
    ENCODER_ERROR,
    PWM_ERROR,
    TIMEOUT,
    CONFIG_ERROR,
    OVERCURRENT,
    POSITION_LIMIT
};

struct MotorConfig {
    uint8_t motor_id = 0;
    MotorControlMode mode = MotorControlMode::OPEN_LOOP;

    // PID parameters for speed control
    float speed_kp = 1.0f;
    float speed_ki = 0.0f;
    float speed_kd = 0.0f;
    float speed_max_integral = 100.0f;
    float speed_max_output = 1.0f;

    // PID parameters for position control
    float position_kp = 2.0f;
    float position_ki = 0.0f;
    float position_kd = 0.0f;
    float position_max_integral = 1000.0f;
    float position_max_output = 10.0f; // rad/s

    // Physical limits
    float max_speed_rad_s = 10.0f;
    float max_acceleration_rad_s2 = 50.0f;
    float position_limit_min_rad = -M_PI * 10.0f;
    float position_limit_max_rad = M_PI * 10.0f;
    bool use_position_limits = false;

    // Safety
    uint32_t watchdog_timeout_ms = 1000;
    float emergency_stop_deceleration = 100.0f;

    // Control loop timing
    uint32_t control_period_ms = 10; // 100Hz control loop
};

struct MotorState {
    uint8_t motor_id = 0;
    MotorControlMode mode = MotorControlMode::DISABLED;
    MotorStatus status = MotorStatus::NOT_INITIALIZED;

    // Current values
    float current_position_rad = 0.0f;
    float current_speed_rad_s = 0.0f;
    float current_duty_cycle = 0.0f;

    // Targets
    float target_position_rad = 0.0f;
    float target_speed_rad_s = 0.0f;
    float target_duty_cycle = 0.0f;

    // PID state for speed control
    float speed_error_integral = 0.0f;
    float speed_error_previous = 0.0f;

    // PID state for position control
    float position_error_integral = 0.0f;
    float position_error_previous = 0.0f;

    // Timing
    uint32_t last_command_time = 0;
    uint32_t last_update_time = 0;

    // Statistics
    uint32_t timeout_count = 0;
    uint32_t error_count = 0;

    bool enabled = false;
};

class MAVLinkDCMotorController {
public:
    MAVLinkDCMotorController();
    ~MAVLinkDCMotorController() = default;

    // Initialization
    MotorStatus init(uint8_t motor_id, DCMotor* motor, Encoder* encoder, UART_HandleTypeDef* uart, uint8_t system_id = 1);
    MotorStatus setConfig(const MotorConfig& config);

    // Control methods
    MotorStatus setMode(MotorControlMode mode);
    MotorStatus setTargetPosition(float position_rad);
    MotorStatus setTargetSpeed(float speed_rad_s);
    MotorStatus setTargetDutyCycle(float duty_cycle);
    MotorStatus enable();
    MotorStatus disable();
    MotorStatus emergencyStop();

    // Status and getters
    const MotorState& getState() const { return state_; }
    const MotorConfig& getConfig() const { return config_; }
    MotorStatus getStatus() const { return state_.status; }

    // Main update function (call regularly from main loop)
    void update();

    // Duty-to-position control mode
    MotorStatus setDutyToPositionParams(float duty_cycle, float target_angle_rad);

    // MAVLink communication
    void handleMessage(mavlink_message_t* msg);
    void processReceivedByte(uint8_t byte);
    void sendTelemetry();

private:
    static constexpr uint32_t TELEMETRY_SEND_RATE_MS = 100; // 10Hz telemetry
    static constexpr float VELOCITY_FILTER_ALPHA = 0.8f;   // Low-pass filter for velocity

    // Hardware interfaces
    DCMotor* motor_;
    Encoder* encoder_;
    UART_HandleTypeDef* uart_;
    uint8_t system_id_;

    // State and configuration
    MotorConfig config_;
    MotorState state_;
    bool initialized_;

    // MAVLink parsing
    mavlink_status_t rx_status_;
    mavlink_message_t rx_msg_;

    // Timing
    uint32_t last_telemetry_send_;
    uint32_t last_control_update_;

    // Velocity calculation
    float previous_position_rad_;
    uint32_t previous_position_time_;
    float filtered_velocity_rad_s_;

    // Duty-to-position control mode
    float duty_to_position_target_rad_;
    float duty_to_position_duty_;
    uint32_t duty_to_position_start_time_;
    float duty_to_position_start_rad_;
    int duty_to_position_direction_; // 1 for positive direction, -1 for negative
    static constexpr uint32_t DUTY_TO_POSITION_TIMEOUT_MS = 10000; // 10 second timeout
    static constexpr float POSITION_TOLERANCE_RAD = 0.1f; // ~5.7 degrees

    // Control algorithms
    void updateControlLoop();
    void updateDutyToPositionControl();
    float calculateSpeedPID(float target_speed, float current_speed, float dt);
    float calculatePositionPID(float target_position, float current_position, float dt);
    void applyDutyCycleLimit(float& duty_cycle);
    void updateVelocityEstimate();

    // Safety and limits
    bool checkPositionLimits(float position_rad);
    bool checkWatchdog();
    void handleTimeout();
    void handleEmergencyStop();

    // MAVLink message handlers
    void handleMotorCommand(mavlink_message_t* msg);
    void handleMotorConfigSet(mavlink_message_t* msg);
    void handleMotorConfigGet(mavlink_message_t* msg);
    void handlePositionTarget(mavlink_message_t* msg);
    void handleRoboMasterMotorControl(mavlink_message_t* msg);
    void handleSpeedTarget(mavlink_message_t* msg);

    // MAVLink message senders
    void sendMotorStatus();
    void sendMotorConfig();
    void sendPositionFeedback();
    void sendSpeedFeedback();
    void sendMessage(mavlink_message_t* msg);

    // Utility functions
    uint32_t getCurrentTimeMs() const;
    float constrainFloat(float value, float min_val, float max_val);
    float wrapAngle(float angle_rad);
};