#pragma once

// Hybrid Robot Configuration Template
// Complex robot with servos, DC motors, and RoboMaster motors for versatile applications

#include "../robot_config.hpp"

namespace Config {
namespace Templates {

// Hybrid servo configurations (for camera gimbal)
constexpr std::array<Devices::ServoInstanceConfig, 2> HYBRID_SERVO_CONFIGS = {{
    { // Camera Pan
        .id = 1,
        .initial_offset = 0.0f,
        .min_angle = -180.0f,
        .max_angle = 180.0f,
        .pulse_min_us = 600,
        .pulse_max_us = 2400,
        .pulse_neutral_us = 1500,
        .direction_inverted = false,
        .max_velocity_deg_per_s = 90.0f,
        .max_acceleration_deg_per_s2 = 180.0f,
        .watchdog_timeout_ms = 800,
        .startup_angle_deg = 0.0f,
        .start_disabled = false,
        .failsafe_behavior = Devices::ServoInstanceConfig::FailSafeBehavior::HOLD_POSITION
    },
    { // Camera Tilt
        .id = 2,
        .initial_offset = 0.0f,
        .min_angle = -90.0f,
        .max_angle = 30.0f,
        .pulse_min_us = 600,
        .pulse_max_us = 2400,
        .pulse_neutral_us = 1500,
        .direction_inverted = true,
        .max_velocity_deg_per_s = 60.0f,
        .max_acceleration_deg_per_s2 = 120.0f,
        .watchdog_timeout_ms = 800,
        .startup_angle_deg = -30.0f,  // Look down initially
        .start_disabled = false,
        .failsafe_behavior = Devices::ServoInstanceConfig::FailSafeBehavior::HOLD_POSITION
    }
}};

// Hybrid DC motor configuration (for precise positioning)
constexpr std::array<Devices::DCMotorInstanceConfig, 1> HYBRID_DC_CONFIGS = {{
    { // Lift mechanism
        .id = 10,
        .speed_kp = 0.8f,
        .speed_ki = 0.4f,
        .speed_kd = 0.02f,
        .speed_max_integral = 3.0f,
        .speed_max_output = 0.9f,
        .position_kp = 2.0f,
        .position_ki = 0.05f,
        .position_kd = 0.1f,
        .position_max_integral = 30.0f,
        .position_max_output = 12.0f,
        .max_speed_rad_s = 15.0f,
        .max_acceleration_rad_s2 = 50.0f,
        .position_limit_min_rad = 0.0f,     // No negative lift
        .position_limit_max_rad = 62.832f,  // 10 revolutions max
        .use_position_limits = true,
        .watchdog_timeout_ms = 1000,
        .control_period_ms = 10,  // 100Hz control
        .direction_inverted = false
    }
}};

// RoboMaster motor configurations (for mobile base)
constexpr std::array<Devices::RoboMasterInstanceConfig, 2> HYBRID_ROBOMASTER_CONFIGS = {{
    { // Left wheel motor
        .id = 20,
        .can_id = 1,
        .angle_kp = 0.15f,
        .angle_ki = 0.01f,
        .angle_kd = 0.005f,
        .speed_kp = 0.2f,
        .speed_ki = 0.02f,
        .speed_kd = 0.001f,
        .max_speed_rad_s = 50.0f,  // High speed for mobility
        .max_acceleration_rad_s2 = 100.0f,
        .watchdog_timeout_ms = 1500
    },
    { // Right wheel motor
        .id = 21,
        .can_id = 2,
        .angle_kp = 0.15f,
        .angle_ki = 0.01f,
        .angle_kd = 0.005f,
        .speed_kp = 0.2f,
        .speed_ki = 0.02f,
        .speed_kd = 0.001f,
        .max_speed_rad_s = 50.0f,  // High speed for mobility
        .max_acceleration_rad_s2 = 100.0f,
        .watchdog_timeout_ms = 1500
    }
}};

// Encoder configuration for DC motor
constexpr std::array<Devices::EncoderInstanceConfig, 1> HYBRID_ENCODER_CONFIGS = {{
    { // Lift encoder
        .id = 10,
        .cpr = 4096,
        .invert_a = false,
        .invert_b = true,
        .use_z = true,
        .watchdog_timeout_ms = 200,
        .offset_counts = 0,
        .wrap_around = false,
        .mode = Devices::EncoderInstanceConfig::Mode::TIM_ENCODER_MODE
    }
}};

// Hardware mappings
constexpr Hardware::TimerConfig HYBRID_SERVO_TIMERS[] = {
    {2, 1},   // TIM2 CH1 - Camera Pan
    {2, 2}    // TIM2 CH2 - Camera Tilt
};

constexpr Hardware::TimerConfig HYBRID_DC_TIMERS[] = {
    {3, 1}    // TIM3 CH1 - Lift Motor
};

constexpr Hardware::TimerConfig HYBRID_ENCODER_TIMERS[] = {
    {1, 0}    // TIM1 - Lift Encoder
};

// Motor instance mapping
constexpr std::array<MotorInstance, 5> HYBRID_MOTOR_INSTANCES = {{
    {1,  MotorInstance::Type::SERVO,      2, 1, true},  // Camera Pan
    {2,  MotorInstance::Type::SERVO,      2, 2, true},  // Camera Tilt
    {10, MotorInstance::Type::DC_MOTOR,   3, 1, true},  // Lift
    {20, MotorInstance::Type::ROBOMASTER, 0, 0, true},  // Left wheel
    {21, MotorInstance::Type::ROBOMASTER, 0, 0, true}   // Right wheel
}};

// Robot settings
namespace HybridRobot {
    static constexpr const char* ROBOT_NAME = "Hybrid Multi-Motor Robot";
    static constexpr const char* ROBOT_VERSION = "1.0.0";
    static constexpr uint8_t MAX_SERVOS = 2;
    static constexpr uint8_t MAX_DC_MOTORS = 1;
    static constexpr uint8_t MAX_ROBOMASTER_MOTORS = 2;
    static constexpr uint32_t MAIN_LOOP_FREQUENCY_HZ = 100;  // 100Hz for balanced performance
    static constexpr uint32_t TELEMETRY_RATE_HZ = 15;
    static constexpr bool ENABLE_LIMIT_SWITCHES = true;
    static constexpr bool ENABLE_VOLTAGE_MONITORING = true;
}

} // namespace Templates
} // namespace Config