#pragma once

// Basic Servo Robot Configuration Template
// A simple 2-servo robot for basic movement control

#include "../robot_config.hpp"

namespace Config {
namespace Templates {

// Basic servo robot with 2 servos
constexpr std::array<Devices::ServoInstanceConfig, 2> BASIC_SERVO_CONFIGS = {{
    { // Pan Servo
        .id = 1,
        .initial_offset = 0.0f,
        .min_angle = -90.0f,
        .max_angle = 90.0f,
        .pulse_min_us = 500,
        .pulse_max_us = 2500,
        .pulse_neutral_us = 1500,
        .direction_inverted = false,
        .max_velocity_deg_per_s = 180.0f,
        .max_acceleration_deg_per_s2 = 360.0f,
        .watchdog_timeout_ms = 1000,
        .startup_angle_deg = 0.0f,
        .start_disabled = false,
        .failsafe_behavior = Devices::ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION
    },
    { // Tilt Servo
        .id = 2,
        .initial_offset = 0.0f,
        .min_angle = -45.0f,
        .max_angle = 45.0f,
        .pulse_min_us = 500,
        .pulse_max_us = 2500,
        .pulse_neutral_us = 1500,
        .direction_inverted = false,
        .max_velocity_deg_per_s = 120.0f,
        .max_acceleration_deg_per_s2 = 240.0f,
        .watchdog_timeout_ms = 1000,
        .startup_angle_deg = 0.0f,
        .start_disabled = false,
        .failsafe_behavior = Devices::ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION
    }
}};

// Hardware mapping for basic servo robot
constexpr Hardware::TimerConfig BASIC_SERVO_TIMERS[] = {
    {2, 1},  // TIM2 CH1 - Pan servo
    {2, 2}   // TIM2 CH2 - Tilt servo
};

// Motor instance mapping
constexpr std::array<MotorInstance, 2> BASIC_MOTOR_INSTANCES = {{
    {1, MotorInstance::Type::SERVO, 2, 1, true},
    {2, MotorInstance::Type::SERVO, 2, 2, true}
}};

// Robot settings
namespace BasicServoRobot {
    static constexpr const char* ROBOT_NAME = "Basic Servo Robot";
    static constexpr const char* ROBOT_VERSION = "1.0.0";
    static constexpr uint8_t MAX_SERVOS = 2;
    static constexpr uint8_t MAX_DC_MOTORS = 0;
    static constexpr uint8_t MAX_ROBOMASTER_MOTORS = 0;
    static constexpr uint32_t MAIN_LOOP_FREQUENCY_HZ = 50;  // 50Hz for basic servo control
}

} // namespace Templates
} // namespace Config