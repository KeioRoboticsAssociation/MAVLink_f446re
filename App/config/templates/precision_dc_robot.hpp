#pragma once

// Precision DC Motor Robot Configuration Template
// High-precision robot with DC motors and encoders for accurate positioning

#include "../robot_config.hpp"

namespace Config {
namespace Templates {

// Precision DC motor configurations
constexpr std::array<Devices::DCMotorInstanceConfig, 2> PRECISION_DC_CONFIGS = {{
    { // Axis 1 - High precision
        .id = 11,
        .speed_kp = 1.2f,
        .speed_ki = 0.8f,
        .speed_kd = 0.05f,
        .speed_max_integral = 5.0f,
        .speed_max_output = 0.8f,
        .position_kp = 5.0f,
        .position_ki = 0.1f,
        .position_kd = 0.2f,
        .position_max_integral = 50.0f,
        .position_max_output = 20.0f,
        .max_speed_rad_s = 25.0f,
        .max_acceleration_rad_s2 = 100.0f,
        .position_limit_min_rad = -628.318f, // -200 revolutions
        .position_limit_max_rad = 628.318f,  // +200 revolutions
        .use_position_limits = true,
        .watchdog_timeout_ms = 500,
        .control_period_ms = 5,  // 200Hz control
        .direction_inverted = false
    },
    { // Axis 2 - High precision
        .id = 12,
        .speed_kp = 1.0f,
        .speed_ki = 0.6f,
        .speed_kd = 0.03f,
        .speed_max_integral = 4.0f,
        .speed_max_output = 0.7f,
        .position_kp = 4.0f,
        .position_ki = 0.08f,
        .position_kd = 0.15f,
        .position_max_integral = 40.0f,
        .position_max_output = 15.0f,
        .max_speed_rad_s = 20.0f,
        .max_acceleration_rad_s2 = 80.0f,
        .position_limit_min_rad = -314.159f, // -100 revolutions
        .position_limit_max_rad = 314.159f,  // +100 revolutions
        .use_position_limits = true,
        .watchdog_timeout_ms = 500,
        .control_period_ms = 5,  // 200Hz control
        .direction_inverted = true
    }
}};

// High-resolution encoder configurations
constexpr std::array<Devices::EncoderInstanceConfig, 2> PRECISION_ENCODER_CONFIGS = {{
    { // Encoder 1 - High resolution
        .id = 11,
        .cpr = 16384,  // 14-bit encoder
        .invert_a = false,
        .invert_b = false,
        .use_z = true,
        .watchdog_timeout_ms = 100,
        .offset_counts = 0,
        .wrap_around = false,
        .mode = Devices::EncoderInstanceConfig::Mode::TIM_ENCODER_MODE
    },
    { // Encoder 2 - High resolution
        .id = 12,
        .cpr = 16384,  // 14-bit encoder
        .invert_a = true,   // Inverted for mechanical setup
        .invert_b = false,
        .use_z = true,
        .watchdog_timeout_ms = 100,
        .offset_counts = 0,
        .wrap_around = false,
        .mode = Devices::EncoderInstanceConfig::Mode::TIM_ENCODER_MODE
    }
}};

// Hardware mapping for precision robot
constexpr Hardware::TimerConfig PRECISION_DC_TIMERS[] = {
    {1, 1},  // TIM1 CH1 - DC Motor 1
    {1, 2}   // TIM1 CH2 - DC Motor 2
};

constexpr Hardware::TimerConfig PRECISION_ENCODER_TIMERS[] = {
    {3, 0},  // TIM3 - Encoder 1
    {4, 0}   // TIM4 - Encoder 2
};

constexpr Hardware::GPIOConfig PRECISION_DIR_PINS[] = {
    {1, 8, false},  // GPIOB PIN8 - Motor 1 direction
    {1, 9, false}   // GPIOB PIN9 - Motor 2 direction
};

// Motor instance mapping
constexpr std::array<MotorInstance, 2> PRECISION_MOTOR_INSTANCES = {{
    {11, MotorInstance::Type::DC_MOTOR, 1, 1, true},
    {12, MotorInstance::Type::DC_MOTOR, 1, 2, true}
}};

// Robot settings
namespace PrecisionDCRobot {
    static constexpr const char* ROBOT_NAME = "Precision DC Motor Robot";
    static constexpr const char* ROBOT_VERSION = "1.0.0";
    static constexpr uint8_t MAX_SERVOS = 0;
    static constexpr uint8_t MAX_DC_MOTORS = 2;
    static constexpr uint8_t MAX_ROBOMASTER_MOTORS = 0;
    static constexpr uint32_t MAIN_LOOP_FREQUENCY_HZ = 200;  // 200Hz for precision control
    static constexpr uint32_t TELEMETRY_RATE_HZ = 20;
}

} // namespace Templates
} // namespace Config