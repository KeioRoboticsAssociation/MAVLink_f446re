#pragma once

// Robot Configuration Template Generator
// Provides macros and utilities for creating new robot configurations

#include "../robot_config.hpp"

namespace Config {
namespace TemplateGenerator {

// Template selection macros - choose one for your robot type
// #define USE_BASIC_SERVO_ROBOT
// #define USE_PRECISION_DC_ROBOT
// #define USE_HYBRID_ROBOT
// #define USE_CUSTOM_ROBOT

#ifdef USE_BASIC_SERVO_ROBOT
    #include "basic_servo_robot.hpp"
    namespace Active = Templates::BasicServoRobot;
    constexpr auto ACTIVE_SERVO_CONFIGS = Templates::BASIC_SERVO_CONFIGS;
    constexpr auto ACTIVE_MOTOR_INSTANCES = Templates::BASIC_MOTOR_INSTANCES;
#elif defined(USE_PRECISION_DC_ROBOT)
    #include "precision_dc_robot.hpp"
    namespace Active = Templates::PrecisionDCRobot;
    constexpr auto ACTIVE_DC_CONFIGS = Templates::PRECISION_DC_CONFIGS;
    constexpr auto ACTIVE_ENCODER_CONFIGS = Templates::PRECISION_ENCODER_CONFIGS;
    constexpr auto ACTIVE_MOTOR_INSTANCES = Templates::PRECISION_MOTOR_INSTANCES;
#elif defined(USE_HYBRID_ROBOT)
    #include "hybrid_robot.hpp"
    namespace Active = Templates::HybridRobot;
    constexpr auto ACTIVE_SERVO_CONFIGS = Templates::HYBRID_SERVO_CONFIGS;
    constexpr auto ACTIVE_DC_CONFIGS = Templates::HYBRID_DC_CONFIGS;
    constexpr auto ACTIVE_ROBOMASTER_CONFIGS = Templates::HYBRID_ROBOMASTER_CONFIGS;
    constexpr auto ACTIVE_ENCODER_CONFIGS = Templates::HYBRID_ENCODER_CONFIGS;
    constexpr auto ACTIVE_MOTOR_INSTANCES = Templates::HYBRID_MOTOR_INSTANCES;
#endif

// Configuration validation helpers
template<size_t N>
constexpr bool validateServoConfigs(const std::array<Devices::ServoInstanceConfig, N>& configs) {
    for (const auto& config : configs) {
        if (config.max_angle <= config.min_angle) return false;
        if (config.pulse_max_us <= config.pulse_min_us) return false;
        if (config.max_velocity_deg_per_s <= 0) return false;
        if (config.max_acceleration_deg_per_s2 <= 0) return false;
        if (config.watchdog_timeout_ms == 0) return false;
    }
    return true;
}

template<size_t N>
constexpr bool validateDCMotorConfigs(const std::array<Devices::DCMotorInstanceConfig, N>& configs) {
    for (const auto& config : configs) {
        if (config.max_speed_rad_s <= 0) return false;
        if (config.max_acceleration_rad_s2 <= 0) return false;
        if (config.watchdog_timeout_ms == 0) return false;
        if (config.control_period_ms == 0) return false;
    }
    return true;
}

template<size_t N>
constexpr bool validateMotorInstances(const std::array<MotorInstance, N>& instances) {
    for (const auto& instance : instances) {
        if (instance.id == 0) return false;  // ID 0 is invalid
        if (!instance.enabled) continue;     // Skip disabled instances

        // Validate timer/channel combinations for PWM-based motors
        if (instance.type == MotorInstance::Type::SERVO ||
            instance.type == MotorInstance::Type::DC_MOTOR) {
            if (instance.timer_id == 0 || instance.channel == 0) return false;
        }
    }
    return true;
}

// Template generation macros
#define GENERATE_SERVO_CONFIG(id, name, min_deg, max_deg, neutral_us, vel_dps, acc_dps2) \
    { \
        .id = id, \
        .initial_offset = 0.0f, \
        .min_angle = min_deg, \
        .max_angle = max_deg, \
        .pulse_min_us = 500, \
        .pulse_max_us = 2500, \
        .pulse_neutral_us = neutral_us, \
        .direction_inverted = false, \
        .max_velocity_deg_per_s = vel_dps, \
        .max_acceleration_deg_per_s2 = acc_dps2, \
        .watchdog_timeout_ms = 1000, \
        .startup_angle_deg = 0.0f, \
        .start_disabled = false, \
        .failsafe_behavior = Devices::ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION \
    }

#define GENERATE_DC_MOTOR_CONFIG(id, speed_kp, speed_ki, pos_kp, max_speed, max_accel) \
    { \
        .id = id, \
        .speed_kp = speed_kp, \
        .speed_ki = speed_ki, \
        .speed_kd = 0.0f, \
        .speed_max_integral = 5.0f, \
        .speed_max_output = 1.0f, \
        .position_kp = pos_kp, \
        .position_ki = 0.0f, \
        .position_kd = 0.0f, \
        .position_max_integral = 100.0f, \
        .position_max_output = max_speed, \
        .max_speed_rad_s = max_speed, \
        .max_acceleration_rad_s2 = max_accel, \
        .position_limit_min_rad = -314.159f, \
        .position_limit_max_rad = 314.159f, \
        .use_position_limits = true, \
        .watchdog_timeout_ms = 2000, \
        .control_period_ms = 10, \
        .direction_inverted = false \
    }

#define GENERATE_ENCODER_CONFIG(id, cpr, invert_a, invert_b, use_z) \
    { \
        .id = id, \
        .cpr = cpr, \
        .invert_a = invert_a, \
        .invert_b = invert_b, \
        .use_z = use_z, \
        .watchdog_timeout_ms = 500, \
        .offset_counts = 0, \
        .wrap_around = false, \
        .mode = Devices::EncoderInstanceConfig::Mode::TIM_ENCODER_MODE \
    }

#define GENERATE_MOTOR_INSTANCE(id, type_enum, timer, chan) \
    {id, MotorInstance::Type::type_enum, timer, chan, true}

// Quick robot template generators
#define SIMPLE_SERVO_ROBOT(servo_count) \
    static_assert(servo_count <= 8, "Maximum 8 servos supported"); \
    namespace Robot { \
        static constexpr const char* ROBOT_NAME = "Custom Servo Robot"; \
        static constexpr uint8_t MAX_SERVOS = servo_count; \
        static constexpr uint8_t MAX_DC_MOTORS = 0; \
        static constexpr uint8_t MAX_ROBOMASTER_MOTORS = 0; \
    }

#define SIMPLE_DC_ROBOT(dc_count) \
    static_assert(dc_count <= 4, "Maximum 4 DC motors supported"); \
    namespace Robot { \
        static constexpr const char* ROBOT_NAME = "Custom DC Motor Robot"; \
        static constexpr uint8_t MAX_SERVOS = 0; \
        static constexpr uint8_t MAX_DC_MOTORS = dc_count; \
        static constexpr uint8_t MAX_ROBOMASTER_MOTORS = 0; \
    }

// Documentation generator helper
#define DOCUMENT_ROBOT_CONFIG(robot_name, description, author) \
    /* \
     * Robot Configuration: robot_name \
     * Description: description \
     * Author: author \
     * Generated: __DATE__ __TIME__ \
     * Architecture: STM32F446 + SystemContext \
     */ \
    namespace Documentation { \
        static constexpr const char* NAME = robot_name; \
        static constexpr const char* DESCRIPTION = description; \
        static constexpr const char* AUTHOR = author; \
        static constexpr const char* GENERATION_DATE = __DATE__ " " __TIME__; \
    }

} // namespace TemplateGenerator
} // namespace Config