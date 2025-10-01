/**
 * @file robot_config.hpp
 * @brief Compile-time device and system configuration for the robot.
 */

#pragma once

#include <cstdint>
#include <array>

namespace Config {

/**
 * @brief Hardware pin and timer definitions.
 */
namespace Hardware {
    /**
     * @brief Configuration for a hardware timer.
     */
    struct TimerConfig {
        uint8_t timer_id;
        uint32_t channel;
    };

    /**
     * @brief Configuration for a GPIO pin.
     */
    struct GPIOConfig {
        uint32_t port;
        uint16_t pin;
        bool active_low;
    };

    constexpr TimerConfig SERVO_TIMERS[] = {
        {2, 1}, {2, 2}, {12, 1}, {12, 2}
    };

    constexpr TimerConfig DC_MOTOR_TIMERS[] = {
        {3, 1}
    };

    constexpr TimerConfig ENCODER_TIMERS[] = {
        {1, 0}, {4, 0}
    };

    constexpr GPIOConfig LIMIT_SWITCHES[] = {
        {1, 0, true}
    };

    constexpr GPIOConfig DC_MOTOR_DIR_PINS[] = {
        {1, 8, false}
    };
}

/**
 * @brief Device-specific configurations.
 */
namespace Devices {
    /**
     * @brief Configuration for a single servo instance.
     */
    struct ServoInstanceConfig {
        uint8_t id;
        float initial_offset;
        float min_angle;
        float max_angle;
        uint16_t pulse_min_us;
        uint16_t pulse_max_us;
        uint16_t pulse_neutral_us;
        bool direction_inverted;
        float max_velocity_deg_per_s;
        float max_acceleration_deg_per_s2;
        uint32_t watchdog_timeout_ms;
        float startup_angle_deg;
        bool start_disabled;
        enum class FailSafeBehavior : uint8_t {
            NEUTRAL_POSITION, HOLD_POSITION, DISABLE
        } failsafe_behavior;
    };

    /**
     * @brief Configuration for a single DC motor instance.
     */
    struct DCMotorInstanceConfig {
        uint8_t id;
        float speed_kp, speed_ki, speed_kd;
        float speed_max_integral, speed_max_output;
        float position_kp, position_ki, position_kd;
        float position_max_integral, position_max_output;
        float max_speed_rad_s, max_acceleration_rad_s2;
        float position_limit_min_rad, position_limit_max_rad;
        bool use_position_limits;
        uint32_t watchdog_timeout_ms;
        uint32_t control_period_ms;
        bool direction_inverted;
    };

    /**
     * @brief Configuration for a single encoder instance.
     */
    struct EncoderInstanceConfig {
        uint8_t id;
        uint32_t cpr;
        bool invert_a, invert_b, use_z;
        uint32_t watchdog_timeout_ms;
        int32_t offset_counts;
        bool wrap_around;
        enum class Mode : uint8_t {
            TIM_ENCODER_MODE, QUADRATURE_X2, QUADRATURE_X4
        } mode;
    };

    /**
     * @brief Configuration for a single RoboMaster motor instance.
     */
    struct RoboMasterInstanceConfig {
        uint8_t id, can_id;
        float angle_kp, angle_ki, angle_kd;
        float speed_kp, speed_ki, speed_kd;
        float max_speed_rad_s, max_acceleration_rad_s2;
        uint32_t watchdog_timeout_ms;
    };

    constexpr std::array<ServoInstanceConfig, 4> SERVO_CONFIGS = {{
        {1, 0.0f, -60.0f, 60.0f, 500, 2000, 500, false, 120.0f, 240.0f, 500, 0.0f, false, ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION},
        {2, 0.0f, -60.0f, 60.0f, 500, 2000, 1250, false, 120.0f, 240.0f, 500, 0.0f, false, ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION},
        {3, 0.0f, -60.0f, 60.0f, 500, 2000, 1250, false, 120.0f, 240.0f, 500, 0.0f, false, ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION},
        {4, 0.0f, -60.0f, 60.0f, 500, 2000, 1250, false, 120.0f, 240.0f, 500, 0.0f, false, ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION}
    }};

    constexpr std::array<DCMotorInstanceConfig, 1> DC_MOTOR_CONFIGS = {{
        {10, 0.1f, 0.1f, 0.0f, 0.3f, 0.5f, 0.1f, 0.0f, 0.0f, 100.0f, 10.0f, 15.0f, 50.0f, -314.159f, 314.159f, true, 2000, 10, true}
    }};

    constexpr std::array<EncoderInstanceConfig, 4> ENCODER_CONFIGS = {{
        {1, 8192, true, false, false, 500, 0, false, EncoderInstanceConfig::Mode::TIM_ENCODER_MODE},
        {2, 2048, false, true, true, 300, 0, true, EncoderInstanceConfig::Mode::TIM_ENCODER_MODE},
        {3, 4096, false, false, false, 200, 0, false, EncoderInstanceConfig::Mode::TIM_ENCODER_MODE},
        {4, 1024, false, false, false, 500, 0, true, EncoderInstanceConfig::Mode::TIM_ENCODER_MODE}
    }};

    constexpr std::array<RoboMasterInstanceConfig, 1> ROBOMASTER_CONFIGS = {{
        {20, 5, 0.1f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 10.0f, 30.0f, 1000}
    }};
}

/**
 * @brief System-wide robot configuration.
 */
namespace Robot {
    static constexpr const char* ROBOT_NAME = "MAVLink Test Robot";
    static constexpr const char* ROBOT_VERSION = "2.0.0";
    static constexpr uint32_t CONFIG_VERSION = 1;
    static constexpr uint8_t MAVLINK_SYSTEM_ID = 1;
    static constexpr uint8_t MAVLINK_COMPONENT_ID = 1;
    static constexpr uint32_t MAVLINK_HEARTBEAT_INTERVAL_MS = 1000;
    static constexpr uint32_t UART_BAUD_RATE = 115200;
    static constexpr uint8_t MAX_SERVOS = 4;
    static constexpr uint8_t MAX_DC_MOTORS = 2;
    static constexpr uint8_t MAX_ENCODERS = 4;
    static constexpr uint8_t MAX_ROBOMASTER_MOTORS = 2;
    static constexpr uint32_t SYSTEM_WATCHDOG_TIMEOUT_MS = 5000;
    static constexpr uint32_t EMERGENCY_STOP_TIMEOUT_MS = 100;
    static constexpr bool ENABLE_LIMIT_SWITCHES = true;
    static constexpr bool ENABLE_VOLTAGE_MONITORING = true;
    static constexpr bool ENABLE_TEMPERATURE_MONITORING = false;
    static constexpr uint32_t MAIN_LOOP_FREQUENCY_HZ = 100;
    static constexpr uint32_t TELEMETRY_RATE_HZ = 10;
    static constexpr uint32_t CONTROL_LOOP_FREQUENCY_HZ = 100;
}

/**
 * @brief Convenience accessors for runtime configuration retrieval.
 */
namespace ConfigAccessor {
    constexpr const Devices::ServoInstanceConfig* getServoConfig(uint8_t id) {
        for (const auto& config : Devices::SERVO_CONFIGS) {
            if (config.id == id) return &config;
        }
        return nullptr;
    }

    constexpr const Devices::DCMotorInstanceConfig* getDCMotorConfig(uint8_t id) {
        for (const auto& config : Devices::DC_MOTOR_CONFIGS) {
            if (config.id == id) return &config;
        }
        return nullptr;
    }

    constexpr const Devices::EncoderInstanceConfig* getEncoderConfig(uint8_t id) {
        for (const auto& config : Devices::ENCODER_CONFIGS) {
            if (config.id == id) return &config;
        }
        return nullptr;
    }

    constexpr const Devices::RoboMasterInstanceConfig* getRoboMasterConfig(uint8_t id) {
        for (const auto& config : Devices::ROBOMASTER_CONFIGS) {
            if (config.id == id) return &config;
        }
        return nullptr;
    }
}

} // namespace Config