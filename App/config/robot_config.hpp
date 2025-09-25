#pragma once

#include <cstdint>
#include <array>

namespace Config {

// Robot Configuration - Compile-time device and system configuration
// This replaces JSON files with embedded STM32-compatible compile-time configuration

// Hardware pin definitions
namespace Hardware {
    struct TimerConfig {
        uint8_t timer_id;
        uint32_t channel;
    };

    struct GPIOConfig {
        uint32_t port;      // GPIO port (encoded as integer for compile-time)
        uint16_t pin;
        bool active_low;
    };

    // Hardware mappings
    constexpr TimerConfig SERVO_TIMERS[] = {
        {2, 1},  // TIM2 CH1 - Servo 1
        {2, 2},  // TIM2 CH2 - Servo 2
        {12, 1}, // TIM12 CH1 - Servo 3
        {12, 2}  // TIM12 CH2 - Servo 4
    };

    constexpr TimerConfig DC_MOTOR_TIMERS[] = {
        {3, 1}   // TIM3 CH1 - DC Motor 1
    };

    constexpr TimerConfig ENCODER_TIMERS[] = {
        {1, 0},  // TIM1 - Encoder for DC Motor 1
        {4, 0}   // TIM4 - Additional encoder
    };

    // GPIO pin definitions
    constexpr GPIOConfig LIMIT_SWITCHES[] = {
        {1, 0, true}  // GPIOB PIN0 - Limit switch (active low)
    };

    constexpr GPIOConfig DC_MOTOR_DIR_PINS[] = {
        {1, 8, false}  // GPIOB PIN8 - DC Motor direction
    };
}

// Device configurations
namespace Devices {

    // Servo-specific configuration
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
            NEUTRAL_POSITION,
            HOLD_POSITION,
            DISABLE
        } failsafe_behavior;
    };

    // DC Motor configuration
    struct DCMotorInstanceConfig {
        uint8_t id;
        // PID parameters for speed control
        float speed_kp;
        float speed_ki;
        float speed_kd;
        float speed_max_integral;
        float speed_max_output;
        // PID parameters for position control
        float position_kp;
        float position_ki;
        float position_kd;
        float position_max_integral;
        float position_max_output;
        // Physical limits
        float max_speed_rad_s;
        float max_acceleration_rad_s2;
        float position_limit_min_rad;
        float position_limit_max_rad;
        bool use_position_limits;
        // Control settings
        uint32_t watchdog_timeout_ms;
        uint32_t control_period_ms;
        bool direction_inverted;
    };

    // Encoder configuration
    struct EncoderInstanceConfig {
        uint8_t id;
        uint32_t cpr;                 // Counts per revolution
        bool invert_a;
        bool invert_b;
        bool use_z;
        uint32_t watchdog_timeout_ms;
        int32_t offset_counts;
        bool wrap_around;
        enum class Mode : uint8_t {
            TIM_ENCODER_MODE,
            QUADRATURE_X2,
            QUADRATURE_X4
        } mode;
    };

    // RoboMaster motor configuration
    struct RoboMasterInstanceConfig {
        uint8_t id;
        uint8_t can_id;
        float angle_kp;
        float angle_ki;
        float angle_kd;
        float speed_kp;
        float speed_ki;
        float speed_kd;
        float max_speed_rad_s;
        float max_acceleration_rad_s2;
        uint32_t watchdog_timeout_ms;
    };

    // Servo configurations (from JSON data)
    constexpr std::array<ServoInstanceConfig, 4> SERVO_CONFIGS = {{
        { // Servo 1
            .id = 1,
            .initial_offset = 0.0f,
            .min_angle = -60.0f,
            .max_angle = 60.0f,
            .pulse_min_us = 500,
            .pulse_max_us = 2000,
            .pulse_neutral_us = 500,
            .direction_inverted = false,
            .max_velocity_deg_per_s = 120.0f,
            .max_acceleration_deg_per_s2 = 240.0f,
            .watchdog_timeout_ms = 500,
            .startup_angle_deg = 0.0f,
            .start_disabled = false,
            .failsafe_behavior = ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION
        },
        { // Servo 2
            .id = 2,
            .initial_offset = 0.0f,
            .min_angle = -60.0f,
            .max_angle = 60.0f,
            .pulse_min_us = 500,
            .pulse_max_us = 2000,
            .pulse_neutral_us = 1250,
            .direction_inverted = false,
            .max_velocity_deg_per_s = 120.0f,
            .max_acceleration_deg_per_s2 = 240.0f,
            .watchdog_timeout_ms = 500,
            .startup_angle_deg = 0.0f,
            .start_disabled = false,
            .failsafe_behavior = ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION
        },
        { // Servo 3
            .id = 3,
            .initial_offset = 0.0f,
            .min_angle = -60.0f,
            .max_angle = 60.0f,
            .pulse_min_us = 500,
            .pulse_max_us = 2000,
            .pulse_neutral_us = 1250,
            .direction_inverted = false,
            .max_velocity_deg_per_s = 120.0f,
            .max_acceleration_deg_per_s2 = 240.0f,
            .watchdog_timeout_ms = 500,
            .startup_angle_deg = 0.0f,
            .start_disabled = false,
            .failsafe_behavior = ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION
        },
        { // Servo 4
            .id = 4,
            .initial_offset = 0.0f,
            .min_angle = -60.0f,
            .max_angle = 60.0f,
            .pulse_min_us = 500,
            .pulse_max_us = 2000,
            .pulse_neutral_us = 1250,
            .direction_inverted = false,
            .max_velocity_deg_per_s = 120.0f,
            .max_acceleration_deg_per_s2 = 240.0f,
            .watchdog_timeout_ms = 500,
            .startup_angle_deg = 0.0f,
            .start_disabled = false,
            .failsafe_behavior = ServoInstanceConfig::FailSafeBehavior::NEUTRAL_POSITION
        }
    }};

    // DC Motor configurations
    constexpr std::array<DCMotorInstanceConfig, 1> DC_MOTOR_CONFIGS = {{
        { // DC Motor 1
            .id = 10,
            .speed_kp = 0.1f,
            .speed_ki = 0.1f,
            .speed_kd = 0.0f,
            .speed_max_integral = 0.3f,
            .speed_max_output = 0.5f,
            .position_kp = 0.1f,
            .position_ki = 0.0f,
            .position_kd = 0.0f,
            .position_max_integral = 100.0f,
            .position_max_output = 10.0f,
            .max_speed_rad_s = 15.0f,
            .max_acceleration_rad_s2 = 50.0f,
            .position_limit_min_rad = -314.159f, // -100 * pi
            .position_limit_max_rad = 314.159f,  // +100 * pi
            .use_position_limits = true,
            .watchdog_timeout_ms = 2000,
            .control_period_ms = 10,
            .direction_inverted = true
        }
    }};

    // Encoder configurations (from JSON data)
    constexpr std::array<EncoderInstanceConfig, 4> ENCODER_CONFIGS = {{
        { // Encoder 1 (for DC Motor)
            .id = 1,
            .cpr = 8192,
            .invert_a = true,
            .invert_b = false,
            .use_z = false,
            .watchdog_timeout_ms = 500,
            .offset_counts = 0,
            .wrap_around = false,
            .mode = EncoderInstanceConfig::Mode::TIM_ENCODER_MODE
        },
        { // Encoder 2
            .id = 2,
            .cpr = 2048,
            .invert_a = false,
            .invert_b = true,
            .use_z = true,
            .watchdog_timeout_ms = 300,
            .offset_counts = 0,
            .wrap_around = true,
            .mode = EncoderInstanceConfig::Mode::TIM_ENCODER_MODE
        },
        { // Encoder 3
            .id = 3,
            .cpr = 4096,
            .invert_a = false,
            .invert_b = false,
            .use_z = false,
            .watchdog_timeout_ms = 200,
            .offset_counts = 0,
            .wrap_around = false,
            .mode = EncoderInstanceConfig::Mode::TIM_ENCODER_MODE
        },
        { // Encoder 4
            .id = 4,
            .cpr = 1024,
            .invert_a = false,
            .invert_b = false,
            .use_z = false,
            .watchdog_timeout_ms = 500,
            .offset_counts = 0,
            .wrap_around = true,
            .mode = EncoderInstanceConfig::Mode::TIM_ENCODER_MODE
        }
    }};

    // RoboMaster motor configurations
    constexpr std::array<RoboMasterInstanceConfig, 1> ROBOMASTER_CONFIGS = {{
        { // RoboMaster 1
            .id = 20,
            .can_id = 5,
            .angle_kp = 0.1f,
            .angle_ki = 0.0f,
            .angle_kd = 0.0f,
            .speed_kp = 0.1f,
            .speed_ki = 0.0f,
            .speed_kd = 0.0f,
            .max_speed_rad_s = 10.0f,
            .max_acceleration_rad_s2 = 30.0f,
            .watchdog_timeout_ms = 1000
        }
    }};

} // namespace Devices

// System-wide robot configuration
namespace Robot {

    // Robot identification
    static constexpr const char* ROBOT_NAME = "MAVLink Test Robot";
    static constexpr const char* ROBOT_VERSION = "2.0.0";
    static constexpr uint32_t CONFIG_VERSION = 1;

    // Communication settings
    static constexpr uint8_t MAVLINK_SYSTEM_ID = 1;
    static constexpr uint8_t MAVLINK_COMPONENT_ID = 1;
    static constexpr uint32_t MAVLINK_HEARTBEAT_INTERVAL_MS = 1000;
    static constexpr uint32_t UART_BAUD_RATE = 115200;

    // System limits
    static constexpr uint8_t MAX_SERVOS = 4;
    static constexpr uint8_t MAX_DC_MOTORS = 2;
    static constexpr uint8_t MAX_ENCODERS = 4;
    static constexpr uint8_t MAX_ROBOMASTER_MOTORS = 2;

    // Safety configuration
    static constexpr uint32_t SYSTEM_WATCHDOG_TIMEOUT_MS = 5000;
    static constexpr uint32_t EMERGENCY_STOP_TIMEOUT_MS = 100;
    static constexpr bool ENABLE_LIMIT_SWITCHES = true;
    static constexpr bool ENABLE_VOLTAGE_MONITORING = true;
    static constexpr bool ENABLE_TEMPERATURE_MONITORING = false;

    // Performance settings
    static constexpr uint32_t MAIN_LOOP_FREQUENCY_HZ = 100;
    static constexpr uint32_t TELEMETRY_RATE_HZ = 10;
    static constexpr uint32_t CONTROL_LOOP_FREQUENCY_HZ = 100;
}

// Convenience accessors for runtime use
namespace ConfigAccessor {

    // Get servo configuration by ID
    constexpr const Devices::ServoInstanceConfig* getServoConfig(uint8_t id) {
        for (const auto& config : Devices::SERVO_CONFIGS) {
            if (config.id == id) {
                return &config;
            }
        }
        return nullptr;
    }

    // Get DC motor configuration by ID
    constexpr const Devices::DCMotorInstanceConfig* getDCMotorConfig(uint8_t id) {
        for (const auto& config : Devices::DC_MOTOR_CONFIGS) {
            if (config.id == id) {
                return &config;
            }
        }
        return nullptr;
    }

    // Get encoder configuration by ID
    constexpr const Devices::EncoderInstanceConfig* getEncoderConfig(uint8_t id) {
        for (const auto& config : Devices::ENCODER_CONFIGS) {
            if (config.id == id) {
                return &config;
            }
        }
        return nullptr;
    }

    // Get RoboMaster motor configuration by ID
    constexpr const Devices::RoboMasterInstanceConfig* getRoboMasterConfig(uint8_t id) {
        for (const auto& config : Devices::ROBOMASTER_CONFIGS) {
            if (config.id == id) {
                return &config;
            }
        }
        return nullptr;
    }
}

} // namespace Config