/**
 * @file motor_config.hpp
 * @brief Defines the configuration for motors and system-wide parameters.
 */

#pragma once

#include <cstdint>
#include <array>
#include "robot_config.hpp"

namespace Config {

/**
 * @brief Defines a motor instance, mapping a device ID to hardware resources.
 */
struct MotorInstance {
    uint8_t id;
    enum class Type : uint8_t {
        SERVO = 0,
        DC_MOTOR = 1,
        ROBOMASTER = 2
    } type;
    uint8_t timer_id;
    uint32_t channel;
    bool enabled;
};

/**
 * @brief Configuration of all motor instances in the system.
 */
constexpr std::array<MotorInstance, 6> MOTOR_INSTANCES = {{
    {1, MotorInstance::Type::SERVO, Hardware::SERVO_TIMERS[0].timer_id, Hardware::SERVO_TIMERS[0].channel, true},
    {2, MotorInstance::Type::SERVO, Hardware::SERVO_TIMERS[1].timer_id, Hardware::SERVO_TIMERS[1].channel, true},
    {3, MotorInstance::Type::SERVO, Hardware::SERVO_TIMERS[2].timer_id, Hardware::SERVO_TIMERS[2].channel, true},
    {4, MotorInstance::Type::SERVO, Hardware::SERVO_TIMERS[3].timer_id, Hardware::SERVO_TIMERS[3].channel, true},
    {10, MotorInstance::Type::DC_MOTOR, Hardware::DC_MOTOR_TIMERS[0].timer_id, Hardware::DC_MOTOR_TIMERS[0].channel, true},
    {20, MotorInstance::Type::ROBOMASTER, 0, 0, true}
}};

/**
 * @brief Legacy compatibility structure for servo configuration.
 */
struct ServoConfig {
    static constexpr float ANGLE_MIN = Devices::SERVO_CONFIGS[0].min_angle;
    static constexpr float ANGLE_MAX = Devices::SERVO_CONFIGS[0].max_angle;
    static constexpr uint16_t PULSE_MIN_US = Devices::SERVO_CONFIGS[0].pulse_min_us;
    static constexpr uint16_t PULSE_MAX_US = Devices::SERVO_CONFIGS[0].pulse_max_us;
    static constexpr uint16_t PULSE_NEUTRAL_US = Devices::SERVO_CONFIGS[0].pulse_neutral_us;
    static constexpr bool DIRECTION_INVERTED = Devices::SERVO_CONFIGS[0].direction_inverted;
    static constexpr float OFFSET_DEG = Devices::SERVO_CONFIGS[0].initial_offset;
    static constexpr float MAX_VELOCITY_DEG_PER_S = Devices::SERVO_CONFIGS[0].max_velocity_deg_per_s;
    static constexpr float MAX_ACCELERATION_DEG_PER_S2 = Devices::SERVO_CONFIGS[0].max_acceleration_deg_per_s2;
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = Devices::SERVO_CONFIGS[0].watchdog_timeout_ms;
    static constexpr float STARTUP_ANGLE_DEG = Devices::SERVO_CONFIGS[0].startup_angle_deg;
    static constexpr bool START_DISABLED = Devices::SERVO_CONFIGS[0].start_disabled;
};

/**
 * @brief Legacy compatibility structure for DC motor configuration.
 */
struct DCMotorConfig {
    static constexpr float SPEED_KP = Devices::DC_MOTOR_CONFIGS[0].speed_kp;
    static constexpr float SPEED_KI = Devices::DC_MOTOR_CONFIGS[0].speed_ki;
    static constexpr float SPEED_KD = Devices::DC_MOTOR_CONFIGS[0].speed_kd;
    static constexpr float SPEED_MAX_INTEGRAL = Devices::DC_MOTOR_CONFIGS[0].speed_max_integral;
    static constexpr float SPEED_MAX_OUTPUT = Devices::DC_MOTOR_CONFIGS[0].speed_max_output;
    static constexpr float POSITION_KP = Devices::DC_MOTOR_CONFIGS[0].position_kp;
    static constexpr float POSITION_KI = Devices::DC_MOTOR_CONFIGS[0].position_ki;
    static constexpr float POSITION_KD = Devices::DC_MOTOR_CONFIGS[0].position_kd;
    static constexpr float POSITION_MAX_INTEGRAL = Devices::DC_MOTOR_CONFIGS[0].position_max_integral;
    static constexpr float POSITION_MAX_OUTPUT = Devices::DC_MOTOR_CONFIGS[0].position_max_output;
    static constexpr float MAX_SPEED_RAD_S = Devices::DC_MOTOR_CONFIGS[0].max_speed_rad_s;
    static constexpr float MAX_ACCELERATION_RAD_S2 = Devices::DC_MOTOR_CONFIGS[0].max_acceleration_rad_s2;
    static constexpr float POSITION_LIMIT_MIN_RAD = Devices::DC_MOTOR_CONFIGS[0].position_limit_min_rad;
    static constexpr float POSITION_LIMIT_MAX_RAD = Devices::DC_MOTOR_CONFIGS[0].position_limit_max_rad;
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = Devices::DC_MOTOR_CONFIGS[0].watchdog_timeout_ms;
    static constexpr uint32_t CONTROL_PERIOD_MS = Devices::DC_MOTOR_CONFIGS[0].control_period_ms;
};

/**
 * @brief Legacy compatibility structure for RoboMaster motor configuration.
 */
struct RoboMasterConfig {
    static constexpr float ANGLE_KP = Devices::ROBOMASTER_CONFIGS[0].angle_kp;
    static constexpr float ANGLE_KI = Devices::ROBOMASTER_CONFIGS[0].angle_ki;
    static constexpr float ANGLE_KD = Devices::ROBOMASTER_CONFIGS[0].angle_kd;
    static constexpr float SPEED_KP = Devices::ROBOMASTER_CONFIGS[0].speed_kp;
    static constexpr float SPEED_KI = Devices::ROBOMASTER_CONFIGS[0].speed_ki;
    static constexpr float SPEED_KD = Devices::ROBOMASTER_CONFIGS[0].speed_kd;
    static constexpr float MAX_SPEED_RAD_S = Devices::ROBOMASTER_CONFIGS[0].max_speed_rad_s;
    static constexpr float MAX_ACCELERATION_RAD_S2 = Devices::ROBOMASTER_CONFIGS[0].max_acceleration_rad_s2;
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = Devices::ROBOMASTER_CONFIGS[0].watchdog_timeout_ms;
};

/**
 * @brief System-wide configuration parameters.
 */
namespace System {
    static constexpr uint8_t MAVLINK_SYSTEM_ID = Robot::MAVLINK_SYSTEM_ID;
    static constexpr uint8_t MAVLINK_COMPONENT_ID = Robot::MAVLINK_COMPONENT_ID;
    static constexpr uint32_t SYSTEM_CLOCK_HZ = 84000000;
    static constexpr uint32_t UART_BAUD_RATE = Robot::UART_BAUD_RATE;
    static constexpr uint8_t MAX_MOTORS = Robot::MAX_SERVOS + Robot::MAX_DC_MOTORS + Robot::MAX_ROBOMASTER_MOTORS;
    static constexpr uint8_t MAX_SERVOS = Robot::MAX_SERVOS;
}

} // namespace Config