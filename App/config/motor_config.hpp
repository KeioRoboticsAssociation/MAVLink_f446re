#pragma once

#include <cstdint>
#include <array>

namespace Config {

// Compile-time servo configuration
struct ServoConfig {
    static constexpr float ANGLE_MIN = -60.0f;
    static constexpr float ANGLE_MAX = 60.0f;
    static constexpr uint16_t PULSE_MIN_US = 500;
    static constexpr uint16_t PULSE_MAX_US = 2000;
    static constexpr uint16_t PULSE_NEUTRAL_US = 1250;
    static constexpr bool DIRECTION_INVERTED = false;
    static constexpr float OFFSET_DEG = 0.0f;
    static constexpr float MAX_VELOCITY_DEG_PER_S = 120.0f;
    static constexpr float MAX_ACCELERATION_DEG_PER_S2 = 240.0f;
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
    static constexpr float STARTUP_ANGLE_DEG = 0.0f;
    static constexpr bool START_DISABLED = false;
};

// DC Motor configuration
struct DCMotorConfig {
    static constexpr float SPEED_KP = 0.8f;
    static constexpr float SPEED_KI = 0.0f;
    static constexpr float SPEED_KD = 0.0f;
    static constexpr float SPEED_MAX_INTEGRAL = 10.0f;
    static constexpr float SPEED_MAX_OUTPUT = 1.0f;
    static constexpr float POSITION_KP = 3.0f;
    static constexpr float POSITION_KI = 0.0f;
    static constexpr float POSITION_KD = 0.0f;
    static constexpr float POSITION_MAX_INTEGRAL = 100.0f;
    static constexpr float POSITION_MAX_OUTPUT = 10.0f;
    static constexpr float MAX_SPEED_RAD_S = 15.0f;
    static constexpr float MAX_ACCELERATION_RAD_S2 = 50.0f;
    static constexpr float POSITION_LIMIT_MIN_RAD = -314.159f; // -100 * pi
    static constexpr float POSITION_LIMIT_MAX_RAD = 314.159f;  // +100 * pi
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 2000;
    static constexpr uint32_t CONTROL_PERIOD_MS = 10;
};

// RoboMaster Motor configuration
struct RoboMasterConfig {
    static constexpr float ANGLE_KP = 0.1f;
    static constexpr float ANGLE_KI = 0.0f;
    static constexpr float ANGLE_KD = 0.0f;
    static constexpr float SPEED_KP = 0.1f;
    static constexpr float SPEED_KI = 0.0f;
    static constexpr float SPEED_KD = 0.0f;
    static constexpr float MAX_SPEED_RAD_S = 10.0f;
    static constexpr float MAX_ACCELERATION_RAD_S2 = 30.0f;
    static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 1000;
};

// System-wide configuration
namespace System {
    static constexpr uint8_t MAVLINK_SYSTEM_ID = 1;
    static constexpr uint8_t MAVLINK_COMPONENT_ID = 1;
    static constexpr uint32_t SYSTEM_CLOCK_HZ = 84000000;
    static constexpr uint32_t UART_BAUD_RATE = 115200;
    static constexpr uint8_t MAX_MOTORS = 8;
    static constexpr uint8_t MAX_SERVOS = 4;
}

// Motor instances configuration
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

// Flash-based configuration table
constexpr std::array<MotorInstance, 6> MOTOR_INSTANCES = {{
    {1, MotorInstance::Type::SERVO, 1, 1, true},      // Servo 1 on TIM1 CH1
    {2, MotorInstance::Type::SERVO, 1, 2, true},      // Servo 2 on TIM1 CH2
    {3, MotorInstance::Type::SERVO, 1, 3, true},      // Servo 3 on TIM1 CH3
    {4, MotorInstance::Type::SERVO, 1, 4, true},      // Servo 4 on TIM1 CH4
    {10, MotorInstance::Type::DC_MOTOR, 3, 1, true},  // DC Motor on TIM3 CH1
    {20, MotorInstance::Type::ROBOMASTER, 0, 0, true} // RoboMaster on CAN
}};

} // namespace Config