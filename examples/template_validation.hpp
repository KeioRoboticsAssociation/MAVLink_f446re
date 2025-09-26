#pragma once

// Template Validation Examples
// Demonstrates how to use and validate different robot configuration templates

#include "../App/config/templates/template_generator.hpp"

namespace Examples {

// Example 1: Basic Servo Robot Validation
namespace BasicServoExample {
    #include "../App/config/templates/basic_servo_robot.hpp"

    using namespace Config::Templates;

    // Compile-time validation
    static_assert(validateServoConfigs(BASIC_SERVO_CONFIGS),
                  "Basic servo configuration validation failed");
    static_assert(validateMotorInstances(BASIC_MOTOR_INSTANCES),
                  "Basic motor instance validation failed");

    // Configuration summary
    constexpr size_t SERVO_COUNT = BASIC_SERVO_CONFIGS.size();
    constexpr size_t MOTOR_INSTANCE_COUNT = BASIC_MOTOR_INSTANCES.size();

    static_assert(SERVO_COUNT == 2, "Expected 2 servos in basic configuration");
    static_assert(MOTOR_INSTANCE_COUNT == 2, "Expected 2 motor instances");
}

// Example 2: Precision DC Robot Validation
namespace PrecisionDCExample {
    #include "../App/config/templates/precision_dc_robot.hpp"

    using namespace Config::Templates;

    // Compile-time validation
    static_assert(validateDCMotorConfigs(PRECISION_DC_CONFIGS),
                  "Precision DC motor configuration validation failed");
    static_assert(validateMotorInstances(PRECISION_MOTOR_INSTANCES),
                  "Precision motor instance validation failed");

    // Verify high-precision settings
    constexpr auto& motor1 = PRECISION_DC_CONFIGS[0];
    constexpr auto& motor2 = PRECISION_DC_CONFIGS[1];

    static_assert(motor1.control_period_ms == 5, "Expected 200Hz control for precision");
    static_assert(motor2.control_period_ms == 5, "Expected 200Hz control for precision");

    // Verify encoder resolution
    constexpr auto& encoder1 = PRECISION_ENCODER_CONFIGS[0];
    constexpr auto& encoder2 = PRECISION_ENCODER_CONFIGS[1];

    static_assert(encoder1.cpr == 16384, "Expected high-resolution encoder");
    static_assert(encoder2.cpr == 16384, "Expected high-resolution encoder");
}

// Example 3: Hybrid Robot Validation
namespace HybridExample {
    #include "../App/config/templates/hybrid_robot.hpp"

    using namespace Config::Templates;

    // Validate all subsystems
    static_assert(validateServoConfigs(HYBRID_SERVO_CONFIGS),
                  "Hybrid servo configuration validation failed");
    static_assert(validateDCMotorConfigs(HYBRID_DC_CONFIGS),
                  "Hybrid DC motor configuration validation failed");
    static_assert(validateMotorInstances(HYBRID_MOTOR_INSTANCES),
                  "Hybrid motor instance validation failed");

    // Verify hybrid configuration complexity
    constexpr size_t TOTAL_MOTORS = HYBRID_MOTOR_INSTANCES.size();
    static_assert(TOTAL_MOTORS == 5, "Expected 5 motors in hybrid configuration");

    // Verify different motor types are present
    constexpr bool hasServo = HYBRID_SERVO_CONFIGS.size() > 0;
    constexpr bool hasDCMotor = HYBRID_DC_CONFIGS.size() > 0;
    constexpr bool hasRoboMaster = HYBRID_ROBOMASTER_CONFIGS.size() > 0;

    static_assert(hasServo && hasDCMotor && hasRoboMaster,
                  "Hybrid robot should have all motor types");
}

// Example 4: Custom Robot Using Template Generator
namespace CustomRobotExample {
    // Generate a custom 3-servo robot configuration
    DOCUMENT_ROBOT_CONFIG("Custom 3-Servo Robot",
                         "Example custom robot with 3 servos",
                         "Template Generator");

    // Define custom servo configurations using generator macros
    constexpr std::array<Config::Devices::ServoInstanceConfig, 3> CUSTOM_SERVO_CONFIGS = {{
        GENERATE_SERVO_CONFIG(1, "Base", -180.0f, 180.0f, 1500, 60.0f, 120.0f),
        GENERATE_SERVO_CONFIG(2, "Shoulder", -90.0f, 90.0f, 1500, 90.0f, 180.0f),
        GENERATE_SERVO_CONFIG(3, "Elbow", -135.0f, 45.0f, 1500, 120.0f, 240.0f)
    }};

    // Hardware mapping
    constexpr std::array<Config::MotorInstance, 3> CUSTOM_MOTOR_INSTANCES = {{
        GENERATE_MOTOR_INSTANCE(1, SERVO, 2, 1),  // TIM2 CH1
        GENERATE_MOTOR_INSTANCE(2, SERVO, 2, 2),  // TIM2 CH2
        GENERATE_MOTOR_INSTANCE(3, SERVO, 12, 1)  // TIM12 CH1
    }};

    // Apply robot template
    SIMPLE_SERVO_ROBOT(3);

    // Validation
    static_assert(validateServoConfigs(CUSTOM_SERVO_CONFIGS),
                  "Custom servo configuration validation failed");
    static_assert(validateMotorInstances(CUSTOM_MOTOR_INSTANCES),
                  "Custom motor instance validation failed");

    // Verify robot settings
    static_assert(Robot::MAX_SERVOS == 3, "Expected 3 servos in custom robot");
    static_assert(Robot::MAX_DC_MOTORS == 0, "Expected no DC motors");
    static_assert(Robot::MAX_ROBOMASTER_MOTORS == 0, "Expected no RoboMaster motors");
}

// Example 5: Configuration Comparison
namespace ConfigurationComparison {

    // Compare control frequencies across templates
    constexpr uint32_t basic_freq = Config::Templates::BasicServoRobot::MAIN_LOOP_FREQUENCY_HZ;
    constexpr uint32_t precision_freq = Config::Templates::PrecisionDCRobot::MAIN_LOOP_FREQUENCY_HZ;
    constexpr uint32_t hybrid_freq = Config::Templates::HybridRobot::MAIN_LOOP_FREQUENCY_HZ;

    static_assert(basic_freq < precision_freq, "Precision robot should have higher frequency");
    static_assert(precision_freq > hybrid_freq, "Precision robot optimized for high frequency");

    // Compare complexity metrics
    constexpr size_t basic_motors = Config::Templates::BASIC_MOTOR_INSTANCES.size();
    constexpr size_t precision_motors = Config::Templates::PRECISION_MOTOR_INSTANCES.size();
    constexpr size_t hybrid_motors = Config::Templates::HYBRID_MOTOR_INSTANCES.size();

    static_assert(hybrid_motors > precision_motors, "Hybrid robot is most complex");
    static_assert(hybrid_motors > basic_motors, "Hybrid robot is most complex");

    // Performance characteristics summary
    struct PerformanceMetrics {
        uint32_t loop_freq_hz;
        size_t motor_count;
        bool has_encoders;
        bool has_can_motors;
        uint32_t telemetry_rate_hz;
    };

    constexpr PerformanceMetrics BASIC_METRICS = {
        .loop_freq_hz = basic_freq,
        .motor_count = basic_motors,
        .has_encoders = false,
        .has_can_motors = false,
        .telemetry_rate_hz = 10
    };

    constexpr PerformanceMetrics PRECISION_METRICS = {
        .loop_freq_hz = precision_freq,
        .motor_count = precision_motors,
        .has_encoders = true,
        .has_can_motors = false,
        .telemetry_rate_hz = 20
    };

    constexpr PerformanceMetrics HYBRID_METRICS = {
        .loop_freq_hz = hybrid_freq,
        .motor_count = hybrid_motors,
        .has_encoders = true,
        .has_can_motors = true,
        .telemetry_rate_hz = 15
    };
}

// Example 6: Template Selection Guide
namespace TemplateSelectionGuide {

    // Helper to recommend template based on requirements
    template<bool needsPrecision, bool needsMobility, bool needsSimplicity>
    struct TemplateRecommendation {
        static constexpr const char* recommendation =
            needsSimplicity ? "Use basic_servo_robot.hpp for simple applications" :
            needsPrecision ? "Use precision_dc_robot.hpp for high-precision control" :
            needsMobility ? "Use hybrid_robot.hpp for mobile robots" :
            "Consider creating custom configuration";
    };

    // Usage examples:
    static constexpr auto simple_camera_mount =
        TemplateRecommendation<false, false, true>::recommendation;

    static constexpr auto cnc_machine =
        TemplateRecommendation<true, false, false>::recommendation;

    static constexpr auto mobile_manipulator =
        TemplateRecommendation<false, true, false>::recommendation;
}

} // namespace Examples

// Global validation summary
namespace ValidationSummary {
    constexpr bool ALL_TEMPLATES_VALID =
        Examples::BasicServoExample::SERVO_COUNT == 2 &&
        Examples::PrecisionDCExample::PRECISION_DC_CONFIGS.size() == 2 &&
        Examples::HybridExample::TOTAL_MOTORS == 5 &&
        Examples::CustomRobotExample::CUSTOM_SERVO_CONFIGS.size() == 3;

    static_assert(ALL_TEMPLATES_VALID, "Template validation failed");
}