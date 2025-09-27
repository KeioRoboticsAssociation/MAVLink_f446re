#include "system_config.hpp"

namespace Config {

ErrorSeverity ErrorUtils::getSeverity(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK:
            return ErrorSeverity::INFO;

        // System-level errors
        case ErrorCode::NOT_INITIALIZED:
        case ErrorCode::ALREADY_INITIALIZED:
            return ErrorSeverity::ERROR;
        case ErrorCode::INVALID_STATE:
        case ErrorCode::RESOURCE_EXHAUSTED:
            return ErrorSeverity::CRITICAL;

        // Hardware errors
        case ErrorCode::HARDWARE_ERROR:
        case ErrorCode::HARDWARE_NOT_FOUND:
        case ErrorCode::SENSOR_ERROR:
            return ErrorSeverity::ERROR;
        case ErrorCode::HARDWARE_FAULT:
        case ErrorCode::ACTUATOR_ERROR:
            return ErrorSeverity::CRITICAL;

        // Communication errors
        case ErrorCode::TIMEOUT:
        case ErrorCode::PROTOCOL_ERROR:
        case ErrorCode::CHECKSUM_ERROR:
            return ErrorSeverity::WARNING;
        case ErrorCode::COMMUNICATION_ERROR:
        case ErrorCode::BUFFER_OVERFLOW:
            return ErrorSeverity::ERROR;

        // Configuration errors
        case ErrorCode::CONFIG_ERROR:
        case ErrorCode::INVALID_PARAMETER:
        case ErrorCode::OUT_OF_RANGE:
        case ErrorCode::MISSING_CONFIG:
        case ErrorCode::CONFIG_PARSE_ERROR:
            return ErrorSeverity::ERROR;

        // Safety errors - all critical or fatal
        case ErrorCode::SAFETY_VIOLATION:
        case ErrorCode::EMERGENCY_STOP:
        case ErrorCode::OVER_TEMPERATURE:
        case ErrorCode::OVER_CURRENT:
        case ErrorCode::POSITION_LIMIT:
        case ErrorCode::VELOCITY_LIMIT:
            return ErrorSeverity::FATAL;

        // Motor errors
        case ErrorCode::MOTOR_ERROR:
        case ErrorCode::ENCODER_ERROR:
        case ErrorCode::CALIBRATION_ERROR:
            return ErrorSeverity::ERROR;
        case ErrorCode::MOTOR_STALL:
            return ErrorSeverity::CRITICAL;

        // Network/MAVLink errors
        case ErrorCode::MAVLINK_ERROR:
        case ErrorCode::MESSAGE_DROPPED:
        case ErrorCode::INVALID_MESSAGE:
        case ErrorCode::SEQUENCE_ERROR:
            return ErrorSeverity::WARNING;

        // Generic errors
        case ErrorCode::NOT_SUPPORTED:
        case ErrorCode::NOT_IMPLEMENTED:
            return ErrorSeverity::ERROR;
        case ErrorCode::UNKNOWN_ERROR:
        case ErrorCode::INTERNAL_ERROR:
            return ErrorSeverity::CRITICAL;

        default:
            return ErrorSeverity::ERROR;
    }
}

ErrorCategory ErrorUtils::getCategory(ErrorCode code) {
    uint8_t codeValue = static_cast<uint8_t>(code);

    if (codeValue == 0) return ErrorCategory::SYSTEM;
    if (codeValue <= 19) return ErrorCategory::SYSTEM;
    if (codeValue <= 39) return ErrorCategory::HARDWARE;
    if (codeValue <= 59) return ErrorCategory::COMMUNICATION;
    if (codeValue <= 79) return ErrorCategory::CONFIGURATION;
    if (codeValue <= 99) return ErrorCategory::SAFETY;
    if (codeValue <= 119) return ErrorCategory::MOTOR;
    if (codeValue <= 139) return ErrorCategory::NETWORK;
    return ErrorCategory::GENERIC;
}

const char* ErrorUtils::getDescription(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK: return "Success";

        // System-level errors
        case ErrorCode::NOT_INITIALIZED: return "Component not initialized";
        case ErrorCode::ALREADY_INITIALIZED: return "Component already initialized";
        case ErrorCode::INVALID_STATE: return "Invalid system state";
        case ErrorCode::RESOURCE_EXHAUSTED: return "System resources exhausted";

        // Hardware errors
        case ErrorCode::HARDWARE_ERROR: return "Hardware error";
        case ErrorCode::HARDWARE_NOT_FOUND: return "Hardware not found";
        case ErrorCode::HARDWARE_FAULT: return "Hardware fault detected";
        case ErrorCode::SENSOR_ERROR: return "Sensor error";
        case ErrorCode::ACTUATOR_ERROR: return "Actuator error";

        // Communication errors
        case ErrorCode::COMMUNICATION_ERROR: return "Communication error";
        case ErrorCode::TIMEOUT: return "Operation timed out";
        case ErrorCode::PROTOCOL_ERROR: return "Protocol error";
        case ErrorCode::CHECKSUM_ERROR: return "Checksum mismatch";
        case ErrorCode::BUFFER_OVERFLOW: return "Buffer overflow";

        // Configuration errors
        case ErrorCode::CONFIG_ERROR: return "Configuration error";
        case ErrorCode::INVALID_PARAMETER: return "Invalid parameter";
        case ErrorCode::OUT_OF_RANGE: return "Value out of range";
        case ErrorCode::MISSING_CONFIG: return "Missing configuration";
        case ErrorCode::CONFIG_PARSE_ERROR: return "Configuration parse error";

        // Safety errors
        case ErrorCode::SAFETY_VIOLATION: return "Safety violation";
        case ErrorCode::EMERGENCY_STOP: return "Emergency stop activated";
        case ErrorCode::OVER_TEMPERATURE: return "Over temperature";
        case ErrorCode::OVER_CURRENT: return "Over current";
        case ErrorCode::POSITION_LIMIT: return "Position limit exceeded";
        case ErrorCode::VELOCITY_LIMIT: return "Velocity limit exceeded";

        // Motor errors
        case ErrorCode::MOTOR_ERROR: return "Motor error";
        case ErrorCode::MOTOR_STALL: return "Motor stall detected";
        case ErrorCode::ENCODER_ERROR: return "Encoder error";
        case ErrorCode::CALIBRATION_ERROR: return "Calibration error";

        // Network/MAVLink errors
        case ErrorCode::MAVLINK_ERROR: return "MAVLink error";
        case ErrorCode::MESSAGE_DROPPED: return "Message dropped";
        case ErrorCode::INVALID_MESSAGE: return "Invalid message";
        case ErrorCode::SEQUENCE_ERROR: return "Sequence error";

        // Generic errors
        case ErrorCode::UNKNOWN_ERROR: return "Unknown error";
        case ErrorCode::NOT_SUPPORTED: return "Operation not supported";
        case ErrorCode::NOT_IMPLEMENTED: return "Feature not implemented";
        case ErrorCode::INTERNAL_ERROR: return "Internal error";

        default: return "Undefined error";
    }
}

const char* ErrorUtils::getRecoveryHint(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK: return "";

        // System-level errors
        case ErrorCode::NOT_INITIALIZED: return "Initialize the component first";
        case ErrorCode::ALREADY_INITIALIZED: return "Component is already ready";
        case ErrorCode::INVALID_STATE: return "Reset system to valid state";
        case ErrorCode::RESOURCE_EXHAUSTED: return "Free resources or restart system";

        // Hardware errors
        case ErrorCode::HARDWARE_ERROR: return "Check hardware connections";
        case ErrorCode::HARDWARE_NOT_FOUND: return "Verify hardware is connected";
        case ErrorCode::HARDWARE_FAULT: return "Replace or repair hardware";
        case ErrorCode::SENSOR_ERROR: return "Check sensor connections and calibration";
        case ErrorCode::ACTUATOR_ERROR: return "Check actuator power and connections";

        // Communication errors
        case ErrorCode::COMMUNICATION_ERROR: return "Check communication links";
        case ErrorCode::TIMEOUT: return "Retry operation or increase timeout";
        case ErrorCode::PROTOCOL_ERROR: return "Verify protocol configuration";
        case ErrorCode::CHECKSUM_ERROR: return "Check for interference or retry";
        case ErrorCode::BUFFER_OVERFLOW: return "Reduce data rate or increase buffer size";

        // Configuration errors
        case ErrorCode::CONFIG_ERROR: return "Verify configuration settings";
        case ErrorCode::INVALID_PARAMETER: return "Use valid parameter values";
        case ErrorCode::OUT_OF_RANGE: return "Use values within allowed range";
        case ErrorCode::MISSING_CONFIG: return "Provide required configuration";
        case ErrorCode::CONFIG_PARSE_ERROR: return "Check configuration file format";

        // Safety errors
        case ErrorCode::SAFETY_VIOLATION: return "Review safety protocols";
        case ErrorCode::EMERGENCY_STOP: return "Clear emergency condition and reset";
        case ErrorCode::OVER_TEMPERATURE: return "Cool down system before continuing";
        case ErrorCode::OVER_CURRENT: return "Reduce load or check for short circuit";
        case ErrorCode::POSITION_LIMIT: return "Move to safe position";
        case ErrorCode::VELOCITY_LIMIT: return "Reduce velocity command";

        // Motor errors
        case ErrorCode::MOTOR_ERROR: return "Check motor connections and power";
        case ErrorCode::MOTOR_STALL: return "Remove obstruction or reduce load";
        case ErrorCode::ENCODER_ERROR: return "Check encoder connections";
        case ErrorCode::CALIBRATION_ERROR: return "Recalibrate motor";

        // Network/MAVLink errors
        case ErrorCode::MAVLINK_ERROR: return "Check MAVLink configuration";
        case ErrorCode::MESSAGE_DROPPED: return "Reduce message rate";
        case ErrorCode::INVALID_MESSAGE: return "Verify message format";
        case ErrorCode::SEQUENCE_ERROR: return "Reset connection";

        // Generic errors
        case ErrorCode::UNKNOWN_ERROR: return "Contact support";
        case ErrorCode::NOT_SUPPORTED: return "Use alternative method";
        case ErrorCode::NOT_IMPLEMENTED: return "Feature will be available in future version";
        case ErrorCode::INTERNAL_ERROR: return "Restart system";

        default: return "No recovery information available";
    }
}

bool ErrorUtils::isRecoverable(ErrorCode code) {
    switch (getSeverity(code)) {
        case ErrorSeverity::INFO:
        case ErrorSeverity::WARNING:
        case ErrorSeverity::ERROR:
            return true;
        case ErrorSeverity::CRITICAL:
            return true; // May be recoverable with intervention
        case ErrorSeverity::FATAL:
            return false;
        default:
            return false;
    }
}

bool ErrorUtils::requiresEmergencyStop(ErrorCode code) {
    switch (code) {
        case ErrorCode::SAFETY_VIOLATION:
        case ErrorCode::EMERGENCY_STOP:
        case ErrorCode::OVER_TEMPERATURE:
        case ErrorCode::OVER_CURRENT:
        case ErrorCode::MOTOR_STALL:
        case ErrorCode::HARDWARE_FAULT:
        case ErrorCode::ACTUATOR_ERROR:
            return true;
        default:
            return false;
    }
}

} // namespace Config