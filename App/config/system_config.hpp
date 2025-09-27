#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>

namespace Config {

// Comprehensive error handling configuration
enum class ErrorCode : uint8_t {
    // Success
    OK = 0,

    // System-level errors (1-19)
    NOT_INITIALIZED = 1,
    ALREADY_INITIALIZED = 2,
    INVALID_STATE = 3,
    RESOURCE_EXHAUSTED = 4,

    // Hardware errors (20-39)
    HARDWARE_ERROR = 20,
    HARDWARE_NOT_FOUND = 21,
    HARDWARE_FAULT = 22,
    SENSOR_ERROR = 23,
    ACTUATOR_ERROR = 24,

    // Communication errors (40-59)
    COMMUNICATION_ERROR = 40,
    TIMEOUT = 41,
    PROTOCOL_ERROR = 42,
    CHECKSUM_ERROR = 43,
    BUFFER_OVERFLOW = 44,

    // Configuration errors (60-79)
    CONFIG_ERROR = 60,
    INVALID_PARAMETER = 61,
    OUT_OF_RANGE = 62,
    MISSING_CONFIG = 63,
    CONFIG_PARSE_ERROR = 64,

    // Safety errors (80-99)
    SAFETY_VIOLATION = 80,
    EMERGENCY_STOP = 81,
    OVER_TEMPERATURE = 82,
    OVER_CURRENT = 83,
    POSITION_LIMIT = 84,
    VELOCITY_LIMIT = 85,

    // Motor-specific errors (100-119)
    MOTOR_ERROR = 100,
    MOTOR_STALL = 101,
    ENCODER_ERROR = 102,
    CALIBRATION_ERROR = 103,

    // Network/MAVLink errors (120-139)
    MAVLINK_ERROR = 120,
    MESSAGE_DROPPED = 121,
    INVALID_MESSAGE = 122,
    SEQUENCE_ERROR = 123,

    // Generic errors (200+)
    UNKNOWN_ERROR = 200,
    NOT_SUPPORTED = 201,
    NOT_IMPLEMENTED = 202,
    INTERNAL_ERROR = 203
};

// Error severity levels for logging and handling
enum class ErrorSeverity : uint8_t {
    INFO = 0,     // Informational - no action needed
    WARNING = 1,  // Warning - system can continue but attention needed
    ERROR = 2,    // Error - functionality affected, recovery possible
    CRITICAL = 3, // Critical - system integrity threatened
    FATAL = 4     // Fatal - system must stop immediately
};

// Error category for grouping related errors
enum class ErrorCategory : uint8_t {
    SYSTEM = 0,
    HARDWARE = 1,
    COMMUNICATION = 2,
    CONFIGURATION = 3,
    SAFETY = 4,
    MOTOR = 5,
    NETWORK = 6,
    GENERIC = 7
};

// Error information structure
struct ErrorInfo {
    ErrorCode code;
    ErrorSeverity severity;
    ErrorCategory category;
    const char* description;
    const char* recovery_hint;
};

// Error code utilities
class ErrorUtils {
public:
    static ErrorSeverity getSeverity(ErrorCode code);
    static ErrorCategory getCategory(ErrorCode code);
    static const char* getDescription(ErrorCode code);
    static const char* getRecoveryHint(ErrorCode code);
    static bool isRecoverable(ErrorCode code);
    static bool requiresEmergencyStop(ErrorCode code);
};

// Result type for error handling
template<typename T>
class Result {
private:
    bool hasValue_;
    union {
        T value_;
        ErrorCode error_;
    };

public:
    Result(T&& val) : hasValue_(true), value_(std::move(val)) {}
    Result(const T& val) : hasValue_(true), value_(val) {}
    explicit Result(ErrorCode err) : hasValue_(false), error_(err) {}

    ~Result() {
        if (hasValue_) {
            value_.~T();
        }
    }

    // Copy constructor
    Result(const Result& other) : hasValue_(other.hasValue_) {
        if (hasValue_) {
            new (&value_) T(other.value_);
        } else {
            error_ = other.error_;
        }
    }

    // Move constructor
    Result(Result&& other) : hasValue_(other.hasValue_) {
        if (hasValue_) {
            new (&value_) T(std::move(other.value_));
        } else {
            error_ = other.error_;
        }
    }

    bool isOk() const { return hasValue_; }
    bool isError() const { return !hasValue_; }

    T& get() { return value_; }
    const T& get() const { return value_; }

    ErrorCode error() const { return error_; }

    // Convenience operators
    explicit operator bool() const { return isOk(); }
    T& operator*() { return get(); }
    const T& operator*() const { return get(); }

    // Functional programming style operations
    template<typename F>
    auto map(F&& func) -> Result<decltype(func(value_))> {
        if (hasValue_) {
            return Result<decltype(func(value_))>(func(value_));
        } else {
            return Result<decltype(func(value_))>(error_);
        }
    }

    template<typename F>
    auto flatMap(F&& func) -> decltype(func(value_)) {
        if (hasValue_) {
            return func(value_);
        } else {
            return decltype(func(value_))(error_);
        }
    }

    // Get value with default
    T getOr(const T& defaultValue) const {
        return hasValue_ ? value_ : defaultValue;
    }

    // Error context support
    ErrorSeverity getSeverity() const { return ErrorUtils::getSeverity(error_); }
    ErrorCategory getCategory() const { return ErrorUtils::getCategory(error_); }
    const char* getDescription() const { return ErrorUtils::getDescription(error_); }
    bool isRecoverable() const { return ErrorUtils::isRecoverable(error_); }
};

// Helper factory functions to create Result instances
struct ResultFactory {
    template<typename T>
    static Result<T> success(T&& value) {
        return Result<T>(std::forward<T>(value));
    }

    template<typename T>
    static Result<T> success(const T& value) {
        return Result<T>(value);
    }

    template<typename T>
    static Result<T> error(ErrorCode err) {
        return Result<T>(err);
    }
};

// Specialization for void type
template<>
class Result<void> {
private:
    bool hasValue_;
    ErrorCode error_;

public:
    Result() : hasValue_(true), error_(ErrorCode::OK) {}
    Result(ErrorCode err) : hasValue_(false), error_(err) {}

    bool isOk() const { return hasValue_; }
    bool isError() const { return !hasValue_; }

    ErrorCode error() const { return error_; }

    // Convenience operators
    explicit operator bool() const { return isOk(); }
};

// Safety limits
struct SafetyLimits {
    static constexpr float MAX_TEMPERATURE_C = 85.0f;
    static constexpr float MAX_CURRENT_A = 5.0f;
    static constexpr float MAX_VOLTAGE_V = 24.0f;
    static constexpr uint32_t EMERGENCY_STOP_TIMEOUT_MS = 100;
    static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 5000;
};

// Communication configuration
namespace Communication {
    static constexpr uint32_t MAVLINK_HEARTBEAT_INTERVAL_MS = 1000;
    static constexpr uint32_t TELEMETRY_RATE_HZ = 10;
    static constexpr uint16_t UART_BUFFER_SIZE = 256;
    static constexpr uint8_t MAX_PENDING_COMMANDS = 16;
}

// Memory management
namespace Memory {
    static constexpr size_t MOTOR_POOL_SIZE = 8;
    static constexpr size_t MESSAGE_POOL_SIZE = 32;
    static constexpr size_t RING_BUFFER_SIZE = 512;
}

// Debug configuration
namespace Debug {
#ifdef DEBUG
    static constexpr bool LOGGING_ENABLED = true;
    static constexpr bool ASSERTIONS_ENABLED = true;
#else
    static constexpr bool LOGGING_ENABLED = false;
    static constexpr bool ASSERTIONS_ENABLED = false;
#endif
}

} // namespace Config