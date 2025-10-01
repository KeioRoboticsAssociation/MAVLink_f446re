/**
 * @file system_config.hpp
 * @brief Defines system-wide configuration, error handling, and utility types.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>

namespace Config {

/**
 * @brief Defines all possible error codes in the system.
 */
enum class ErrorCode : uint8_t {
    OK = 0,
    NOT_INITIALIZED = 1,
    ALREADY_INITIALIZED = 2,
    INVALID_STATE = 3,
    RESOURCE_EXHAUSTED = 4,
    HARDWARE_ERROR = 20,
    HARDWARE_NOT_FOUND = 21,
    HARDWARE_FAULT = 22,
    SENSOR_ERROR = 23,
    ACTUATOR_ERROR = 24,
    COMMUNICATION_ERROR = 40,
    TIMEOUT = 41,
    PROTOCOL_ERROR = 42,
    CHECKSUM_ERROR = 43,
    BUFFER_OVERFLOW = 44,
    CONFIG_ERROR = 60,
    INVALID_PARAMETER = 61,
    OUT_OF_RANGE = 62,
    MISSING_CONFIG = 63,
    CONFIG_PARSE_ERROR = 64,
    SAFETY_VIOLATION = 80,
    EMERGENCY_STOP = 81,
    OVER_TEMPERATURE = 82,
    OVER_CURRENT = 83,
    POSITION_LIMIT = 84,
    VELOCITY_LIMIT = 85,
    MOTOR_ERROR = 100,
    MOTOR_STALL = 101,
    ENCODER_ERROR = 102,
    CALIBRATION_ERROR = 103,
    MAVLINK_ERROR = 120,
    MESSAGE_DROPPED = 121,
    INVALID_MESSAGE = 122,
    SEQUENCE_ERROR = 123,
    UNKNOWN_ERROR = 200,
    NOT_SUPPORTED = 201,
    NOT_IMPLEMENTED = 202,
    INTERNAL_ERROR = 203
};

/**
 * @brief Defines the severity levels for errors.
 */
enum class ErrorSeverity : uint8_t {
    INFO = 0,
    WARNING = 1,
    ERROR = 2,
    CRITICAL = 3,
    FATAL = 4
};

/**
 * @brief Defines categories for grouping related errors.
 */
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

/**
 * @brief Contains detailed information about an error.
 */
struct ErrorInfo {
    ErrorCode code;
    ErrorSeverity severity;
    ErrorCategory category;
    const char* description;
    const char* recovery_hint;
};

/**
 * @brief Provides utility functions for working with error codes.
 */
class ErrorUtils {
public:
    static ErrorSeverity getSeverity(ErrorCode code);
    static ErrorCategory getCategory(ErrorCode code);
    static const char* getDescription(ErrorCode code);
    static const char* getRecoveryHint(ErrorCode code);
    static bool isRecoverable(ErrorCode code);
    static bool requiresEmergencyStop(ErrorCode code);
};

/**
 * @brief A template class for handling results of operations that can fail.
 * @tparam T The type of the value returned on success.
 */
template<typename T>
class Result {
private:
    bool hasValue_;
    union {
        T value_;
        ErrorCode error_;
    };

public:
    Result(T&& val);
    Result(const T& val);
    explicit Result(ErrorCode err);
    ~Result();

    Result(const Result& other);
    Result(Result&& other);

    bool isOk() const;
    bool isError() const;
    T& get();
    const T& get() const;
    ErrorCode error() const;

    explicit operator bool() const;
    T& operator*();
    const T& operator*() const;

    template<typename F>
    auto map(F&& func) -> Result<decltype(func(value_))>;
    template<typename F>
    auto flatMap(F&& func) -> decltype(func(value_))>;

    T getOr(const T& defaultValue) const;

    ErrorSeverity getSeverity() const;
    ErrorCategory getCategory() const;
    const char* getDescription() const;
    bool isRecoverable() const;
};

/**
 * @brief A factory for creating Result instances.
 */
struct ResultFactory {
    template<typename T>
    static Result<T> success(T&& value);

    template<typename T>
    static Result<T> success(const T& value);

    template<typename T>
    static Result<T> error(ErrorCode err);
};

/**
 * @brief Specialization of the Result class for void return types.
 */
template<>
class Result<void> {
private:
    bool hasValue_;
    ErrorCode error_;

public:
    Result();
    Result(ErrorCode err);

    bool isOk() const;
    bool isError() const;
    ErrorCode error() const;
    explicit operator bool() const;
};

/**
 * @brief Defines system-wide safety limits.
 */
struct SafetyLimits {
    static constexpr float MAX_TEMPERATURE_C = 85.0f;
    static constexpr float MAX_CURRENT_A = 5.0f;
    static constexpr float MAX_VOLTAGE_V = 24.0f;
    static constexpr uint32_t EMERGENCY_STOP_TIMEOUT_MS = 100;
    static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 5000;
};

/**
 * @brief Configuration for communication protocols.
 */
namespace Communication {
    static constexpr uint32_t MAVLINK_HEARTBEAT_INTERVAL_MS = 1000;
    static constexpr uint32_t TELEMETRY_RATE_HZ = 10;
    static constexpr uint16_t UART_BUFFER_SIZE = 256;
    static constexpr uint8_t MAX_PENDING_COMMANDS = 16;
}

/**
 * @brief Configuration for memory management.
 */
namespace Memory {
    static constexpr size_t MOTOR_POOL_SIZE = 8;
    static constexpr size_t MESSAGE_POOL_SIZE = 32;
    static constexpr size_t RING_BUFFER_SIZE = 512;
}

/**
 * @brief Configuration for debugging features.
 */
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