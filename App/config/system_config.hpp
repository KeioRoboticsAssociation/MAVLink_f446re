#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>

namespace Config {

// Error handling configuration
enum class ErrorCode : uint8_t {
    OK = 0,
    NOT_INITIALIZED = 1,
    HARDWARE_ERROR = 2,
    TIMEOUT = 3,
    OUT_OF_RANGE = 4,
    CONFIG_ERROR = 5,
    COMMUNICATION_ERROR = 6,
    SAFETY_VIOLATION = 7
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