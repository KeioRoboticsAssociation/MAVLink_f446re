#pragma once

#include "system_config.hpp"
#include <cstdint>
#include <functional>
#include <array>

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Config {

// Timeout types for different system operations
enum class TimeoutType : uint8_t {
    // Communication timeouts
    MAVLINK_HEARTBEAT = 0,
    UART_TRANSMISSION = 1,
    MESSAGE_RESPONSE = 2,

    // Motor timeouts
    MOTOR_WATCHDOG = 3,
    MOTOR_MOVEMENT = 4,
    MOTOR_CALIBRATION = 5,

    // Safety timeouts
    EMERGENCY_STOP = 6,
    SAFETY_CHECK = 7,

    // System timeouts
    INITIALIZATION = 8,
    SYSTEM_UPDATE = 9,

    MAX_TIMEOUT_TYPES = 10
};

// Timeout callback type
using TimeoutCallback = std::function<void(TimeoutType, uint32_t elapsedMs)>;

// Individual timeout tracker
struct TimeoutTracker {
    TimeoutType type;
    uint32_t timeoutMs;
    uint32_t startTime;
    bool active;
    bool recurring;
    TimeoutCallback callback;

    TimeoutTracker() : type(TimeoutType::SYSTEM_UPDATE), timeoutMs(0), startTime(0),
                      active(false), recurring(false), callback(nullptr) {}

    void start(uint32_t currentTime = HAL_GetTick()) {
        startTime = currentTime;
        active = true;
    }

    void stop() {
        active = false;
    }

    bool hasExpired(uint32_t currentTime = HAL_GetTick()) const {
        if (!active) return false;
        return (currentTime - startTime) >= timeoutMs;
    }

    uint32_t getElapsed(uint32_t currentTime = HAL_GetTick()) const {
        return active ? (currentTime - startTime) : 0;
    }

    uint32_t getRemaining(uint32_t currentTime = HAL_GetTick()) const {
        if (!active) return 0;
        uint32_t elapsed = getElapsed(currentTime);
        return (elapsed >= timeoutMs) ? 0 : (timeoutMs - elapsed);
    }
};

// Unified timeout manager
class TimeoutManager {
private:
    std::array<TimeoutTracker, static_cast<size_t>(TimeoutType::MAX_TIMEOUT_TYPES)> timeouts_;

    // Statistics
    uint32_t totalTimeouts_ = 0;
    uint32_t activeTimeouts_ = 0;

public:
    TimeoutManager();

    // Configuration
    void configure(TimeoutType type, uint32_t timeoutMs, bool recurring = false,
                  TimeoutCallback callback = nullptr);

    // Control
    void start(TimeoutType type);
    void stop(TimeoutType type);
    void restart(TimeoutType type);
    void stopAll();

    // Status
    bool isActive(TimeoutType type) const;
    bool hasExpired(TimeoutType type) const;
    uint32_t getElapsed(TimeoutType type) const;
    uint32_t getRemaining(TimeoutType type) const;

    // Update - call this regularly from main loop
    void update();

    // Statistics
    uint32_t getActiveCount() const { return activeTimeouts_; }
    uint32_t getTotalTimeouts() const { return totalTimeouts_; }

    // Default timeout values
    static uint32_t getDefaultTimeout(TimeoutType type);
    static const char* getTimeoutName(TimeoutType type);

    // Convenience factory methods
    static TimeoutManager& getInstance();

private:
    void handleExpiredTimeout(TimeoutTracker& timeout);
    void updateStatistics();
};

// Timeout guard RAII helper
class TimeoutGuard {
private:
    TimeoutManager& manager_;
    TimeoutType type_;
    bool started_;

public:
    TimeoutGuard(TimeoutManager& manager, TimeoutType type, uint32_t timeoutMs = 0)
        : manager_(manager), type_(type), started_(false) {
        if (timeoutMs > 0) {
            manager_.configure(type_, timeoutMs);
        }
        start();
    }

    ~TimeoutGuard() {
        if (started_) {
            manager_.stop(type_);
        }
    }

    void start() {
        if (!started_) {
            manager_.start(type_);
            started_ = true;
        }
    }

    void stop() {
        if (started_) {
            manager_.stop(type_);
            started_ = false;
        }
    }

    bool hasExpired() const {
        return started_ && manager_.hasExpired(type_);
    }

    uint32_t getRemaining() const {
        return started_ ? manager_.getRemaining(type_) : 0;
    }

    // Non-copyable, movable
    TimeoutGuard(const TimeoutGuard&) = delete;
    TimeoutGuard& operator=(const TimeoutGuard&) = delete;
    TimeoutGuard(TimeoutGuard&& other) noexcept
        : manager_(other.manager_), type_(other.type_), started_(other.started_) {
        other.started_ = false;
    }
    TimeoutGuard& operator=(TimeoutGuard&& other) noexcept {
        if (this != &other) {
            if (started_) manager_.stop(type_);
            manager_ = other.manager_;
            type_ = other.type_;
            started_ = other.started_;
            other.started_ = false;
        }
        return *this;
    }
};

// Global timeout manager instance
extern TimeoutManager& timeoutManager;

} // namespace Config