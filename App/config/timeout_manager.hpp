/**
 * @file timeout_manager.hpp
 * @brief Defines a manager for handling system-wide timeouts.
 */

#pragma once

#include "system_config.hpp"
#include <cstdint>
#include <functional>
#include <array>

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Config {

/**
 * @brief Defines the types of timeouts used in the system.
 */
enum class TimeoutType : uint8_t {
    MAVLINK_HEARTBEAT = 0,
    UART_TRANSMISSION = 1,
    MESSAGE_RESPONSE = 2,
    MOTOR_WATCHDOG = 3,
    MOTOR_MOVEMENT = 4,
    MOTOR_CALIBRATION = 5,
    EMERGENCY_STOP = 6,
    SAFETY_CHECK = 7,
    INITIALIZATION = 8,
    SYSTEM_UPDATE = 9,
    MAX_TIMEOUT_TYPES = 10
};

using TimeoutCallback = std::function<void(TimeoutType, uint32_t elapsedMs)>;

/**
 * @brief Tracks an individual timeout.
 */
struct TimeoutTracker {
    TimeoutType type;
    uint32_t timeoutMs;
    uint32_t startTime;
    bool active;
    bool recurring;
    TimeoutCallback callback;

    TimeoutTracker();

    void start(uint32_t currentTime = HAL_GetTick());
    void stop();
    bool hasExpired(uint32_t currentTime = HAL_GetTick()) const;
    uint32_t getElapsed(uint32_t currentTime = HAL_GetTick()) const;
    uint32_t getRemaining(uint32_t currentTime = HAL_GetTick()) const;
};

/**
 * @brief Manages all system timeouts.
 */
class TimeoutManager {
private:
    std::array<TimeoutTracker, static_cast<size_t>(TimeoutType::MAX_TIMEOUT_TYPES)> timeouts_;
    uint32_t totalTimeouts_ = 0;
    uint32_t activeTimeouts_ = 0;

public:
    TimeoutManager();

    void configure(TimeoutType type, uint32_t timeoutMs, bool recurring = false,
                  TimeoutCallback callback = nullptr);
    void start(TimeoutType type);
    void stop(TimeoutType type);
    void restart(TimeoutType type);
    void stopAll();

    bool isActive(TimeoutType type) const;
    bool hasExpired(TimeoutType type) const;
    uint32_t getElapsed(TimeoutType type) const;
    uint32_t getRemaining(TimeoutType type) const;

    void update();

    uint32_t getActiveCount() const { return activeTimeouts_; }
    uint32_t getTotalTimeouts() const { return totalTimeouts_; }

    static uint32_t getDefaultTimeout(TimeoutType type);
    static const char* getTimeoutName(TimeoutType type);
    static TimeoutManager& getInstance();

private:
    void handleExpiredTimeout(TimeoutTracker& timeout);
    void updateStatistics();
};

/**
 * @brief An RAII guard for managing timeouts.
 */
class TimeoutGuard {
private:
    TimeoutManager& manager_;
    TimeoutType type_;
    bool started_;

public:
    TimeoutGuard(TimeoutManager& manager, TimeoutType type, uint32_t timeoutMs = 0);
    ~TimeoutGuard();

    void start();
    void stop();
    bool hasExpired() const;
    uint32_t getRemaining() const;

    TimeoutGuard(const TimeoutGuard&) = delete;
    TimeoutGuard& operator=(const TimeoutGuard&) = delete;
    TimeoutGuard(TimeoutGuard&& other) noexcept;
    TimeoutGuard& operator=(TimeoutGuard&& other) noexcept;
};

extern TimeoutManager& timeoutManager;

} // namespace Config