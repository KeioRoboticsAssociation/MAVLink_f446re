/**
 * @file timeout_manager.cpp
 * @brief Implements the manager for handling system-wide timeouts.
 */

#include "timeout_manager.hpp"

namespace Config {

/**
 * @brief Construct a new TimeoutManager object.
 */
TimeoutManager::TimeoutManager() {
    for (size_t i = 0; i < timeouts_.size(); ++i) {
        TimeoutType type = static_cast<TimeoutType>(i);
        timeouts_[i].type = type;
        timeouts_[i].timeoutMs = getDefaultTimeout(type);
        timeouts_[i].active = false;
        timeouts_[i].recurring = false;
        timeouts_[i].callback = nullptr;
    }
}

/**
 * @brief Configures a timeout.
 * @param type The type of timeout to configure.
 * @param timeoutMs The timeout duration in milliseconds.
 * @param recurring Whether the timeout should recur.
 * @param callback The callback function to execute on timeout.
 */
void TimeoutManager::configure(TimeoutType type, uint32_t timeoutMs, bool recurring, TimeoutCallback callback) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    timeout.timeoutMs = timeoutMs;
    timeout.recurring = recurring;
    timeout.callback = callback;
}

/**
 * @brief Starts a timeout.
 * @param type The type of timeout to start.
 */
void TimeoutManager::start(TimeoutType type) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    if (!timeout.active) {
        timeout.start();
        activeTimeouts_++;
    }
}

/**
 * @brief Stops a timeout.
 * @param type The type of timeout to stop.
 */
void TimeoutManager::stop(TimeoutType type) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    if (timeout.active) {
        timeout.stop();
        activeTimeouts_--;
    }
}

/**
 * @brief Restarts a timeout.
 * @param type The type of timeout to restart.
 */
void TimeoutManager::restart(TimeoutType type) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    timeout.start();
    if (!timeout.active) {
        activeTimeouts_++;
    }
}

/**
 * @brief Stops all active timeouts.
 */
void TimeoutManager::stopAll() {
    for (auto& timeout : timeouts_) {
        timeout.stop();
    }
    activeTimeouts_ = 0;
}

/**
 * @brief Checks if a timeout is active.
 * @param type The type of timeout to check.
 * @return true if the timeout is active, false otherwise.
 */
bool TimeoutManager::isActive(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return false;
    return timeouts_[index].active;
}

/**
 * @brief Checks if a timeout has expired.
 * @param type The type of timeout to check.
 * @return true if the timeout has expired, false otherwise.
 */
bool TimeoutManager::hasExpired(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return false;
    return timeouts_[index].hasExpired();
}

/**
 * @brief Gets the elapsed time of a timeout.
 * @param type The type of timeout to check.
 * @return The elapsed time in milliseconds.
 */
uint32_t TimeoutManager::getElapsed(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return 0;
    return timeouts_[index].getElapsed();
}

/**
 * @brief Gets the remaining time of a timeout.
 * @param type The type of timeout to check.
 * @return The remaining time in milliseconds.
 */
uint32_t TimeoutManager::getRemaining(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return 0;
    return timeouts_[index].getRemaining();
}

/**
 * @brief Updates the timeout manager, checking for expired timeouts.
 */
void TimeoutManager::update() {
    uint32_t currentTime = HAL_GetTick();

    for (auto& timeout : timeouts_) {
        if (timeout.active && timeout.hasExpired(currentTime)) {
            handleExpiredTimeout(timeout);
        }
    }

    updateStatistics();
}

/**
 * @brief Gets the default timeout duration for a given timeout type.
 * @param type The type of timeout.
 * @return The default timeout duration in milliseconds.
 */
uint32_t TimeoutManager::getDefaultTimeout(TimeoutType type) {
    switch (type) {
        case TimeoutType::MAVLINK_HEARTBEAT: return 5000;
        case TimeoutType::UART_TRANSMISSION: return 100;
        case TimeoutType::MESSAGE_RESPONSE: return 1000;
        case TimeoutType::MOTOR_WATCHDOG: return 500;
        case TimeoutType::MOTOR_MOVEMENT: return 5000;
        case TimeoutType::MOTOR_CALIBRATION: return 10000;
        case TimeoutType::EMERGENCY_STOP: return 100;
        case TimeoutType::SAFETY_CHECK: return 1000;
        case TimeoutType::INITIALIZATION: return 30000;
        case TimeoutType::SYSTEM_UPDATE: return 50;
        default: return 1000;
    }
}

/**
 * @brief Gets the name of a timeout type.
 * @param type The type of timeout.
 * @return The name of the timeout type.
 */
const char* TimeoutManager::getTimeoutName(TimeoutType type) {
    switch (type) {
        case TimeoutType::MAVLINK_HEARTBEAT: return "MAVLink Heartbeat";
        case TimeoutType::UART_TRANSMISSION: return "UART Transmission";
        case TimeoutType::MESSAGE_RESPONSE: return "Message Response";
        case TimeoutType::MOTOR_WATCHDOG: return "Motor Watchdog";
        case TimeoutType::MOTOR_MOVEMENT: return "Motor Movement";
        case TimeoutType::MOTOR_CALIBRATION: return "Motor Calibration";
        case TimeoutType::EMERGENCY_STOP: return "Emergency Stop";
        case TimeoutType::SAFETY_CHECK: return "Safety Check";
        case TimeoutType::INITIALIZATION: return "System Initialization";
        case TimeoutType::SYSTEM_UPDATE: return "System Update";
        default: return "Unknown";
    }
}

/**
 * @brief Gets the singleton instance of the TimeoutManager.
 * @return The singleton instance.
 */
TimeoutManager& TimeoutManager::getInstance() {
    static TimeoutManager instance;
    return instance;
}

void TimeoutManager::handleExpiredTimeout(TimeoutTracker& timeout) {
    totalTimeouts_++;

    if (timeout.callback) {
        timeout.callback(timeout.type, timeout.getElapsed());
    }

    if (timeout.recurring) {
        timeout.start();
    } else {
        timeout.stop();
        activeTimeouts_--;
    }
}

void TimeoutManager::updateStatistics() {
    uint32_t actualActive = 0;
    for (const auto& timeout : timeouts_) {
        if (timeout.active) {
            actualActive++;
        }
    }
    activeTimeouts_ = actualActive;
}

TimeoutManager& timeoutManager = TimeoutManager::getInstance();

} // namespace Config