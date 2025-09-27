#include "timeout_manager.hpp"

namespace Config {

TimeoutManager::TimeoutManager() {
    // Initialize with default configurations
    for (size_t i = 0; i < timeouts_.size(); ++i) {
        TimeoutType type = static_cast<TimeoutType>(i);
        timeouts_[i].type = type;
        timeouts_[i].timeoutMs = getDefaultTimeout(type);
        timeouts_[i].active = false;
        timeouts_[i].recurring = false;
        timeouts_[i].callback = nullptr;
    }
}

void TimeoutManager::configure(TimeoutType type, uint32_t timeoutMs, bool recurring, TimeoutCallback callback) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    timeout.timeoutMs = timeoutMs;
    timeout.recurring = recurring;
    timeout.callback = callback;
}

void TimeoutManager::start(TimeoutType type) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    if (!timeout.active) {
        timeout.start();
        activeTimeouts_++;
    }
}

void TimeoutManager::stop(TimeoutType type) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    if (timeout.active) {
        timeout.stop();
        activeTimeouts_--;
    }
}

void TimeoutManager::restart(TimeoutType type) {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return;

    auto& timeout = timeouts_[index];
    timeout.start();
    if (!timeout.active) {
        activeTimeouts_++;
    }
}

void TimeoutManager::stopAll() {
    for (auto& timeout : timeouts_) {
        timeout.stop();
    }
    activeTimeouts_ = 0;
}

bool TimeoutManager::isActive(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return false;
    return timeouts_[index].active;
}

bool TimeoutManager::hasExpired(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return false;
    return timeouts_[index].hasExpired();
}

uint32_t TimeoutManager::getElapsed(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return 0;
    return timeouts_[index].getElapsed();
}

uint32_t TimeoutManager::getRemaining(TimeoutType type) const {
    size_t index = static_cast<size_t>(type);
    if (index >= timeouts_.size()) return 0;
    return timeouts_[index].getRemaining();
}

void TimeoutManager::update() {
    uint32_t currentTime = HAL_GetTick();

    for (auto& timeout : timeouts_) {
        if (timeout.active && timeout.hasExpired(currentTime)) {
            handleExpiredTimeout(timeout);
        }
    }

    updateStatistics();
}

uint32_t TimeoutManager::getDefaultTimeout(TimeoutType type) {
    switch (type) {
        // Communication timeouts
        case TimeoutType::MAVLINK_HEARTBEAT: return 5000;   // 5 seconds
        case TimeoutType::UART_TRANSMISSION: return 100;    // 100ms
        case TimeoutType::MESSAGE_RESPONSE: return 1000;    // 1 second

        // Motor timeouts
        case TimeoutType::MOTOR_WATCHDOG: return 500;       // 500ms
        case TimeoutType::MOTOR_MOVEMENT: return 5000;      // 5 seconds
        case TimeoutType::MOTOR_CALIBRATION: return 10000;  // 10 seconds

        // Safety timeouts
        case TimeoutType::EMERGENCY_STOP: return 100;       // 100ms
        case TimeoutType::SAFETY_CHECK: return 1000;        // 1 second

        // System timeouts
        case TimeoutType::INITIALIZATION: return 30000;     // 30 seconds
        case TimeoutType::SYSTEM_UPDATE: return 50;         // 50ms

        default: return 1000; // 1 second default
    }
}

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

TimeoutManager& TimeoutManager::getInstance() {
    static TimeoutManager instance;
    return instance;
}

void TimeoutManager::handleExpiredTimeout(TimeoutTracker& timeout) {
    totalTimeouts_++;

    // Call callback if provided
    if (timeout.callback) {
        timeout.callback(timeout.type, timeout.getElapsed());
    }

    // Handle recurring timeouts
    if (timeout.recurring) {
        timeout.start(); // Restart for recurring timeouts
    } else {
        timeout.stop(); // Stop non-recurring timeouts
        activeTimeouts_--;
    }
}

void TimeoutManager::updateStatistics() {
    // Update active count (in case it got out of sync)
    uint32_t actualActive = 0;
    for (const auto& timeout : timeouts_) {
        if (timeout.active) {
            actualActive++;
        }
    }
    activeTimeouts_ = actualActive;
}

// Global instance
TimeoutManager& timeoutManager = TimeoutManager::getInstance();

} // namespace Config