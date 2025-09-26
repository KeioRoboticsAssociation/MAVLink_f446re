#include "system_context.hpp"
#include "config/motor_config.hpp"
#include "motors/base/motor_factory.hpp"
#include "comm/unified_mavlink_handler.hpp"

// External C dependencies from STM32 HAL
extern "C" {
    #include "main.h"

    // HAL handles from main.c
    extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
    extern UART_HandleTypeDef huart2;
    extern CAN_HandleTypeDef hcan1;
}

// Global system context
static System::SystemContext g_systemContext;

extern "C" {

// C-compatible entry points called from main.c
void cpp_setup() {
    // Initialize the system context
    auto result = g_systemContext.initialize();
    if (!result) {
        // Handle initialization failure
        g_systemContext.reportError(result.error(), "System initialization failed");

        // Fall back to emergency mode
        g_systemContext.setEmergencyStop(true);
        return;
    }

    // System successfully initialized
}

void cpp_loop() {
    // Update all subsystems
    auto result = g_systemContext.update();
    if (!result) {
        // Handle update failure
        g_systemContext.reportError(result.error(), "System update failed");

        // Continue running but log the error
        return;
    }

    // Optional: Add small delay to prevent overwhelming the system
    // HAL_Delay(1); // 1ms delay for 1000Hz loop
}

void cpp_emergency_stop() {
    // Emergency stop handler
    g_systemContext.setEmergencyStop(true);
}

void cpp_shutdown() {
    // Graceful shutdown
    g_systemContext.shutdown();
}

} // extern "C"

// C++ implementation

namespace System {

Config::Result<Config::ErrorCode> SystemContext::initialize() {
    if (state.initialized) {
        return Config::ErrorCode::OK;
    }

    state.startTime = HAL_GetTick();

    // Initialize hardware subsystem
    auto hwResult = hardware.initialize();
    if (!hwResult) {
        reportError(hwResult.error(), "Hardware initialization failed");
        return hwResult.error();
    }

    // Initialize motor subsystem
    auto motorResult = motors.initialize(hardware.get());
    if (!motorResult) {
        reportError(motorResult.error(), "Motor initialization failed");
        return motorResult.error();
    }

    // Initialize communication subsystem
    auto commResult = communication.initialize(hardware.get());
    if (!commResult) {
        reportError(commResult.error(), "Communication initialization failed");
        return commResult.error();
    }

    // Initialize safety subsystem
    auto safetyResult = safety.initialize(hardware.get());
    if (!safetyResult) {
        reportError(safetyResult.error(), "Safety initialization failed");
        return safetyResult.error();
    }

    // Create all configured motors
    auto createResult = motors.createAllMotors();
    if (!createResult) {
        reportError(createResult.error(), "Motor creation failed");
        return createResult.error();
    }

    state.initialized = true;
    state.lastUpdate = HAL_GetTick();

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> SystemContext::update() {
    if (!state.initialized) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    uint32_t currentTime = HAL_GetTick();
    float deltaTime = (currentTime - state.lastUpdate) / 1000.0f; // Convert to seconds
    state.lastUpdate = currentTime;

    // Update safety first
    auto safetyResult = safety.get()->update();
    if (!safetyResult) {
        reportError(safetyResult.error(), "Safety update failed");
        return safetyResult.error();
    }

    // Skip motor updates if in emergency stop (Phase 2 - placeholder)
    if (!state.emergencyStop) {
        // Motor updates will be implemented in Phase 2-3
        // For now, continue without motor updates
        (void)deltaTime; // Suppress unused warning
    }

    // Update communication (Phase 2 - placeholder)
    // Communication subsystem will be implemented in Phase 2-3
    // For now, continue without communication updates

    return Config::ErrorCode::OK;
}

void SystemContext::shutdown() {
    if (state.initialized) {
        // Stop all motors (Phase 2 - placeholder)
        // Motor emergency stop will be implemented in Phase 2-3

        // Reset state
        state.initialized = false;
        state.emergencyStop = false;
    }
}

uint32_t SystemContext::getUptime() const {
    if (!state.initialized) {
        return 0;
    }
    return HAL_GetTick() - state.startTime;
}

void SystemContext::setEmergencyStop(bool emergency) {
    if (emergency && !state.emergencyStop) {
        // Entering emergency stop (Phase 2 - placeholder)
        // Motor emergency stop will be implemented in Phase 2-3
        if (safety.get()) {
            safety.get()->triggerEmergencyStop("External trigger");
        }
    } else if (!emergency && state.emergencyStop) {
        // Clearing emergency stop
        if (safety.get()) {
            safety.get()->clearEmergencyStop();
        }
    }

    state.emergencyStop = emergency;
}

void SystemContext::reportError(Config::ErrorCode error, const char* context) {
    state.lastError = error;
    state.errorCount++;

    // Log error if context provided (in debug builds)
    if constexpr (Config::Debug::LOGGING_ENABLED) {
        if (context) {
            // Would log to debug output
            (void)context; // Suppress unused warning in release
        }
    }
}

// Motor subsystem implementation (Phase 1 stub)
Config::Result<Config::ErrorCode> SystemContext::Motors::initialize(HAL::HardwareManager* /*hwManager*/) {
    // Phase 1: Stub implementations to complete architecture cleanup
    // Motor registry and factory will be fully implemented in Phase 2-3

    // For now, return success to allow the system to initialize
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> SystemContext::Motors::createAllMotors() {
    // Phase 1: Stub implementation to complete architecture cleanup
    // Motor creation will be implemented in Phase 2-3 when motor interfaces are complete

    // For now, just return success to allow system initialization
    return Config::ErrorCode::OK;
}

// Safety subsystem implementation
Config::Result<Config::ErrorCode> SystemContext::Safety::initialize(HAL::HardwareManager* hwManager) {
    manager = std::make_unique<SafetyManager>(hwManager, nullptr); // Would pass system context
    return manager->initialize();
}

Config::Result<Config::ErrorCode> SafetyManager::initialize() {
    lastSafetyCheck_ = hwManager_->getSystemTick();
    lastHeartbeat_ = lastSafetyCheck_;

    // Configure safety limits from robot configuration
    limits_.heartbeatTimeout = Config::Robot::SYSTEM_WATCHDOG_TIMEOUT_MS;
    limits_.maxTemperature = Config::SafetyLimits::MAX_TEMPERATURE_C;
    limits_.maxCurrent = Config::SafetyLimits::MAX_CURRENT_A;
    limits_.maxVoltage = Config::SafetyLimits::MAX_VOLTAGE_V;

    // Setup safety monitoring callbacks
    setStateChangeCallback([this](SafetyState /*oldState*/, SafetyState newState) {
        // Handle state transitions based on robot configuration
        if (newState == SafetyState::EMERGENCY_STOP) {
            systemContext_->setEmergencyStop(true);
        }
    });

    return Config::ErrorCode::OK;
}

Config::Result<void> SafetyManager::update() {
    uint32_t currentTime = hwManager_->getSystemTick();

    // Run safety checks every 100ms
    if (currentTime - lastSafetyCheck_ >= 100) {
        auto result = checkAllLimits();
        lastSafetyCheck_ = currentTime;

        if (!result) {
            return Config::Result<void>(result.error());
        }
    }

    return Config::Result<void>();
}

Config::Result<void> SafetyManager::checkAllLimits() {
    // Check heartbeat timeout
    if (!isHeartbeatValid()) {
        triggerEmergencyStop("Heartbeat timeout");
        return Config::Result<void>(Config::ErrorCode::TIMEOUT);
    }

    // Additional safety checks would go here
    // - Temperature monitoring
    // - Current monitoring
    // - Limit switch states

    return Config::Result<void>();
}

bool SafetyManager::isHeartbeatValid() const {
    uint32_t currentTime = hwManager_->getSystemTick();
    return (currentTime - lastHeartbeat_) < limits_.heartbeatTimeout;
}

void SafetyManager::triggerEmergencyStop(const char* reason) {
    setState(SafetyState::EMERGENCY_STOP);
    emergencyStopTime_ = hwManager_->getSystemTick();

    if (emergencyCallback_) {
        emergencyCallback_(reason);
    }
}

void SafetyManager::clearEmergencyStop() {
    if (currentState_ == SafetyState::EMERGENCY_STOP) {
        setState(SafetyState::NORMAL);
        emergencyStopTime_ = 0;
    }
}

void SafetyManager::setState(SafetyState newState) {
    SafetyState oldState = currentState_;
    currentState_ = newState;

    if (stateChangeCallback_ && oldState != newState) {
        stateChangeCallback_(oldState, newState);
    }
}

} // namespace System