#include "system_context.hpp"
#include "config/motor_config.hpp"

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

    // Skip motor updates if in emergency stop
    if (!state.emergencyStop) {
        // TODO: Update motors
        // motors.getRegistry()->updateAll(deltaTime);
        (void)deltaTime; // Suppress unused warning
    }

    // TODO: Update communication
    // auto commResult = communication.get()->update();
    // if (!commResult) {
    //     reportError(commResult.error(), "Communication update failed");
    //     // Don't return error for communication failures - continue running
    // }

    return Config::ErrorCode::OK;
}

void SystemContext::shutdown() {
    if (state.initialized) {
        // TODO: Stop all motors
        // motors.getRegistry()->emergencyStopAll();

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
        // Entering emergency stop
        // TODO: motors.getRegistry()->emergencyStopAll();
        safety.get()->triggerEmergencyStop("External trigger");
    } else if (!emergency && state.emergencyStop) {
        // Clearing emergency stop
        safety.get()->clearEmergencyStop();
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

// Motor subsystem implementation
Config::Result<Config::ErrorCode> SystemContext::Motors::initialize(HAL::HardwareManager* hwManager) {
    // TODO: Implement MotorRegistry
    // registry = std::make_unique<Motors::MotorRegistry>();

    // Create concrete motor factory (implementation would be in separate file)
    // factory = std::make_unique<ConcreteMotorFactory>(hwManager);

    (void)hwManager; // Suppress unused parameter warning
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> SystemContext::Motors::createAllMotors() {
    // Create motors based on compile-time configuration
    for (const auto& motorConfig : Config::MOTOR_INSTANCES) {
        if (!motorConfig.enabled) {
            continue;
        }

        // Create motor based on type
        switch (motorConfig.type) {
            case Config::MotorInstance::Type::SERVO: {
                // auto servo = factory->createServo(motorConfig.id);
                // if (servo) {
                //     registry->registerMotor(motorConfig.id, std::move(servo));
                // }
                break;
            }
            case Config::MotorInstance::Type::DC_MOTOR: {
                // auto dcMotor = factory->createDCMotor(motorConfig.id);
                // if (dcMotor) {
                //     registry->registerMotor(motorConfig.id, std::move(dcMotor));
                // }
                break;
            }
            case Config::MotorInstance::Type::ROBOMASTER: {
                // auto roboMaster = factory->createRoboMaster(motorConfig.id);
                // if (roboMaster) {
                //     registry->registerMotor(motorConfig.id, std::move(roboMaster));
                // }
                break;
            }
        }
    }

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

    // Setup safety monitoring callbacks
    setStateChangeCallback([this](SafetyState /*oldState*/, SafetyState newState) {
        // Handle state transitions
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