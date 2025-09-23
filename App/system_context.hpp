#pragma once

#include "hal/hardware_manager.hpp"
#include "motors/base/motor_interface.hpp"
#include "comm/unified_mavlink_handler.hpp"
#include "config/motor_config.hpp"
#include "config/system_config.hpp"
#include <memory>
#include <vector>

namespace System {

// Forward declarations
class SafetyManager;
class Application;

// System context for dependency injection
class SystemContext {
public:
    // Hardware subsystem
    struct Hardware {
        std::unique_ptr<HAL::HardwareManager> manager;

        Config::Result<Config::ErrorCode> initialize() {
            manager = std::make_unique<HAL::HardwareManager>();
            return manager->initialize();
        }

        HAL::HardwareManager* get() const { return manager.get(); }
    } hardware;

    // Motor subsystem
    struct Motors {
        std::unique_ptr<Motors::MotorRegistry> registry;
        std::unique_ptr<Motors::IMotorFactory> factory;

        Config::Result<Config::ErrorCode> initialize(HAL::HardwareManager* hwManager);

        Motors::MotorRegistry* getRegistry() const { return registry.get(); }
        Motors::IMotorFactory* getFactory() const { return factory.get(); }

        Config::Result<Config::ErrorCode> createAllMotors();
    } motors;

    // Communication subsystem
    struct Communication {
        std::unique_ptr<Communication::UnifiedMAVLinkHandler> mavlink;

        Config::Result<Config::ErrorCode> initialize(HAL::HardwareManager* hwManager) {
            mavlink = std::make_unique<Communication::UnifiedMAVLinkHandler>(hwManager);
            return mavlink->initialize();
        }

        Communication::UnifiedMAVLinkHandler* get() const { return mavlink.get(); }
    } communication;

    // Safety subsystem
    struct Safety {
        std::unique_ptr<SafetyManager> manager;

        Config::Result<Config::ErrorCode> initialize(HAL::HardwareManager* hwManager);

        SafetyManager* get() const { return manager.get(); }
    } safety;

    // System-wide state
    struct SystemState {
        bool initialized = false;
        bool emergencyStop = false;
        uint32_t startTime = 0;
        uint32_t lastUpdate = 0;
        uint32_t errorCount = 0;
        Config::ErrorCode lastError = Config::ErrorCode::OK;
    } state;

public:
    // System lifecycle
    Config::Result<Config::ErrorCode> initialize();
    Config::Result<Config::ErrorCode> update();
    void shutdown();

    // Subsystem access
    HAL::HardwareManager* getHardware() const { return hardware.get(); }
    Motors::MotorRegistry* getMotors() const { return motors.getRegistry(); }
    Communication::UnifiedMAVLinkHandler* getCommunication() const { return communication.get(); }
    SafetyManager* getSafety() const { return safety.get(); }

    // State access
    bool isInitialized() const { return state.initialized; }
    bool isEmergencyStop() const { return state.emergencyStop; }
    uint32_t getUptime() const;
    Config::ErrorCode getLastError() const { return state.lastError; }

    // Error handling
    void setEmergencyStop(bool emergency);
    void reportError(Config::ErrorCode error, const char* context = nullptr);

private:
    void logSystemState();
};

// Safety manager implementation
class SafetyManager {
private:
    HAL::HardwareManager* hwManager_;
    SystemContext* systemContext_;

    // Safety state
    enum class SafetyState {
        NORMAL,
        WARNING,
        FAULT,
        EMERGENCY_STOP
    } currentState_ = SafetyState::NORMAL;

    // Limits and thresholds
    struct SafetyLimits {
        float maxTemperature = Config::SafetyLimits::MAX_TEMPERATURE_C;
        float maxCurrent = Config::SafetyLimits::MAX_CURRENT_A;
        float maxVoltage = Config::SafetyLimits::MAX_VOLTAGE_V;
        uint32_t heartbeatTimeout = Config::SafetyLimits::HEARTBEAT_TIMEOUT_MS;
    } limits_;

    // Monitoring state
    uint32_t lastHeartbeat_ = 0;
    uint32_t lastSafetyCheck_ = 0;
    uint32_t emergencyStopTime_ = 0;

    // Callbacks
    std::function<void(SafetyState, SafetyState)> stateChangeCallback_;
    std::function<void(const char*)> emergencyCallback_;

public:
    SafetyManager(HAL::HardwareManager* hwManager, SystemContext* systemContext)
        : hwManager_(hwManager), systemContext_(systemContext) {}

    // Initialization
    Config::Result<Config::ErrorCode> initialize();

    // Safety monitoring
    Config::Result<Config::ErrorCode> update();
    Config::Result<Config::ErrorCode> checkAllLimits();

    // Limit switch management
    Config::Result<Config::ErrorCode> registerLimitSwitch(GPIO_TypeDef* port, uint16_t pin,
                                                          std::function<void()> callback);

    // Safety state management
    SafetyState getCurrentState() const { return currentState_; }
    void transitionToSafeState();
    void triggerEmergencyStop(const char* reason);
    void clearEmergencyStop();

    // Heartbeat monitoring
    void updateHeartbeat() { lastHeartbeat_ = hwManager_->getSystemTick(); }
    bool isHeartbeatValid() const;

    // Configuration
    void setLimits(const SafetyLimits& limits) { limits_ = limits; }
    SafetyLimits getLimits() const { return limits_; }

    // Callbacks
    void setStateChangeCallback(std::function<void(SafetyState, SafetyState)> callback) {
        stateChangeCallback_ = callback;
    }
    void setEmergencyCallback(std::function<void(const char*)> callback) {
        emergencyCallback_ = callback;
    }

private:
    void setState(SafetyState newState);
    Config::Result<Config::ErrorCode> checkTemperatures();
    Config::Result<Config::ErrorCode> checkCurrents();
    Config::Result<Config::ErrorCode> checkVoltages();
    Config::Result<Config::ErrorCode> checkHeartbeat();
};

// Application class that orchestrates everything
class Application {
private:
    SystemContext& context_;
    bool running_ = false;
    uint32_t updateInterval_ = 10; // 10ms = 100Hz

public:
    explicit Application(SystemContext& context) : context_(context) {}

    // Application lifecycle
    Config::Result<Config::ErrorCode> initialize();
    Config::Result<Config::ErrorCode> run();
    void stop() { running_ = false; }

    // Configuration
    void setUpdateInterval(uint32_t intervalMs) { updateInterval_ = intervalMs; }
    uint32_t getUpdateInterval() const { return updateInterval_; }

private:
    Config::Result<Config::ErrorCode> mainLoop();
    Config::Result<Config::ErrorCode> updateSubsystems();
    void handleErrors();
};

} // namespace System