/**
 * @file system_context.hpp
 * @brief Defines the main system context, safety manager, and application class.
 */

#pragma once

#include "hal/hardware_manager.hpp"
#include "config/motor_config.hpp"
#include "config/system_config.hpp"
#include <memory>
#include <vector>
#include <functional>

// Forward declarations
namespace Motors {
    class MotorRegistry;
    class IMotorFactory;
}

namespace Communication {
    class UnifiedMAVLinkHandler;
}

namespace System {

class SafetyManager;
class Application;

/**
 * @brief Manages the overall state and dependencies of the system.
 *
 * This class acts as a central point for accessing all major subsystems,
 * ensuring that they are initialized in the correct order and can interact
 * with each other.
 */
class SystemContext {
public:
    /**
     * @brief Manages the hardware abstraction layer.
     */
    struct Hardware {
        std::unique_ptr<HAL::HardwareManager> manager;

        Config::Result<void> initialize();
        HAL::HardwareManager* get() const;
    } hardware;

    /**
     * @brief Manages all motor-related subsystems.
     */
    struct Motors {
        std::unique_ptr<class ::Motors::MotorRegistry> registry_;
        std::unique_ptr<class ::Motors::IMotorFactory> factory_;

        Config::Result<Config::ErrorCode> initialize(HAL::HardwareManager* hwManager);
        Config::Result<Config::ErrorCode> createAllMotors();
        ::Motors::MotorRegistry* getRegistry() const;
        ::Motors::IMotorFactory* getFactory() const;
    } motors;

    /**
     * @brief Manages all communication protocols.
     */
    struct Communication {
        std::unique_ptr<class ::Communication::UnifiedMAVLinkHandler> handler_;

        Config::Result<Config::ErrorCode> initialize(HAL::HardwareManager* hwManager, ::Motors::MotorRegistry* motorRegistry);
        ::Communication::UnifiedMAVLinkHandler* get() const;
    } communication;

    /**
     * @brief Manages system safety features.
     */
    struct Safety {
        std::unique_ptr<SafetyManager> manager;

        Config::Result<Config::ErrorCode> initialize(HAL::HardwareManager* hwManager);
        SafetyManager* get() const;
    } safety;

    /**
     * @brief Holds the overall state of the system.
     */
    struct SystemState {
        bool initialized = false;
        bool emergencyStop = false;
        uint32_t startTime = 0;
        uint32_t lastUpdate = 0;
        uint32_t errorCount = 0;
        Config::ErrorCode lastError = Config::ErrorCode::OK;
    } state;

public:
    Config::Result<Config::ErrorCode> initialize();
    Config::Result<Config::ErrorCode> update();
    void shutdown();

    HAL::HardwareManager* getHardware() const;
    ::Motors::MotorRegistry* getMotors() const;
    ::Communication::UnifiedMAVLinkHandler* getCommunication() const;
    SafetyManager* getSafety() const;

    bool isInitialized() const;
    bool isEmergencyStop() const;
    uint32_t getUptime() const;
    Config::ErrorCode getLastError() const;

    void setEmergencyStop(bool emergency);
    void reportError(Config::ErrorCode error, const char* context = nullptr);

private:
    void logSystemState();
};

/**
 * @brief Manages system safety by monitoring limits and handling emergencies.
 */
class SafetyManager {
private:
    HAL::HardwareManager* hwManager_;
    SystemContext* systemContext_;

    enum class SafetyState {
        NORMAL,
        WARNING,
        FAULT,
        EMERGENCY_STOP
    } currentState_ = SafetyState::NORMAL;

    struct SafetyLimits {
        float maxTemperature = Config::SafetyLimits::MAX_TEMPERATURE_C;
        float maxCurrent = Config::SafetyLimits::MAX_CURRENT_A;
        float maxVoltage = Config::SafetyLimits::MAX_VOLTAGE_V;
        uint32_t heartbeatTimeout = Config::SafetyLimits::HEARTBEAT_TIMEOUT_MS;
    } limits_;

    uint32_t lastHeartbeat_ = 0;
    uint32_t lastSafetyCheck_ = 0;
    uint32_t emergencyStopTime_ = 0;

    std::function<void(SafetyState, SafetyState)> stateChangeCallback_;
    std::function<void(const char*)> emergencyCallback_;

public:
    SafetyManager(HAL::HardwareManager* hwManager, SystemContext* systemContext);

    Config::Result<Config::ErrorCode> initialize();
    Config::Result<void> update();
    Config::Result<void> checkAllLimits();
    Config::Result<Config::ErrorCode> registerLimitSwitch(GPIO_TypeDef* port, uint16_t pin,
                                                          std::function<void()> callback);
    SafetyState getCurrentState() const;
    void transitionToSafeState();
    void triggerEmergencyStop(const char* reason);
    void clearEmergencyStop();
    void updateHeartbeat();
    bool isHeartbeatValid() const;
    void setLimits(const SafetyLimits& limits);
    SafetyLimits getLimits() const;
    void setStateChangeCallback(std::function<void(SafetyState, SafetyState)> callback);
    void setEmergencyCallback(std::function<void(const char*)> callback);

private:
    void setState(SafetyState newState);
    Config::Result<Config::ErrorCode> checkTemperatures();
    Config::Result<Config::ErrorCode> checkCurrents();
    Config::Result<Config::ErrorCode> checkVoltages();
    Config::Result<Config::ErrorCode> checkHeartbeat();
};

/**
 * @brief Orchestrates the main application logic.
 */
class Application {
private:
    SystemContext& context_;
    bool running_ = false;
    uint32_t updateInterval_ = 10;

public:
    explicit Application(SystemContext& context);

    Config::Result<Config::ErrorCode> initialize();
    Config::Result<Config::ErrorCode> run();
    void stop();
    void setUpdateInterval(uint32_t intervalMs);
    uint32_t getUpdateInterval() const;

private:
    Config::Result<Config::ErrorCode> mainLoop();
    Config::Result<Config::ErrorCode> updateSubsystems();
    void handleErrors();
};

} // namespace System