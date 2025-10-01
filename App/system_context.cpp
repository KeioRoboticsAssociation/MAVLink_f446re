/**
 * @file system_context.cpp
 * @brief Implements the main system context, safety manager, and application class.
 */

#include "system_context.hpp"
#include "motors/base/motor_factory.hpp"
#include "comm/unified_mavlink_handler.hpp"
#include "config/robot_config.hpp"

extern "C" {
#include "main.h"
}

namespace System {

/**
 * @brief Initializes the motor subsystem.
 * @param hwManager A pointer to the hardware manager.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> SystemContext::Motors::initialize(HAL::HardwareManager* hwManager) {
    if (!hwManager) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    factory_ = std::make_unique<::Motors::ConcreteMotorFactory>(hwManager);
    if (!factory_) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    registry_ = std::make_unique<::Motors::MotorRegistry>();
    if (!registry_) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    return Config::ErrorCode::OK;
}

/**
 * @brief Creates all configured motors.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> SystemContext::Motors::createAllMotors() {
    if (!factory_ || !registry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    for (const auto& motorInstance : Config::MOTOR_INSTANCES) {
        switch (motorInstance.type) {
            case Config::MotorInstance::Type::SERVO: {
                auto servoController = factory_->createServo(motorInstance.id);
                if (!servoController) return Config::ErrorCode::CONFIG_ERROR;
                Config::ServoConfig defaultServoConfig{};
                auto initResult = servoController->initialize(defaultServoConfig);
                if (!initResult) return Config::ErrorCode::HARDWARE_ERROR;
                auto regResult = registry_->registerMotor(motorInstance.id, std::move(servoController));
                if (!regResult) return regResult.error();
                break;
            }
            case Config::MotorInstance::Type::DC_MOTOR: {
                auto dcController = factory_->createDCMotor(motorInstance.id);
                if (!dcController) return Config::ErrorCode::CONFIG_ERROR;
                Config::DCMotorConfig defaultDCConfig{};
                auto initResult = dcController->initialize(defaultDCConfig);
                if (!initResult) return Config::ErrorCode::HARDWARE_ERROR;
                auto regResult = registry_->registerMotor(motorInstance.id, std::move(dcController));
                if (!regResult) return regResult.error();
                break;
            }
            case Config::MotorInstance::Type::ROBOMASTER: {
                auto roboController = factory_->createRoboMaster(motorInstance.id);
                if (!roboController) return Config::ErrorCode::CONFIG_ERROR;
                Config::RoboMasterConfig defaultRoboConfig{};
                auto initResult = roboController->initialize(defaultRoboConfig);
                if (!initResult) return Config::ErrorCode::HARDWARE_ERROR;
                auto regResult = registry_->registerMotor(motorInstance.id, std::move(roboController));
                if (!regResult) return regResult.error();
                break;
            }
            default:
                return Config::ErrorCode::CONFIG_ERROR;
        }
    }
    return Config::ErrorCode::OK;
}

/**
 * @brief Gets the motor registry.
 * @return A pointer to the motor registry.
 */
::Motors::MotorRegistry* SystemContext::Motors::getRegistry() const {
    return registry_.get();
}

/**
 * @brief Gets the motor factory.
 * @return A pointer to the motor factory.
 */
::Motors::IMotorFactory* SystemContext::Motors::getFactory() const {
    return factory_.get();
}

/**
 * @brief Initializes the communication subsystem.
 * @param hwManager A pointer to the hardware manager.
 * @param motorRegistry A pointer to the motor registry.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> SystemContext::Communication::initialize(HAL::HardwareManager* hwManager, ::Motors::MotorRegistry* motorRegistry) {
    if (!hwManager || !motorRegistry) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }
    handler_ = std::make_unique<::Communication::UnifiedMAVLinkHandler>(
        hwManager, motorRegistry, Config::System::MAVLINK_SYSTEM_ID, Config::System::MAVLINK_COMPONENT_ID
    );
    if (!handler_) return Config::ErrorCode::OUT_OF_RANGE;
    auto initResult = handler_->initialize();
    if (!initResult) return initResult.error();
    return Config::ErrorCode::OK;
}

/**
 * @brief Gets the communication handler.
 * @return A pointer to the communication handler.
 */
::Communication::UnifiedMAVLinkHandler* SystemContext::Communication::get() const {
    return handler_.get();
}

/**
 * @brief Initializes the safety subsystem.
 * @param hwManager A pointer to the hardware manager.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> SystemContext::Safety::initialize(HAL::HardwareManager* hwManager) {
    if (!hwManager) return Config::ErrorCode::NOT_INITIALIZED;
    return Config::ErrorCode::OK;
}

/**
 * @brief Initializes the system context and all its subsystems.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> SystemContext::initialize() {
    if (state.initialized) return Config::ErrorCode::OK;
    state.startTime = HAL_GetTick();
    state.lastUpdate = state.startTime;

    auto hwResult = hardware.initialize();
    if (!hwResult) {
        reportError(Config::ErrorCode::HARDWARE_ERROR, "Hardware initialization failed");
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    auto motorResult = motors.initialize(hardware.get());
    if (!motorResult) {
        reportError(motorResult.error(), "Motor subsystem initialization failed");
        return motorResult.error();
    }

    auto createResult = motors.createAllMotors();
    if (!createResult) {
        reportError(createResult.error(), "Motor creation failed");
        return createResult.error();
    }

    auto commResult = communication.initialize(hardware.get(), motors.getRegistry());
    if (!commResult) {
        reportError(commResult.error(), "Communication subsystem initialization failed");
        return commResult.error();
    }

    auto safetyResult = safety.initialize(hardware.get());
    if (!safetyResult) {
        reportError(safetyResult.error(), "Safety subsystem initialization failed");
        return safetyResult.error();
    }

    state.initialized = true;
    logSystemState();
    return Config::ErrorCode::OK;
}

/**
 * @brief Updates all subsystems.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> SystemContext::update() {
    if (!state.initialized) return Config::ErrorCode::NOT_INITIALIZED;
    uint32_t currentTime = HAL_GetTick();
    float deltaTime = (currentTime - state.lastUpdate) / 1000.0f;
    state.lastUpdate = currentTime;

    if (motors.getRegistry()) motors.getRegistry()->updateAll(deltaTime);
    if (communication.get()) {
        auto commResult = communication.get()->update();
        if (!commResult) reportError(commResult.error(), "Communication update failed");
    }

    if (state.emergencyStop && motors.getRegistry()) {
        motors.getRegistry()->emergencyStopAll();
    }
    return Config::ErrorCode::OK;
}

/**
 * @brief Shuts down all subsystems.
 */
void SystemContext::shutdown() {
    if (motors.getRegistry()) motors.getRegistry()->emergencyStopAll();
    communication.handler_.reset();
    motors.registry_.reset();
    motors.factory_.reset();
    safety.manager.reset();
    hardware.manager.reset();
    state.initialized = false;
}

/**
 * @brief Gets the system uptime in milliseconds.
 * @return The system uptime.
 */
uint32_t SystemContext::getUptime() const {
    return HAL_GetTick() - state.startTime;
}

/**
 * @brief Sets the emergency stop flag.
 * @param emergency true to activate emergency stop, false to deactivate.
 */
void SystemContext::setEmergencyStop(bool emergency) {
    state.emergencyStop = emergency;
    if (emergency && motors.getRegistry()) {
        motors.getRegistry()->emergencyStopAll();
    }
}

/**
 * @brief Reports an error.
 * @param error The error code.
 * @param context A description of the context in which the error occurred.
 */
void SystemContext::reportError(Config::ErrorCode error, const char* context) {
    state.errorCount++;
    state.lastError = error;
    (void)context;
    if (error == Config::ErrorCode::HARDWARE_ERROR || error == Config::ErrorCode::COMMUNICATION_ERROR) {
        setEmergencyStop(true);
    }
}

/**
 * @brief Logs the current system state.
 */
void SystemContext::logSystemState() {}

/**
 * @brief Initializes the application.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> Application::initialize() {
    auto result = context_.initialize();
    return result.isOk() ? Config::ErrorCode::OK : result.error();
}

/**
 * @brief Runs the main application loop.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> Application::run() {
    auto initResult = initialize();
    if (!initResult) return initResult.error();
    running_ = true;
    return mainLoop();
}

/**
 * @brief The main application loop.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> Application::mainLoop() {
    uint32_t lastUpdate = HAL_GetTick();
    while (running_) {
        uint32_t currentTime = HAL_GetTick();
        if (currentTime - lastUpdate >= updateInterval_) {
            auto updateResult = updateSubsystems();
            if (!updateResult) {
                handleErrors();
                if (updateResult.error() == Config::ErrorCode::HARDWARE_ERROR) break;
            }
            lastUpdate = currentTime;
        }
    }
    context_.shutdown();
    return Config::ErrorCode::OK;
}

/**
 * @brief Updates all subsystems.
 * @return A Result indicating success or failure.
 */
Config::Result<Config::ErrorCode> Application::updateSubsystems() {
    return context_.update();
}

/**
 * @brief Handles errors that occur in the main loop.
 */
void Application::handleErrors() {
    Config::ErrorCode lastError = context_.getLastError();
    if (lastError == Config::ErrorCode::HARDWARE_ERROR) {
        context_.setEmergencyStop(true);
    }
}

} // namespace System