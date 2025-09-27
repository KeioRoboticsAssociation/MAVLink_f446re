#include "system_context.hpp"
#include "motors/base/motor_factory.hpp"
#include "comm/unified_mavlink_handler.hpp"
#include "config/robot_config.hpp"

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace System {

// SystemContext::Motors implementation
Config::Result<Config::ErrorCode> SystemContext::Motors::initialize(HAL::HardwareManager* hwManager) {
    if (!hwManager) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    // Create motor factory
    factory_ = std::make_unique<::Motors::ConcreteMotorFactory>(hwManager);
    if (!factory_) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    // Create motor registry
    registry_ = std::make_unique<::Motors::MotorRegistry>();
    if (!registry_) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> SystemContext::Motors::createAllMotors() {
    if (!factory_ || !registry_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    // Create motors based on configuration
    for (const auto& motorInstance : Config::MOTOR_INSTANCES) {
        Config::Result<Config::ErrorCode> result = Config::ErrorCode::OK;

        switch (motorInstance.type) {
            case Config::MotorInstance::Type::SERVO: {
                auto servoController = factory_->createServo(motorInstance.id);
                if (!servoController) {
                    return Config::ErrorCode::CONFIG_ERROR;
                }

                // Initialize servo with default configuration - use default config for now
                Config::ServoConfig defaultServoConfig{};
                auto initResult = servoController->initialize(defaultServoConfig);
                if (!initResult) {
                    return Config::ErrorCode::HARDWARE_ERROR;
                }

                auto regResult = registry_->registerMotor(
                    motorInstance.id, std::move(servoController));
                if (!regResult) {
                    return regResult.error();
                }
                break;
            }

            case Config::MotorInstance::Type::DC_MOTOR: {
                auto dcController = factory_->createDCMotor(motorInstance.id);
                if (!dcController) {
                    return Config::ErrorCode::CONFIG_ERROR;
                }

                // Initialize DC motor with default configuration
                Config::DCMotorConfig defaultDCConfig{};
                auto initResult = dcController->initialize(defaultDCConfig);
                if (!initResult) {
                    return Config::ErrorCode::HARDWARE_ERROR;
                }

                auto regResult = registry_->registerMotor(
                    motorInstance.id, std::move(dcController));
                if (!regResult) {
                    return regResult.error();
                }
                break;
            }

            case Config::MotorInstance::Type::ROBOMASTER: {
                auto roboController = factory_->createRoboMaster(motorInstance.id);
                if (!roboController) {
                    return Config::ErrorCode::CONFIG_ERROR;
                }

                // Initialize RoboMaster with default configuration
                Config::RoboMasterConfig defaultRoboConfig{};
                auto initResult = roboController->initialize(defaultRoboConfig);
                if (!initResult) {
                    return Config::ErrorCode::HARDWARE_ERROR;
                }

                auto regResult = registry_->registerMotor(
                    motorInstance.id, std::move(roboController));
                if (!regResult) {
                    return regResult.error();
                }
                break;
            }

            default:
                return Config::ErrorCode::CONFIG_ERROR;
        }

    }

    return Config::ErrorCode::OK;
}

::Motors::MotorRegistry* SystemContext::Motors::getRegistry() const {
    return registry_.get();
}

::Motors::IMotorFactory* SystemContext::Motors::getFactory() const {
    return factory_.get();
}

// SystemContext::Communication implementation
Config::Result<Config::ErrorCode> SystemContext::Communication::initialize(HAL::HardwareManager* hwManager, ::Motors::MotorRegistry* motorRegistry) {
    if (!hwManager || !motorRegistry) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    // Create unified MAVLink handler with motor registry
    handler_ = std::make_unique<::Communication::UnifiedMAVLinkHandler>(
        hwManager,
        motorRegistry,
        Config::System::MAVLINK_SYSTEM_ID,
        Config::System::MAVLINK_COMPONENT_ID
    );

    if (!handler_) {
        return Config::ErrorCode::OUT_OF_RANGE;
    }

    // Initialize the communication handler
    auto initResult = handler_->initialize();
    if (!initResult) {
        return initResult.error();
    }

    return Config::ErrorCode::OK;
}

::Communication::UnifiedMAVLinkHandler* SystemContext::Communication::get() const {
    return handler_.get();
}

// SystemContext::Safety implementation
Config::Result<Config::ErrorCode> SystemContext::Safety::initialize(HAL::HardwareManager* hwManager) {
    if (!hwManager) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    // Create safety manager - implementation would be in a separate file
    // For now, return OK as stub
    return Config::ErrorCode::OK;
}


// SystemContext main implementation
Config::Result<Config::ErrorCode> SystemContext::initialize() {
    if (state.initialized) {
        return Config::ErrorCode::OK;
    }

    state.startTime = HAL_GetTick();
    state.lastUpdate = state.startTime;

    // Initialize hardware subsystem first
    auto hwResult = hardware.initialize();
    if (!hwResult) {
        reportError(Config::ErrorCode::HARDWARE_ERROR, "Hardware initialization failed");
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    // Initialize motor subsystem
    auto motorResult = motors.initialize(hardware.get());
    if (!motorResult) {
        reportError(motorResult.error(), "Motor subsystem initialization failed");
        return motorResult.error();
    }

    // Create all configured motors
    auto createResult = motors.createAllMotors();
    if (!createResult) {
        reportError(createResult.error(), "Motor creation failed");
        return createResult.error();
    }

    // Initialize communication subsystem
    auto commResult = communication.initialize(hardware.get(), motors.getRegistry());
    if (!commResult) {
        reportError(commResult.error(), "Communication subsystem initialization failed");
        return commResult.error();
    }

    // Initialize safety subsystem
    auto safetyResult = safety.initialize(hardware.get());
    if (!safetyResult) {
        reportError(safetyResult.error(), "Safety subsystem initialization failed");
        return safetyResult.error();
    }

    state.initialized = true;
    logSystemState();

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> SystemContext::update() {
    if (!state.initialized) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    uint32_t currentTime = HAL_GetTick();
    float deltaTime = (currentTime - state.lastUpdate) / 1000.0f;
    state.lastUpdate = currentTime;

    // Update motor subsystem
    if (motors.getRegistry()) {
        motors.getRegistry()->updateAll(deltaTime);
    }

    // Update communication subsystem
    if (communication.get()) {
        auto commResult = communication.get()->update();
        if (!commResult) {
            reportError(commResult.error(), "Communication update failed");
        }
    }

    // Update safety subsystem
    if (safety.get()) {
        // auto safetyResult = safety.get()->update();
        // if (!safetyResult) {
        //     handleError(safetyResult.error());
        // }
    }

    // Check for emergency stop conditions
    if (state.emergencyStop) {
        if (motors.getRegistry()) {
            motors.getRegistry()->emergencyStopAll();
        }
    }

    return Config::ErrorCode::OK;
}

void SystemContext::shutdown() {
    // Stop all motors
    if (motors.getRegistry()) {
        motors.getRegistry()->emergencyStopAll();
    }

    // Reset subsystems
    communication.handler_.reset();
    motors.registry_.reset();
    motors.factory_.reset();
    safety.manager.reset();
    hardware.manager.reset();

    state.initialized = false;
}

uint32_t SystemContext::getUptime() const {
    return HAL_GetTick() - state.startTime;
}

void SystemContext::setEmergencyStop(bool emergency) {
    state.emergencyStop = emergency;
    if (emergency && motors.getRegistry()) {
        motors.getRegistry()->emergencyStopAll();
    }
}

void SystemContext::reportError(Config::ErrorCode error, const char* context) {
    state.errorCount++;
    state.lastError = error;

    // Log error context if provided
    (void)context; // Suppress unused warning for now

    // Handle critical errors
    if (error == Config::ErrorCode::HARDWARE_ERROR ||
        error == Config::ErrorCode::COMMUNICATION_ERROR) {
        setEmergencyStop(true);
    }
}

void SystemContext::logSystemState() {
    // Log current system state for debugging
    // Implementation would depend on available logging mechanism
}

// Application implementation
Config::Result<Config::ErrorCode> Application::initialize() {
    auto result = context_.initialize();
    if (!result) {
        return result.error();
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> Application::run() {
    auto initResult = initialize();
    if (!initResult) {
        return initResult.error();
    }

    running_ = true;
    return mainLoop();
}

Config::Result<Config::ErrorCode> Application::mainLoop() {
    uint32_t lastUpdate = HAL_GetTick();

    while (running_) {
        uint32_t currentTime = HAL_GetTick();

        // Update at specified interval
        if (currentTime - lastUpdate >= updateInterval_) {
            auto updateResult = updateSubsystems();
            if (!updateResult) {
                handleErrors();
                if (updateResult.error() == Config::ErrorCode::HARDWARE_ERROR) {
                    // Critical error - stop execution
                    break;
                }
            }
            lastUpdate = currentTime;
        }

        // Allow other tasks to run (if using RTOS)
        // HAL_Delay(1);
    }

    context_.shutdown();
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> Application::updateSubsystems() {
    return context_.update();
}

void Application::handleErrors() {
    Config::ErrorCode lastError = context_.getLastError();

    switch (lastError) {
        case Config::ErrorCode::TIMEOUT:
            // Handle timeout errors
            break;
        case Config::ErrorCode::COMMUNICATION_ERROR:
            // Handle communication errors
            break;
        case Config::ErrorCode::HARDWARE_ERROR:
            // Handle hardware errors - trigger emergency stop
            context_.setEmergencyStop(true);
            break;
        default:
            break;
    }
}

} // namespace System