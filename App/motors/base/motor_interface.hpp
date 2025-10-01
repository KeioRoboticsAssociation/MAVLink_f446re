#pragma once

#include "../../config/system_config.hpp"
#include "../../config/motor_config.hpp"
#include <cstdint>
#include <functional>
#include <memory>

namespace Motors {

// Motor status using unified error codes
using MotorStatus = Config::ErrorCode;

// Base state structure for all motors
struct BaseMotorState {
    float currentPosition = 0.0f;      // Current position (degrees for servo, radians for others)
    float targetPosition = 0.0f;       // Target position
    float currentVelocity = 0.0f;      // Current velocity
    float targetVelocity = 0.0f;       // Target velocity
    float currentCurrent = 0.0f;       // Current draw in Amperes
    float temperature = 0.0f;          // Temperature in Celsius
    MotorStatus status = Config::ErrorCode::NOT_INITIALIZED;
    bool enabled = false;
    uint32_t lastUpdateTime = 0;
    uint32_t errorCount = 0;
    uint32_t timeoutCount = 0;
};

// Alias for compatibility
using MotorState = BaseMotorState;

// Motor control modes
enum class ControlMode : uint8_t {
    POSITION = 0,
    VELOCITY = 1,
    CURRENT = 2,
    DUTY_CYCLE = 3,
    DISABLED = 255
};

// Motor command structure
struct MotorCommand {
    uint8_t motorId;
    ControlMode mode;
    float targetValue;
    bool enable;
    uint32_t timestamp;
};

// Unified motor interface
template<typename TConfig>
class IMotorController {
public:
    virtual ~IMotorController() = default;

    // Core interface
    virtual Config::Result<void> initialize(const TConfig& config) = 0;
    virtual Config::Result<void> update(float deltaTime) = 0;
    virtual Config::Result<void> setCommand(const MotorCommand& cmd) = 0;
    virtual Config::Result<void> setEnabled(bool enabled) = 0;

    // State access
    virtual BaseMotorState getState() const = 0;
    virtual uint8_t getId() const = 0;
    virtual MotorStatus getStatus() const = 0;

    // Safety and maintenance
    virtual void emergencyStop() = 0;
    virtual void resetWatchdog() = 0;
    virtual Config::Result<void> runSelfTest() = 0;

    // Configuration
    virtual Config::Result<void> updateConfig(const TConfig& config) = 0;
    virtual TConfig getConfig() const = 0;

    // Callbacks for external systems
    virtual void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) = 0;
    virtual void setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) = 0;

protected:
    // Common validation helpers
    static bool isValidPosition(float position, float minPos, float maxPos) {
        return position >= minPos && position <= maxPos;
    }

    static bool isValidVelocity(float velocity, float maxVel) {
        return velocity >= -maxVel && velocity <= maxVel;
    }

    static float constrainValue(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
};

// Motor factory interface
class IMotorFactory {
public:
    virtual ~IMotorFactory() = default;
    virtual std::unique_ptr<IMotorController<Config::ServoConfig>> createServo(uint8_t id) = 0;
    virtual std::unique_ptr<IMotorController<Config::DCMotorConfig>> createDCMotor(uint8_t id) = 0;
    virtual std::unique_ptr<IMotorController<Config::RoboMasterConfig>> createRoboMaster(uint8_t id) = 0;
};

// Base class for type-erased motor controllers
class IMotorControllerBase {
public:
    virtual ~IMotorControllerBase() = default;
    virtual BaseMotorState getState() const = 0;
    virtual Config::Result<void> setCommand(const MotorCommand& cmd) = 0;
    virtual void emergencyStop() = 0;
    virtual Config::Result<void> update(float deltaTime) = 0;
    virtual uint8_t getId() const = 0;
};

// Template wrapper for type-erased motor controllers
template<typename TController>
class MotorControllerWrapper : public IMotorControllerBase {
private:
    std::unique_ptr<TController> controller_;

public:
    explicit MotorControllerWrapper(std::unique_ptr<TController> controller)
        : controller_(std::move(controller)) {}

    BaseMotorState getState() const override {
        return controller_->getState();
    }

    Config::Result<void> setCommand(const MotorCommand& cmd) override {
        return controller_->setCommand(cmd);
    }

    void emergencyStop() override {
        controller_->emergencyStop();
    }

    Config::Result<void> update(float deltaTime) override {
        return controller_->update(deltaTime);
    }

    uint8_t getId() const override {
        return controller_->getId();
    }

    TController* get() const { return controller_.get(); }
};

// Motor registry for managing all motor instances
class MotorRegistry {
private:
    struct MotorInstance {
        std::unique_ptr<IMotorControllerBase> controller;
        Config::MotorInstance::Type type;
    };

    std::array<MotorInstance, Config::System::MAX_MOTORS> motors_;
    size_t motorCount_ = 0;

public:
    template<typename TController>
    Config::Result<Config::ErrorCode> registerMotor(uint8_t id, std::unique_ptr<TController> controller) {
        if (motorCount_ >= Config::System::MAX_MOTORS) {
            return Config::ErrorCode::OUT_OF_RANGE;
        }

        // Check for duplicate IDs
        for (size_t i = 0; i < motorCount_; ++i) {
            if (motors_[i].controller && motors_[i].controller->getId() == id) {
                return Config::ErrorCode::CONFIG_ERROR;
            }
        }

        // Create type-erased wrapper
        MotorInstance instance;
        instance.controller = std::make_unique<MotorControllerWrapper<TController>>(std::move(controller));
        instance.type = Config::MotorInstance::Type::SERVO; // Default type, can be set properly later

        motors_[motorCount_++] = std::move(instance);
        return Config::ErrorCode::OK;
    }

    BaseMotorState getMotorState(uint8_t id) const;
    Config::Result<void> sendCommand(uint8_t id, const MotorCommand& cmd);
    void emergencyStopAll();
    void updateAll(float deltaTime);
    size_t getMotorCount() const { return motorCount_; }
};

} // namespace Motors