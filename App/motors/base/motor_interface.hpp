/**
 * @file motor_interface.hpp
 * @brief Defines the core interfaces and data structures for motor control.
 */

#pragma once

#include "../../config/system_config.hpp"
#include "../../config/motor_config.hpp"
#include <cstdint>
#include <functional>
#include <memory>

namespace Motors {

/**
 * @brief Represents the status of a motor, using the unified error code system.
 */
using MotorStatus = Config::ErrorCode;

/**
 * @brief Base state structure for all motors.
 */
struct BaseMotorState {
    float currentPosition = 0.0f;
    float targetPosition = 0.0f;
    float currentVelocity = 0.0f;
    float targetVelocity = 0.0f;
    float currentCurrent = 0.0f;
    float temperature = 0.0f;
    MotorStatus status = Config::ErrorCode::NOT_INITIALIZED;
    bool enabled = false;
    uint32_t lastUpdateTime = 0;
    uint32_t errorCount = 0;
    uint32_t timeoutCount = 0;
};

using MotorState = BaseMotorState;

/**
 * @brief Defines the control modes for a motor.
 */
enum class ControlMode : uint8_t {
    POSITION = 0,
    VELOCITY = 1,
    CURRENT = 2,
    DUTY_CYCLE = 3,
    DISABLED = 255
};

/**
 * @brief Represents a command to be sent to a motor.
 */
struct MotorCommand {
    uint8_t motorId;
    ControlMode mode;
    float targetValue;
    bool enable;
    uint32_t timestamp;
};

/**
 * @brief Unified interface for all motor controllers.
 * @tparam TConfig The configuration type for the motor controller.
 */
template<typename TConfig>
class IMotorController {
public:
    virtual ~IMotorController() = default;

    virtual Config::Result<void> initialize(const TConfig& config) = 0;
    virtual Config::Result<void> update(float deltaTime) = 0;
    virtual Config::Result<void> setCommand(const MotorCommand& cmd) = 0;
    virtual Config::Result<void> setEnabled(bool enabled) = 0;

    virtual BaseMotorState getState() const = 0;
    virtual uint8_t getId() const = 0;
    virtual MotorStatus getStatus() const = 0;

    virtual void emergencyStop() = 0;
    virtual void resetWatchdog() = 0;
    virtual Config::Result<void> runSelfTest() = 0;

    virtual Config::Result<void> updateConfig(const TConfig& config) = 0;
    virtual TConfig getConfig() const = 0;

    virtual void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) = 0;
    virtual void setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) = 0;

protected:
    static bool isValidPosition(float position, float minPos, float maxPos);
    static bool isValidVelocity(float velocity, float maxVel);
    static float constrainValue(float value, float min, float max);
};

/**
 * @brief Interface for a motor factory.
 */
class IMotorFactory {
public:
    virtual ~IMotorFactory() = default;
    virtual std::unique_ptr<IMotorController<Config::ServoConfig>> createServo(uint8_t id) = 0;
    virtual std::unique_ptr<IMotorController<Config::DCMotorConfig>> createDCMotor(uint8_t id) = 0;
    virtual std::unique_ptr<IMotorController<Config::RoboMasterConfig>> createRoboMaster(uint8_t id) = 0;
};

/**
 * @brief Base class for type-erased motor controllers.
 */
class IMotorControllerBase {
public:
    virtual ~IMotorControllerBase() = default;
    virtual BaseMotorState getState() const = 0;
    virtual Config::Result<void> setCommand(const MotorCommand& cmd) = 0;
    virtual void emergencyStop() = 0;
    virtual Config::Result<void> update(float deltaTime) = 0;
    virtual uint8_t getId() const = 0;
};

/**
 * @brief Template wrapper for type-erased motor controllers.
 * @tparam TController The specific motor controller type.
 */
template<typename TController>
class MotorControllerWrapper : public IMotorControllerBase {
private:
    std::unique_ptr<TController> controller_;

public:
    explicit MotorControllerWrapper(std::unique_ptr<TController> controller);
    BaseMotorState getState() const override;
    Config::Result<void> setCommand(const MotorCommand& cmd) override;
    void emergencyStop() override;
    Config::Result<void> update(float deltaTime) override;
    uint8_t getId() const override;
    TController* get() const;
};

/**
 * @brief Manages all motor instances in the system.
 */
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
    Config::Result<Config::ErrorCode> registerMotor(uint8_t id, std::unique_ptr<TController> controller);
    BaseMotorState getMotorState(uint8_t id) const;
    Config::Result<void> sendCommand(uint8_t id, const MotorCommand& cmd);
    void emergencyStopAll();
    void updateAll(float deltaTime);
    size_t getMotorCount() const { return motorCount_; }
};

} // namespace Motors