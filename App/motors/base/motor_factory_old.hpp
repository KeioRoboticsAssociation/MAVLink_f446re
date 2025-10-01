#pragma once

#include "motor_interface.hpp"
#include "../../hal/hardware_manager.hpp"
#include "../../config/robot_config.hpp"

namespace Motors {

// Forward declarations for concrete motor implementations
class ServoMotorController;
class DCMotorController;
class RoboMasterMotorController;

// Concrete motor factory implementation
class ConcreteMotorFactory : public IMotorFactory {
private:
    HAL::HardwareManager* hwManager_;

public:
    explicit ConcreteMotorFactory(HAL::HardwareManager* hwManager)
        : hwManager_(hwManager) {}

    std::unique_ptr<IMotorController<Config::ServoConfig>> createServo(uint8_t id) override;
    std::unique_ptr<IMotorController<Config::DCMotorConfig>> createDCMotor(uint8_t id) override;
    std::unique_ptr<IMotorController<Config::RoboMasterConfig>> createRoboMaster(uint8_t id) override;

private:
    // Helper to find hardware mapping for motor ID
    const Config::MotorInstance* findMotorInstance(uint8_t id);
};

// Servo motor controller implementation
class ServoMotorController : public IMotorController<Config::ServoConfig> {
public:
    using config_type = Config::ServoConfig;

private:
    uint8_t id_;
    Config::ServoConfig config_;
    BaseMotorState state_;
    HAL::HardwareManager* hwManager_;
    HAL::TimerID timerId_;
    uint32_t channel_;

    // Callbacks
    std::function<void(uint8_t, MotorStatus)> errorCallback_;
    std::function<void(uint8_t, const BaseMotorState&)> stateCallback_;

    // Control state
    uint32_t lastWatchdogReset_ = 0;
    bool watchdogExpired_ = false;

public:
    ServoMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel);

    // IMotorController implementation
    Config::Result<void> initialize(const Config::ServoConfig& config) override;
    Config::Result<void> update(float deltaTime) override;
    Config::Result<void> setCommand(const MotorCommand& cmd) override;
    Config::Result<void> setEnabled(bool enabled) override;

    BaseMotorState getState() const override { return state_; }
    uint8_t getId() const override { return id_; }
    MotorStatus getStatus() const override { return state_.status; }

    void emergencyStop() override;
    void resetWatchdog() override;
    Config::Result<void> runSelfTest() override;

    Config::Result<void> updateConfig(const Config::ServoConfig& config) override;
    Config::ServoConfig getConfig() const override { return config_; }

    void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) override {
        errorCallback_ = callback;
    }
    void setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) override {
        stateCallback_ = callback;
    }

private:
    float degreesToPulseWidth(float degrees);
    float pulseWidthToDegrees(float pulseWidth);
    void checkWatchdog();
    void reportError(MotorStatus error);
    void updateState();
};

// DC Motor controller implementation
class DCMotorController : public IMotorController<Config::DCMotorConfig> {
public:
    using config_type = Config::DCMotorConfig;

private:
    uint8_t id_;
    Config::DCMotorConfig config_;
    BaseMotorState state_;
    HAL::HardwareManager* hwManager_;
    HAL::TimerID timerId_;
    uint32_t channel_;

    // Callbacks
    std::function<void(uint8_t, MotorStatus)> errorCallback_;
    std::function<void(uint8_t, const BaseMotorState&)> stateCallback_;

    // PID control state
    float speedIntegral_ = 0.0f;
    float speedLastError_ = 0.0f;
    float positionIntegral_ = 0.0f;
    float positionLastError_ = 0.0f;
    uint32_t lastControlTime_ = 0;

    // Watchdog
    uint32_t lastWatchdogReset_ = 0;
    bool watchdogExpired_ = false;

public:
    DCMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel);

    // IMotorController implementation
    Config::Result<void> initialize(const Config::DCMotorConfig& config) override;
    Config::Result<void> update(float deltaTime) override;
    Config::Result<void> setCommand(const MotorCommand& cmd) override;
    Config::Result<void> setEnabled(bool enabled) override;

    BaseMotorState getState() const override { return state_; }
    uint8_t getId() const override { return id_; }
    MotorStatus getStatus() const override { return state_.status; }

    void emergencyStop() override;
    void resetWatchdog() override;
    Config::Result<void> runSelfTest() override;

    Config::Result<void> updateConfig(const Config::DCMotorConfig& config) override;
    Config::DCMotorConfig getConfig() const override { return config_; }

    void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) override {
        errorCallback_ = callback;
    }
    void setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) override {
        stateCallback_ = callback;
    }

private:
    float calculatePID(float error, float& integral, float& lastError, float kp, float ki, float kd,
                      float maxIntegral, float maxOutput, float deltaTime);
    void checkWatchdog();
    void reportError(MotorStatus error);
    void updateState();
};

// RoboMaster motor controller implementation (placeholder)
class RoboMasterMotorController : public IMotorController<Config::RoboMasterConfig> {
public:
    using config_type = Config::RoboMasterConfig;

private:
    uint8_t id_;
    Config::RoboMasterConfig config_;
    BaseMotorState state_;
    HAL::HardwareManager* hwManager_;

    // Callbacks
    std::function<void(uint8_t, MotorStatus)> errorCallback_;
    std::function<void(uint8_t, const BaseMotorState&)> stateCallback_;

public:
    RoboMasterMotorController(uint8_t id, HAL::HardwareManager* hwManager);

    // IMotorController implementation
    Config::Result<void> initialize(const Config::RoboMasterConfig& config) override;
    Config::Result<void> update(float deltaTime) override;
    Config::Result<void> setCommand(const MotorCommand& cmd) override;
    Config::Result<void> setEnabled(bool enabled) override;

    BaseMotorState getState() const override { return state_; }
    uint8_t getId() const override { return id_; }
    MotorStatus getStatus() const override { return state_.status; }

    void emergencyStop() override;
    void resetWatchdog() override;
    Config::Result<void> runSelfTest() override;

    Config::Result<void> updateConfig(const Config::RoboMasterConfig& config) override;
    Config::RoboMasterConfig getConfig() const override { return config_; }

    void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) override {
        errorCallback_ = callback;
    }
    void setStateCallback(std::function<void(uint8_t, const BaseMotorState&)> callback) override {
        stateCallback_ = callback;
    }

private:
    void reportError(MotorStatus error);
    void updateState();
};

} // namespace Motors