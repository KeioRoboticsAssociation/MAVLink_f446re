#pragma once

#include "../base/motor_interface.hpp"
#include "../../config/robot_config.hpp"
#include "../../hal/hardware_manager.hpp"

extern "C" {
#include "main.h"
}

namespace Motors {
namespace RoboMaster {

class RoboMasterMotorController : public IMotorController<Config::RoboMasterConfig> {
private:
    uint8_t id_;
    HAL::HardwareManager* hwManager_;
    Config::RoboMasterConfig config_;
    MotorState state_;

    uint32_t lastWatchdogReset_;
    bool watchdogExpired_;

    // CAN communication variables
    uint32_t canId_;
    uint32_t lastCanRx_;

    // PID control variables for angle and speed
    float anglePidIntegral_;
    float anglePidLastError_;
    float speedPidIntegral_;
    float speedPidLastError_;

    std::function<void(uint8_t, MotorStatus)> errorCallback_;
    std::function<void(uint8_t, const MotorState&)> stateCallback_;

public:
    RoboMasterMotorController(uint8_t id, HAL::HardwareManager* hwManager);
    ~RoboMasterMotorController() = default;

    // IMotorController implementation
    Config::Result<void> initialize(const Config::RoboMasterConfig& config) override;
    Config::Result<void> update(float deltaTime) override;
    Config::Result<void> setCommand(const MotorCommand& cmd) override;
    Config::Result<void> setEnabled(bool enabled) override;
    void emergencyStop() override;
    void resetWatchdog() override;
    Config::Result<void> runSelfTest() override;
    Config::Result<void> updateConfig(const Config::RoboMasterConfig& config) override;

    MotorState getState() const override { return state_; }
    uint8_t getId() const override { return id_; }
    MotorStatus getStatus() const override { return state_.status; }
    Config::RoboMasterConfig getConfig() const override { return config_; }

    void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) override {
        errorCallback_ = callback;
    }

    void setStateCallback(std::function<void(uint8_t, const MotorState&)> callback) override {
        stateCallback_ = callback;
    }

    // RoboMaster specific methods
    Config::Result<void> processCanMessage(uint32_t canId, const uint8_t* data, uint8_t length);
    Config::Result<void> sendCanCommand(int16_t current);

private:
    float calculatePID(float error, float& integral, float& lastError, float kp, float ki, float kd,
                      float maxIntegral, float maxOutput, float deltaTime);
    void checkWatchdog();
    void reportError(MotorStatus error);
    void updateState();

    template<typename T>
    T constrainValue(T value, T min, T max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
};

} // namespace RoboMaster
} // namespace Motors