/**
 * @file dc_controller.hpp
 * @brief Defines the controller for DC motors.
 */

#pragma once

#include "../base/motor_interface.hpp"
#include "../../config/robot_config.hpp"
#include "../../hal/hardware_manager.hpp"

extern "C" {
#include "main.h"
}

namespace Motors {
namespace DC {

/**
 * @brief A controller for DC motors.
 */
class DCMotorController : public IMotorController<Config::DCMotorConfig> {
private:
    uint8_t id_;
    HAL::HardwareManager* hwManager_;
    HAL::TimerID timerId_;
    uint32_t channel_;
    Config::DCMotorConfig config_;
    MotorState state_;
    uint32_t lastWatchdogReset_;
    bool watchdogExpired_;
    float pidIntegral_;
    float pidLastError_;
    std::function<void(uint8_t, MotorStatus)> errorCallback_;
    std::function<void(uint8_t, const MotorState&)> stateCallback_;

public:
    /**
     * @brief Construct a new DCMotorController object.
     * @param id The ID of the motor.
     * @param hwManager A pointer to the hardware manager.
     * @param timerId The ID of the timer to use for PWM.
     * @param channel The timer channel.
     */
    DCMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel);
    ~DCMotorController() = default;

    Config::Result<void> initialize(const Config::DCMotorConfig& config) override;
    Config::Result<void> update(float deltaTime) override;
    Config::Result<void> setCommand(const MotorCommand& cmd) override;
    Config::Result<void> setEnabled(bool enabled) override;
    void emergencyStop() override;
    void resetWatchdog() override;
    Config::Result<void> runSelfTest() override;
    Config::Result<void> updateConfig(const Config::DCMotorConfig& config) override;

    MotorState getState() const override { return state_; }
    uint8_t getId() const override { return id_; }
    MotorStatus getStatus() const override { return state_.status; }
    Config::DCMotorConfig getConfig() const override { return config_; }

    void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) override;
    void setStateCallback(std::function<void(uint8_t, const MotorState&)> callback) override;

private:
    float calculatePID(float error, float& integral, float& lastError, float kp, float ki, float kd,
                      float maxIntegral, float maxOutput, float deltaTime);
    void checkWatchdog();
    void reportError(MotorStatus error);
    void updateState();

    template<typename T>
    T constrainValue(T value, T min, T max);
};

} // namespace DC
} // namespace Motors