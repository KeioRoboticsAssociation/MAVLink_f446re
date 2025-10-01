/**
 * @file servo_controller.hpp
 * @brief Defines the controller for servo motors.
 */

#pragma once

#include "../base/motor_interface.hpp"
#include "../../config/robot_config.hpp"
#include "../../hal/hardware_manager.hpp"

extern "C" {
#include "main.h"
}

namespace Motors {
namespace Servo {

/**
 * @brief A controller for servo motors.
 */
class ServoMotorController : public IMotorController<Config::ServoConfig> {
private:
    uint8_t id_;
    HAL::HardwareManager* hwManager_;
    HAL::TimerID timerId_;
    uint32_t channel_;
    Config::ServoConfig config_;
    MotorState state_;

    uint32_t lastWatchdogReset_;
    bool watchdogExpired_;

    std::function<void(uint8_t, MotorStatus)> errorCallback_;
    std::function<void(uint8_t, const MotorState&)> stateCallback_;

public:
    /**
     * @brief Construct a new ServoMotorController object.
     * @param id The ID of the motor.
     * @param hwManager A pointer to the hardware manager.
     * @param timerId The ID of the timer to use for PWM.
     * @param channel The timer channel.
     */
    ServoMotorController(uint8_t id, HAL::HardwareManager* hwManager, HAL::TimerID timerId, uint32_t channel);
    ~ServoMotorController() = default;

    Config::Result<void> initialize(const Config::ServoConfig& config) override;
    Config::Result<void> update(float deltaTime) override;
    Config::Result<void> setCommand(const MotorCommand& cmd) override;
    Config::Result<void> setEnabled(bool enabled) override;
    void emergencyStop() override;
    void resetWatchdog() override;
    Config::Result<void> runSelfTest() override;
    Config::Result<void> updateConfig(const Config::ServoConfig& config) override;

    MotorState getState() const override { return state_; }
    uint8_t getId() const override { return id_; }
    MotorStatus getStatus() const override { return state_.status; }
    Config::ServoConfig getConfig() const override { return config_; }

    void setErrorCallback(std::function<void(uint8_t, MotorStatus)> callback) override;
    void setStateCallback(std::function<void(uint8_t, const MotorState&)> callback) override;

private:
    float degreesToPulseWidth(float degrees);
    float pulseWidthToDegrees(float pulseWidth);
    void checkWatchdog();
    void reportError(MotorStatus error);
    void updateState();

    template<typename T>
    T constrainValue(T value, T min, T max);
};

} // namespace Servo
} // namespace Motors