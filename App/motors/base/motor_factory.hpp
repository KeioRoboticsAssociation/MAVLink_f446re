#pragma once

#include "motor_interface.hpp"
#include "../../hal/hardware_manager.hpp"
#include "../../config/robot_config.hpp"
#include "../servo/servo_controller.hpp"
#include "../dc/dc_controller.hpp"
#include "../robomaster/robomaster_controller.hpp"

namespace Motors {

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

} // namespace Motors