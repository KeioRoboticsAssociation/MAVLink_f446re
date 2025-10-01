/**
 * @file motor_factory.hpp
 * @brief Defines a factory for creating motor controller instances.
 */

#pragma once

#include "motor_interface.hpp"
#include "../../hal/hardware_manager.hpp"
#include "../../config/robot_config.hpp"
#include "../servo/servo_controller.hpp"
#include "../dc/dc_controller.hpp"
#include "../robomaster/robomaster_controller.hpp"

namespace Motors {

/**
 * @brief A concrete implementation of the motor factory.
 */
class ConcreteMotorFactory : public IMotorFactory {
private:
    HAL::HardwareManager* hwManager_;

public:
    /**
     * @brief Construct a new ConcreteMotorFactory object.
     * @param hwManager A pointer to the hardware manager.
     */
    explicit ConcreteMotorFactory(HAL::HardwareManager* hwManager)
        : hwManager_(hwManager) {}

    /**
     * @brief Creates a new servo motor controller.
     * @param id The ID of the servo to create.
     * @return A unique pointer to the created servo controller.
     */
    std::unique_ptr<IMotorController<Config::ServoConfig>> createServo(uint8_t id) override;

    /**
     * @brief Creates a new DC motor controller.
     * @param id The ID of the DC motor to create.
     * @return A unique pointer to the created DC motor controller.
     */
    std::unique_ptr<IMotorController<Config::DCMotorConfig>> createDCMotor(uint8_t id) override;

    /**
     * @brief Creates a new RoboMaster motor controller.
     * @param id The ID of the RoboMaster motor to create.
     * @return A unique pointer to the created RoboMaster motor controller.
     */
    std::unique_ptr<IMotorController<Config::RoboMasterConfig>> createRoboMaster(uint8_t id) override;

private:
    const Config::MotorInstance* findMotorInstance(uint8_t id);
};

} // namespace Motors