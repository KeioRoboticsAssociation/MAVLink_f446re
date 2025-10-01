/**
 * @file motor_factory.cpp
 * @brief Implements the factory for creating motor controller instances.
 */

#include "motor_factory.hpp"

namespace Motors {

/**
 * @brief Creates a new servo motor controller.
 * @param id The ID of the servo to create.
 * @return A unique pointer to the created servo controller, or nullptr if creation fails.
 */
std::unique_ptr<IMotorController<Config::ServoConfig>> ConcreteMotorFactory::createServo(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::SERVO) {
        return nullptr;
    }

    HAL::TimerID timerId = static_cast<HAL::TimerID>(instance->timer_id);
    return std::make_unique<Servo::ServoMotorController>(id, hwManager_, timerId, instance->channel);
}

/**
 * @brief Creates a new DC motor controller.
 * @param id The ID of the DC motor to create.
 * @return A unique pointer to the created DC motor controller, or nullptr if creation fails.
 */
std::unique_ptr<IMotorController<Config::DCMotorConfig>> ConcreteMotorFactory::createDCMotor(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::DC_MOTOR) {
        return nullptr;
    }

    HAL::TimerID timerId = static_cast<HAL::TimerID>(instance->timer_id);
    return std::make_unique<DC::DCMotorController>(id, hwManager_, timerId, instance->channel);
}

/**
 * @brief Creates a new RoboMaster motor controller.
 * @param id The ID of the RoboMaster motor to create.
 * @return A unique pointer to the created RoboMaster motor controller, or nullptr if creation fails.
 */
std::unique_ptr<IMotorController<Config::RoboMasterConfig>> ConcreteMotorFactory::createRoboMaster(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::ROBOMASTER) {
        return nullptr;
    }

    return std::make_unique<RoboMaster::RoboMasterMotorController>(id, hwManager_);
}

/**
 * @brief Finds the motor instance configuration for a given motor ID.
 * @param id The ID of the motor.
 * @return A pointer to the motor instance configuration, or nullptr if not found.
 */
const Config::MotorInstance* ConcreteMotorFactory::findMotorInstance(uint8_t id) {
    for (const auto& instance : Config::MOTOR_INSTANCES) {
        if (instance.id == id) {
            return &instance;
        }
    }
    return nullptr;
}

} // namespace Motors