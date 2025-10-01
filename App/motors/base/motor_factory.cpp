#include "motor_factory.hpp"

namespace Motors {

std::unique_ptr<IMotorController<Config::ServoConfig>> ConcreteMotorFactory::createServo(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::SERVO) {
        return nullptr;
    }

    // Map timer ID to HAL::TimerID enum
    HAL::TimerID timerId = static_cast<HAL::TimerID>(instance->timer_id);

    return std::make_unique<Servo::ServoMotorController>(id, hwManager_, timerId, instance->channel);
}

std::unique_ptr<IMotorController<Config::DCMotorConfig>> ConcreteMotorFactory::createDCMotor(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::DC_MOTOR) {
        return nullptr;
    }

    // Map timer ID to HAL::TimerID enum
    HAL::TimerID timerId = static_cast<HAL::TimerID>(instance->timer_id);

    return std::make_unique<DC::DCMotorController>(id, hwManager_, timerId, instance->channel);
}

std::unique_ptr<IMotorController<Config::RoboMasterConfig>> ConcreteMotorFactory::createRoboMaster(uint8_t id) {
    const auto* instance = findMotorInstance(id);
    if (!instance || instance->type != Config::MotorInstance::Type::ROBOMASTER) {
        return nullptr;
    }

    return std::make_unique<RoboMaster::RoboMasterMotorController>(id, hwManager_);
}

const Config::MotorInstance* ConcreteMotorFactory::findMotorInstance(uint8_t id) {
    for (const auto& instance : Config::MOTOR_INSTANCES) {
        if (instance.id == id) {
            return &instance;
        }
    }
    return nullptr;
}

} // namespace Motors