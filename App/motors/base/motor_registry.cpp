#include "motor_interface.hpp"
#include "../../config/robot_config.hpp"

namespace Motors {

BaseMotorState MotorRegistry::getMotorState(uint8_t id) const {
    // Find motor by ID
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller && motors_[i].controller->getId() == id) {
            return motors_[i].controller->getState();
        }
    }

    // Return default state if not found
    BaseMotorState defaultState;
    defaultState.status = Config::ErrorCode::NOT_INITIALIZED;
    return defaultState;
}

Config::Result<void> MotorRegistry::sendCommand(uint8_t id, const MotorCommand& cmd) {
    // Find motor by ID and send command
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller && motors_[i].controller->getId() == id) {
            return motors_[i].controller->setCommand(cmd);
        }
    }

    return Config::Result<void>(Config::ErrorCode::NOT_INITIALIZED);
}

void MotorRegistry::emergencyStopAll() {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller) {
            motors_[i].controller->emergencyStop();
        }
    }
}

void MotorRegistry::updateAll(float deltaTime) {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller) {
            motors_[i].controller->update(deltaTime);
        }
    }
}

} // namespace Motors