#include "motor_interface.hpp"
#include "../../config/robot_config.hpp"

namespace Motors {

BaseMotorState MotorRegistry::getMotorState(uint8_t id) const {
    // Find motor by ID
    for (size_t i = 0; i < motorCount_; ++i) {
        // For now, return the state using the function wrapper
        // In a real implementation, we would need to store the ID mapping
        if (i < motorCount_ && motors_[i].getState) {
            return motors_[i].getState();
        }
    }

    // Return default state if not found
    BaseMotorState defaultState;
    defaultState.status = MotorStatus::NOT_INITIALIZED;
    return defaultState;
}

Config::Result<MotorStatus> MotorRegistry::sendCommand(uint8_t id, const MotorCommand& cmd) {
    // Find motor by ID and send command
    for (size_t i = 0; i < motorCount_; ++i) {
        // For now, send to the first available motor
        // In a real implementation, we would need proper ID mapping
        if (i < motorCount_ && motors_[i].setCommand) {
            return motors_[i].setCommand(cmd);
        }
    }

    return Config::Result<MotorStatus>(MotorStatus::NOT_INITIALIZED);
}

void MotorRegistry::emergencyStopAll() {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].emergencyStop) {
            motors_[i].emergencyStop();
        }
    }
}

void MotorRegistry::updateAll(float deltaTime) {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].update) {
            motors_[i].update(deltaTime);
        }
    }
}

} // namespace Motors