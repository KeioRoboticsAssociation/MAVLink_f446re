/**
 * @file motor_registry.cpp
 * @brief Implements the motor registry for managing all motor instances.
 */

#include "motor_interface.hpp"
#include "../../config/robot_config.hpp"

namespace Motors {

/**
 * @brief Gets the state of a motor.
 * @param id The ID of the motor.
 * @return The state of the motor. If the motor is not found, a default state with a status of NOT_INITIALIZED is returned.
 */
BaseMotorState MotorRegistry::getMotorState(uint8_t id) const {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller && motors_[i].controller->getId() == id) {
            return motors_[i].controller->getState();
        }
    }

    BaseMotorState defaultState;
    defaultState.status = Config::ErrorCode::NOT_INITIALIZED;
    return defaultState;
}

/**
 * @brief Sends a command to a motor.
 * @param id The ID of the motor.
 * @param cmd The command to send.
 * @return Result of the operation.
 */
Config::Result<void> MotorRegistry::sendCommand(uint8_t id, const MotorCommand& cmd) {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller && motors_[i].controller->getId() == id) {
            return motors_[i].controller->setCommand(cmd);
        }
    }

    return Config::Result<void>(Config::ErrorCode::NOT_INITIALIZED);
}

/**
 * @brief Stops all motors immediately.
 */
void MotorRegistry::emergencyStopAll() {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller) {
            motors_[i].controller->emergencyStop();
        }
    }
}

/**
 * @brief Updates all registered motors.
 * @param deltaTime The time since the last update.
 */
void MotorRegistry::updateAll(float deltaTime) {
    for (size_t i = 0; i < motorCount_; ++i) {
        if (motors_[i].controller) {
            motors_[i].controller->update(deltaTime);
        }
    }
}

} // namespace Motors