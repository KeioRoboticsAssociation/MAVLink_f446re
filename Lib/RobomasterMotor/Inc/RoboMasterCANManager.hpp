#pragma once

#include "stm32f4xx_hal.h"
#include <cstdint>

// Forward declaration
class RoboMasterMotor;

enum class CANManagerStatus {
    OK = 0,
    CAN_ERROR,
    MOTOR_NOT_FOUND,
    BUFFER_FULL,
    INVALID_ID
};

namespace RoboMasterCANIDs {
    // Standard RoboMaster CAN IDs
    constexpr uint32_t MOTOR_CMD_ID_1_4 = 0x200;     // Motors 1-4 command
    constexpr uint32_t MOTOR_CMD_ID_5_8 = 0x1FF;     // Motors 5-8 command
    constexpr uint32_t MOTOR_FEEDBACK_BASE = 0x201;  // Motor feedback base ID
    
    // Additional command IDs for extended control
    constexpr uint32_t MOTOR_CMD_EXT_ID_1_4 = 0x1FE; // Extended command for motors 1-4
    constexpr uint32_t MOTOR_CMD_EXT_ID_5_8 = 0x1FD; // Extended command for motors 5-8
    
    // Special command IDs
    constexpr uint32_t BROADCAST_ID = 0x000;          // Broadcast to all motors
}

struct CANMotorCommand {
    uint8_t motor_id;
    int16_t current;
    bool valid;
    uint32_t timestamp;
};

class RoboMasterCANManager {
public:
    static constexpr uint8_t MAX_MOTORS = 8;
    static constexpr uint32_t CAN_TIMEOUT_MS = 100;

    RoboMasterCANManager();
    ~RoboMasterCANManager() = default;

    // Initialization
    CANManagerStatus init(CAN_HandleTypeDef* hcan);
    CANManagerStatus start();
    CANManagerStatus stop();

    // Motor registration
    CANManagerStatus registerMotor(RoboMasterMotor* motor, uint8_t motor_id);
    CANManagerStatus unregisterMotor(uint8_t motor_id);

    // Command transmission
    CANManagerStatus sendCurrentCommand(uint8_t motor_id, int16_t current);
    CANManagerStatus sendAllCurrentCommands();  // Send all pending commands
    
    // Emergency stop
    CANManagerStatus emergencyStop();
    CANManagerStatus emergencyStopMotor(uint8_t motor_id);

    // CAN message handling (called from interrupt)
    void handleCANReceive();
    void handleCANReceive(uint32_t rx_id, const uint8_t* data, uint8_t length);
    
    // Status and diagnostics
    bool isInitialized() const { return initialized_; }
    bool isStarted() const { return started_; }
    uint32_t getTransmissionErrors() const { return tx_error_count_; }
    uint32_t getReceptionErrors() const { return rx_error_count_; }
    uint32_t getLastTransmissionTime() const { return last_tx_time_; }
    
    // Update method (call periodically)
    void update();

private:
    CAN_HandleTypeDef* hcan_ = nullptr;
    bool initialized_ = false;
    bool started_ = false;
    
    // Motor registry
    RoboMasterMotor* motors_[MAX_MOTORS] = {nullptr};
    bool motor_registered_[MAX_MOTORS] = {false};
    
    // Command buffering
    CANMotorCommand command_buffer_1_4_[4];  // Commands for motors 1-4
    CANMotorCommand command_buffer_5_8_[4];  // Commands for motors 5-8
    bool pending_commands_1_4_ = false;
    bool pending_commands_5_8_ = false;
    
    // Error tracking
    uint32_t tx_error_count_ = 0;
    uint32_t rx_error_count_ = 0;
    uint32_t last_tx_time_ = 0;
    uint32_t last_rx_time_ = 0;
    
    // Internal methods
    CANManagerStatus setupCANFilters();
    CANManagerStatus transmitMessage(uint32_t id, const uint8_t* data, uint8_t length);
    CANManagerStatus transmitCurrentGroup(uint32_t cmd_id, const CANMotorCommand* commands);
    void processMotorFeedback(uint8_t motor_id, const uint8_t* data, uint8_t length);
    uint8_t getMotorIdFromCANId(uint32_t can_id) const;
    bool isValidMotorId(uint8_t motor_id) const;
    uint32_t getCurrentTimeMs() const;
    
    // CAN frame construction
    void buildCurrentCommandFrame(const CANMotorCommand* commands, uint8_t* frame_data);
    
    // Error handling
    void handleTransmissionError();
    void handleReceptionError();
    void clearCommandBuffer(CANMotorCommand* buffer, size_t size);
};