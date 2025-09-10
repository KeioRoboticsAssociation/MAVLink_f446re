#include "RoboMasterCANManager.hpp"
#include "RoboMasterMotor.hpp"
#include <cstring>

RoboMasterCANManager::RoboMasterCANManager() {
    // Initialize motor registry
    for (int i = 0; i < MAX_MOTORS; i++) {
        motors_[i] = nullptr;
        motor_registered_[i] = false;
    }
    
    // Initialize command buffers
    clearCommandBuffer(command_buffer_1_4_, 4);
    clearCommandBuffer(command_buffer_5_8_, 4);
}

CANManagerStatus RoboMasterCANManager::init(CAN_HandleTypeDef* hcan) {
    if (hcan == nullptr) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    hcan_ = hcan;
    
    // Setup CAN filters
    CANManagerStatus status = setupCANFilters();
    if (status != CANManagerStatus::OK) {
        return status;
    }
    
    initialized_ = true;
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::start() {
    if (!initialized_) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    // Start CAN peripheral
    if (HAL_CAN_Start(hcan_) != HAL_OK) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    // Activate notifications
    if (HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    started_ = true;
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::stop() {
    if (!initialized_) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    // Send emergency stop to all motors
    emergencyStop();
    
    // Stop CAN peripheral
    HAL_CAN_Stop(hcan_);
    started_ = false;
    
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::registerMotor(RoboMasterMotor* motor, uint8_t motor_id) {
    if (!isValidMotorId(motor_id) || motor == nullptr) {
        return CANManagerStatus::INVALID_ID;
    }
    
    uint8_t array_index = motor_id - 1;  // Convert to 0-based index
    
    if (motor_registered_[array_index]) {
        return CANManagerStatus::INVALID_ID;  // Already registered
    }
    
    motors_[array_index] = motor;
    motor_registered_[array_index] = true;
    
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::unregisterMotor(uint8_t motor_id) {
    if (!isValidMotorId(motor_id)) {
        return CANManagerStatus::INVALID_ID;
    }
    
    uint8_t array_index = motor_id - 1;
    
    motors_[array_index] = nullptr;
    motor_registered_[array_index] = false;
    
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::sendCurrentCommand(uint8_t motor_id, int16_t current) {
    if (!isValidMotorId(motor_id)) {
        return CANManagerStatus::INVALID_ID;
    }
    
    // Determine which command buffer to use
    CANMotorCommand* buffer;
    bool* pending_flag;
    uint8_t buffer_index;
    
    if (motor_id <= 4) {
        buffer = command_buffer_1_4_;
        pending_flag = &pending_commands_1_4_;
        buffer_index = motor_id - 1;
    } else {
        buffer = command_buffer_5_8_;
        pending_flag = &pending_commands_5_8_;
        buffer_index = motor_id - 5;
    }
    
    // Update command buffer
    buffer[buffer_index].motor_id = motor_id;
    buffer[buffer_index].current = current;
    buffer[buffer_index].valid = true;
    buffer[buffer_index].timestamp = getCurrentTimeMs();
    
    *pending_flag = true;
    
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::sendAllCurrentCommands() {
    if (!started_) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    CANManagerStatus status = CANManagerStatus::OK;
    
    // Send commands for motors 1-4 if pending
    if (pending_commands_1_4_) {
        CANManagerStatus result = transmitCurrentGroup(RoboMasterCANIDs::MOTOR_CMD_ID_1_4, command_buffer_1_4_);
        if (result == CANManagerStatus::OK) {
            pending_commands_1_4_ = false;
        } else {
            status = result;
        }
    }
    
    // Send commands for motors 5-8 if pending
    if (pending_commands_5_8_) {
        CANManagerStatus result = transmitCurrentGroup(RoboMasterCANIDs::MOTOR_CMD_ID_5_8, command_buffer_5_8_);
        if (result == CANManagerStatus::OK) {
            pending_commands_5_8_ = false;
        } else {
            status = result;
        }
    }
    
    return status;
}

CANManagerStatus RoboMasterCANManager::emergencyStop() {
    // Send zero current to all motors immediately
    uint8_t zero_frame[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    
    transmitMessage(RoboMasterCANIDs::MOTOR_CMD_ID_1_4, zero_frame, 8);
    transmitMessage(RoboMasterCANIDs::MOTOR_CMD_ID_5_8, zero_frame, 8);
    
    // Clear all pending commands
    clearCommandBuffer(command_buffer_1_4_, 4);
    clearCommandBuffer(command_buffer_5_8_, 4);
    pending_commands_1_4_ = false;
    pending_commands_5_8_ = false;
    
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::emergencyStopMotor(uint8_t motor_id) {
    return sendCurrentCommand(motor_id, 0);
}

void RoboMasterCANManager::handleCANReceive() {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    // Get message from FIFO0
    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        handleCANReceive(rx_header.StdId, rx_data, rx_header.DLC);
        last_rx_time_ = getCurrentTimeMs();
    } else {
        rx_error_count_++;
        handleReceptionError();
    }
}

void RoboMasterCANManager::handleCANReceive(uint32_t rx_id, const uint8_t* data, uint8_t length) {
    // Determine motor ID from CAN ID
    uint8_t motor_id = getMotorIdFromCANId(rx_id);
    
    if (motor_id > 0 && motor_id <= MAX_MOTORS) {
        processMotorFeedback(motor_id, data, length);
    }
}

void RoboMasterCANManager::update() {
    if (!started_) {
        return;
    }
    
    // Automatically send pending commands
    sendAllCurrentCommands();
    
    // Update all registered motors
    for (int i = 0; i < MAX_MOTORS; i++) {
        if (motor_registered_[i] && motors_[i] != nullptr) {
            motors_[i]->update();
        }
    }
}

CANManagerStatus RoboMasterCANManager::setupCANFilters() {
    CAN_FilterTypeDef filter_config;
    
    // Configure filter to accept RoboMaster feedback messages (0x201-0x208)
    filter_config.FilterBank = 0;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = (RoboMasterCANIDs::MOTOR_FEEDBACK_BASE << 5);
    filter_config.FilterIdLow = 0;
    filter_config.FilterMaskIdHigh = (0x1F8 << 5);  // Mask to accept 0x201-0x208
    filter_config.FilterMaskIdLow = 0;
    filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(hcan_, &filter_config) != HAL_OK) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::transmitMessage(uint32_t id, const uint8_t* data, uint8_t length) {
    if (!started_) {
        return CANManagerStatus::CAN_ERROR;
    }
    
    // Check if mailbox is available
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0) {
        return CANManagerStatus::BUFFER_FULL;
    }
    
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;
    
    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(hcan_, &tx_header, const_cast<uint8_t*>(data), &tx_mailbox) != HAL_OK) {
        tx_error_count_++;
        handleTransmissionError();
        return CANManagerStatus::CAN_ERROR;
    }
    
    last_tx_time_ = getCurrentTimeMs();
    return CANManagerStatus::OK;
}

CANManagerStatus RoboMasterCANManager::transmitCurrentGroup(uint32_t cmd_id, const CANMotorCommand* commands) {
    uint8_t frame_data[8];
    buildCurrentCommandFrame(commands, frame_data);
    
    return transmitMessage(cmd_id, frame_data, 8);
}

void RoboMasterCANManager::processMotorFeedback(uint8_t motor_id, const uint8_t* data, uint8_t length) {
    uint8_t array_index = motor_id - 1;
    
    if (motor_registered_[array_index] && motors_[array_index] != nullptr) {
        motors_[array_index]->processCANData(data, length);
    }
}

uint8_t RoboMasterCANManager::getMotorIdFromCANId(uint32_t can_id) const {
    if (can_id >= RoboMasterCANIDs::MOTOR_FEEDBACK_BASE && 
        can_id < RoboMasterCANIDs::MOTOR_FEEDBACK_BASE + MAX_MOTORS) {
        return static_cast<uint8_t>(can_id - RoboMasterCANIDs::MOTOR_FEEDBACK_BASE + 1);
    }
    
    return 0;  // Invalid motor ID
}

bool RoboMasterCANManager::isValidMotorId(uint8_t motor_id) const {
    return (motor_id >= 1 && motor_id <= MAX_MOTORS);
}

uint32_t RoboMasterCANManager::getCurrentTimeMs() const {
    return HAL_GetTick();
}

void RoboMasterCANManager::buildCurrentCommandFrame(const CANMotorCommand* commands, uint8_t* frame_data) {
    for (int i = 0; i < 4; i++) {
        int16_t current = commands[i].valid ? commands[i].current : 0;
        
        // Convert to big-endian format
        frame_data[i * 2] = static_cast<uint8_t>((current >> 8) & 0xFF);
        frame_data[i * 2 + 1] = static_cast<uint8_t>(current & 0xFF);
    }
}

void RoboMasterCANManager::handleTransmissionError() {
    // Could implement error recovery logic here
}

void RoboMasterCANManager::handleReceptionError() {
    // Could implement error recovery logic here
}

void RoboMasterCANManager::clearCommandBuffer(CANMotorCommand* buffer, size_t size) {
    for (size_t i = 0; i < size; i++) {
        buffer[i].motor_id = 0;
        buffer[i].current = 0;
        buffer[i].valid = false;
        buffer[i].timestamp = 0;
    }
}