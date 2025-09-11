#pragma once

#include "stm32f4xx_hal.h"
#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"
#include "../../mavlink/c_library_v2/common/mavlink.h"

/**
 * @brief MAVLink interface for RoboMaster motor control and parameter management
 * 
 * This class provides ROS/MAVLink communication interface for:
 * - Reading/writing motor parameters (PID gains, limits, etc.)
 * - Motor control commands (position, velocity, current)
 * - Status monitoring and telemetry
 * - Configuration save/load functionality
 * - Emergency stop and safety commands
 */
class MAVLinkRoboMasterController {
public:
    static constexpr uint8_t MAX_MOTORS = 8;
    static constexpr uint32_t TELEMETRY_RATE_MS = 100;  // 10Hz telemetry
    static constexpr uint32_t HEARTBEAT_RATE_MS = 1000; // 1Hz heartbeat

    MAVLinkRoboMasterController();
    ~MAVLinkRoboMasterController() = default;

    // Initialization
    void init(UART_HandleTypeDef* uart, uint8_t system_id = 1);
    void addMotor(RoboMasterMotor* motor, uint8_t motor_id);
    void setCANManager(RoboMasterCANManager* can_manager);
    
    // Communication
    void update();
    void processReceivedByte(uint8_t byte);
    void handleMessage(mavlink_message_t* msg);

    // Control interface
    void setTelemetryEnabled(bool enabled) { telemetry_enabled_ = enabled; }
    void setTelemetryRate(uint32_t rate_ms) { telemetry_rate_ms_ = rate_ms; }

private:
    UART_HandleTypeDef* uart_;
    uint8_t system_id_;
    RoboMasterCANManager* can_manager_;
    
    // Motor registry
    RoboMasterMotor* motors_[MAX_MOTORS];
    bool motor_registered_[MAX_MOTORS];
    
    // MAVLink parsing
    mavlink_status_t rx_status_;
    mavlink_message_t rx_msg_;
    
    // Timing
    uint32_t last_heartbeat_;
    uint32_t last_telemetry_;
    uint32_t telemetry_rate_ms_;
    bool telemetry_enabled_;
    
    // Parameter management
    struct ParameterInfo {
        const char* name;
        float* value_ptr;
        float min_value;
        float max_value;
        uint8_t motor_id;
    };
    
    static constexpr uint16_t MAX_PARAMETERS = 64;
    ParameterInfo parameters_[MAX_PARAMETERS];
    uint16_t parameter_count_;
    
    // Message handlers
    void handleHeartbeat(mavlink_message_t* msg);
    void handleParamRequestRead(mavlink_message_t* msg);
    void handleParamRequestList(mavlink_message_t* msg);
    void handleParamSet(mavlink_message_t* msg);
    void handleCommandLong(mavlink_message_t* msg);
    void handleManualControl(mavlink_message_t* msg);
    void handleRCChannelsOverride(mavlink_message_t* msg);
    void handleRequestDataStream(mavlink_message_t* msg);
    
    // Custom RoboMaster messages
    void handleMotorControl(mavlink_message_t* msg);
    void handleMotorConfigSet(mavlink_message_t* msg);
    void handleMotorConfigGet(mavlink_message_t* msg);
    void handleMotorStatusRequest(mavlink_message_t* msg);
    
    // Outgoing messages
    void sendHeartbeat();
    void sendMotorTelemetry();
    void sendMotorStatus(uint8_t motor_id);
    void sendParameterValue(uint16_t param_index);
    void sendParameterValue(const char* param_name, float value);
    void sendCommandAck(uint16_t command, uint8_t result);
    void sendStatusText(uint8_t severity, const char* text);
    
    // Parameter system
    void initializeParameters();
    void registerMotorParameters(uint8_t motor_id);
    int16_t findParameterIndex(const char* param_name) const;
    int16_t findParameterIndex(uint16_t param_index) const;
    bool setParameterValue(uint16_t param_index, float value);
    bool setParameterValue(const char* param_name, float value);
    void saveParametersToFlash();
    void loadParametersFromFlash();
    
    // Configuration management
    void saveMotorConfig(uint8_t motor_id);
    void loadMotorConfig(uint8_t motor_id);
    void resetMotorConfig(uint8_t motor_id);
    
    // Utility functions
    void sendMessage(mavlink_message_t* msg);
    RoboMasterMotor* findMotor(uint8_t motor_id);
    uint32_t getCurrentTimeMs() const;
    
    // Safety and emergency functions
    void emergencyStop();
    void emergencyStopMotor(uint8_t motor_id);
    bool validateMotorCommand(uint8_t motor_id, float value, const char* param_type);
    
    // Custom MAVLink message IDs (using available message slots)
    enum CustomMessageIDs {
        MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL = 180,
        MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS = 181,
        MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG = 182,
        MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY = 183
    };
    
    // Parameter categories for organization
    enum ParameterCategory {
        PARAM_CONTROL = 0,
        PARAM_LIMITS = 1,
        PARAM_SAFETY = 2,
        PARAM_STATUS = 3
    };
};