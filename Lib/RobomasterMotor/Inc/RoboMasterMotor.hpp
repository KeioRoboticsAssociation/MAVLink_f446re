#pragma once

#include "stm32f4xx_hal.h"
#include <cstdint>
#include <cmath>

enum class RoboMasterStatus {
    OK = 0,
    NOT_INITIALIZED,
    CAN_ERROR,
    OUT_OF_RANGE,
    TIMEOUT,
    CONFIG_ERROR,
    OVERHEAT,
    OVERCURRENT
};

enum class RoboMasterControlMode {
    CURRENT = 0,
    VELOCITY = 1,
    POSITION = 2
};

enum class RoboMasterFailSafeBehavior {
    HOLD_POSITION = 0,
    BRAKE = 1,
    DISABLE_OUTPUT = 2
};

struct RoboMasterConfig {
    // Physical limits
    float maxVelocityRPS = 100.0f;           // Max velocity in RPS
    float maxAccelerationRPS2 = 200.0f;     // Max acceleration in RPS^2
    int16_t maxCurrent = 16000;              // Max current in mA
    int16_t minCurrent = -16000;             // Min current in mA
    
    // Position limits
    float minPositionRad = -100.0f * M_PI;   // Min position in radians
    float maxPositionRad = 100.0f * M_PI;    // Max position in radians
    bool positionLimitsEnabled = false;      // Enable position limits
    
    // Safety settings
    uint32_t watchdogTimeoutMs = 1000;       // Watchdog timeout
    uint8_t maxTemperature = 80;             // Max temperature in Celsius
    RoboMasterFailSafeBehavior failSafeBehavior = RoboMasterFailSafeBehavior::BRAKE;
    
    // Control parameters
    float positionKp = 10.0f;                // Position P gain
    float positionKi = 0.0f;                 // Position I gain
    float positionKd = 0.1f;                 // Position D gain
    float velocityKp = 35.0f;                // Velocity P gain
    float velocityKi = 0.15f;                // Velocity I gain
    float velocityKd = 0.0f;                 // Velocity D gain
    
    // Startup settings
    float startupPositionRad = 0.0f;         // Initial position
    RoboMasterControlMode startupMode = RoboMasterControlMode::VELOCITY;
    bool startDisabled = false;              // Start in disabled state
    
    // Direction and offset
    bool directionInverted = false;          // Invert rotation direction
    float positionOffsetRad = 0.0f;          // Position offset
};

struct RoboMasterState {
    // Current measurements
    float currentPositionRad = 0.0f;         // Current position in radians
    float currentVelocityRPS = 0.0f;         // Current velocity in RPS
    int16_t currentMilliamps = 0;            // Current in mA
    uint8_t temperatureCelsius = 0;          // Temperature in Celsius
    
    // Target values
    float targetPositionRad = 0.0f;          // Target position
    float targetVelocityRPS = 0.0f;          // Target velocity
    int16_t targetCurrent = 0;               // Target current
    
    // Control state
    RoboMasterControlMode controlMode = RoboMasterControlMode::VELOCITY;
    bool enabled = false;                    // Motor enabled state
    RoboMasterStatus status = RoboMasterStatus::NOT_INITIALIZED;
    
    // Statistics
    uint32_t lastCommandTime = 0;            // Last command timestamp
    uint32_t saturationCount = 0;            // Number of saturations
    uint32_t timeoutCount = 0;               // Number of timeouts
    uint32_t errorCount = 0;                 // Number of errors
    uint32_t overHeatCount = 0;              // Number of overheats
};

struct RoboMasterLimits {
    float maxVelocityRPS;
    float maxAccelerationRPS2;
    int16_t maxCurrent;
    int16_t minCurrent;
    float minPositionRad;
    float maxPositionRad;
    uint8_t maxTemperature;
};

// Forward declaration for CAN manager
class RoboMasterCANManager;

class RoboMasterMotor {
public:
    RoboMasterMotor();
    ~RoboMasterMotor() = default;

    // Initialization
    RoboMasterStatus create(uint8_t motor_id, RoboMasterCANManager* can_manager);
    RoboMasterStatus init();
    RoboMasterStatus init(const RoboMasterConfig& config);

    // Control commands
    RoboMasterStatus setPositionRad(float positionRad);
    RoboMasterStatus setVelocityRPS(float velocityRPS);
    RoboMasterStatus setCurrent(int16_t current);
    RoboMasterStatus setControlMode(RoboMasterControlMode mode);
    RoboMasterStatus setEnabled(bool enabled);

    // Configuration
    RoboMasterStatus setConfig(const RoboMasterConfig& config);
    RoboMasterStatus updateConfig(const RoboMasterConfig& config);
    RoboMasterStatus loadConfigFromJson(const char* jsonString);
    RoboMasterStatus loadConfigFromFile(const char* filePath);
    RoboMasterStatus saveConfigToFile(const char* filePath) const;
    
    // Parameter access for MAVLink integration
    const RoboMasterConfig& getConfig() const { return config_; }
    RoboMasterConfig& getConfigReference() { return config_; }
    RoboMasterStatus updateParameter(const char* param_name, float value);
    float getParameter(const char* param_name) const;

    // Getters
    uint8_t getId() const { return motor_id_; }
    float getPositionCmd() const { return state_.targetPositionRad; }
    float getVelocityCmd() const { return state_.targetVelocityRPS; }
    int16_t getCurrentCmd() const { return state_.targetCurrent; }
    
    float getCurrentPosition() const { return state_.currentPositionRad; }
    float getCurrentVelocity() const { return state_.currentVelocityRPS; }
    int16_t getCurrentMilliamps() const { return state_.currentMilliamps; }
    uint8_t getTemperature() const { return state_.temperatureCelsius; }
    
    RoboMasterLimits getLimits() const;
    RoboMasterStatus getStatus() const { return state_.status; }
    const RoboMasterState& getState() const { return state_; }
    RoboMasterControlMode getControlMode() const { return state_.controlMode; }

    // Update and maintenance
    void update();
    void resetWatchdog();
    
    // CAN data processing (called by CAN manager)
    void processCANData(const uint8_t* data, uint8_t length);

private:
    uint8_t motor_id_ = 0;
    RoboMasterCANManager* can_manager_ = nullptr;
    bool initialized_ = false;
    bool zero_set_ = false;
    
    RoboMasterConfig config_;
    RoboMasterState state_;
    
    // Position tracking
    float absolute_position_rad_ = 0.0f;     // Raw encoder position
    float zero_position_rad_ = 0.0f;         // Zero reference position
    int32_t position_wraps_ = 0;             // Number of full rotations
    float last_raw_position_ = 0.0f;         // For wrap detection
    
    // Control state
    float position_integral_ = 0.0f;         // Position integral term
    float position_derivative_ = 0.0f;       // Position derivative term
    float last_position_error_ = 0.0f;       // Last position error
    float velocity_integral_ = 0.0f;         // Velocity integral term
    float last_velocity_error_ = 0.0f;       // Last velocity error
    uint32_t last_update_time_ = 0;          // Last update timestamp
    
    // Rate limiting
    float last_target_velocity_ = 0.0f;      // For acceleration limiting
    
    // Internal methods
    RoboMasterStatus validateConfig(const RoboMasterConfig& config) const;
    float constrainPosition(float positionRad) const;
    float constrainVelocity(float velocityRPS) const;
    int16_t constrainCurrent(int16_t current) const;
    void applyRateLimiting(float& targetVelocity, float deltaTimeS);
    void updateControlLoop();
    void handleTimeout();
    void applyFailSafe();
    void checkSafetyLimits();
    uint32_t getCurrentTimeMs() const;
    void sendCurrentCommand(int16_t current);
    
    // Constants
    static constexpr float ENCODER_RESOLUTION = 8192.0f;  // 13-bit encoder
    static constexpr float RAD_PER_COUNT = (2.0f * M_PI) / ENCODER_RESOLUTION;
};