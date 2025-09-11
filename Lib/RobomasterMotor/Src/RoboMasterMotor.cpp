#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"
#include <algorithm>
#include <cstring>

RoboMasterMotor::RoboMasterMotor() {
    state_.status = RoboMasterStatus::NOT_INITIALIZED;
    state_.enabled = false;
    state_.currentPositionRad = 0.0f;
    state_.targetPositionRad = 0.0f;
    state_.lastCommandTime = getCurrentTimeMs();
}

RoboMasterStatus RoboMasterMotor::create(uint8_t motor_id, RoboMasterCANManager* can_manager) {
    if (can_manager == nullptr) {
        return RoboMasterStatus::CAN_ERROR;
    }
    
    if (motor_id < 1 || motor_id > 8) {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    motor_id_ = motor_id;
    can_manager_ = can_manager;
    
    return RoboMasterStatus::OK;
}

RoboMasterStatus RoboMasterMotor::init() {
    return init(RoboMasterConfig{});
}

RoboMasterStatus RoboMasterMotor::init(const RoboMasterConfig& config) {
    if (can_manager_ == nullptr) {
        state_.status = RoboMasterStatus::CAN_ERROR;
        return state_.status;
    }
    
    RoboMasterStatus configStatus = validateConfig(config);
    if (configStatus != RoboMasterStatus::OK) {
        state_.status = configStatus;
        return state_.status;
    }
    
    config_ = config;
    
    // Register with CAN manager
    if (can_manager_->registerMotor(this, motor_id_) != CANManagerStatus::OK) {
        state_.status = RoboMasterStatus::CAN_ERROR;
        return state_.status;
    }
    
    // Initialize state
    initialized_ = true;
    state_.enabled = !config_.startDisabled;
    state_.controlMode = config_.startupMode;
    state_.targetPositionRad = config_.startupPositionRad;
    state_.lastCommandTime = getCurrentTimeMs();
    last_update_time_ = state_.lastCommandTime;
    
    // Reset control state
    position_integral_ = 0.0f;
    position_derivative_ = 0.0f;
    last_position_error_ = 0.0f;
    velocity_integral_ = 0.0f;
    last_velocity_error_ = 0.0f;
    last_target_velocity_ = 0.0f;
    
    state_.status = RoboMasterStatus::OK;
    return state_.status;
}

RoboMasterStatus RoboMasterMotor::setPositionRad(float positionRad) {
    if (!initialized_) {
        return RoboMasterStatus::NOT_INITIALIZED;
    }
    
    state_.lastCommandTime = getCurrentTimeMs();
    resetWatchdog();
    
    float constrainedPosition = constrainPosition(positionRad);
    if (std::abs(constrainedPosition - positionRad) > 0.01f) {
        state_.saturationCount++;
    }
    
    state_.targetPositionRad = constrainedPosition;
    
    if (!state_.enabled) {
        return RoboMasterStatus::OK;
    }
    
    return RoboMasterStatus::OK;
}

RoboMasterStatus RoboMasterMotor::setVelocityRPS(float velocityRPS) {
    if (!initialized_) {
        return RoboMasterStatus::NOT_INITIALIZED;
    }
    
    state_.lastCommandTime = getCurrentTimeMs();
    resetWatchdog();
    
    float constrainedVelocity = constrainVelocity(velocityRPS);
    if (std::abs(constrainedVelocity - velocityRPS) > 0.01f) {
        state_.saturationCount++;
    }
    
    state_.targetVelocityRPS = constrainedVelocity;
    
    if (!state_.enabled) {
        return RoboMasterStatus::OK;
    }
    
    return RoboMasterStatus::OK;
}

RoboMasterStatus RoboMasterMotor::setCurrent(int16_t current) {
    if (!initialized_) {
        return RoboMasterStatus::NOT_INITIALIZED;
    }
    
    state_.lastCommandTime = getCurrentTimeMs();
    resetWatchdog();
    
    int16_t constrainedCurrent = constrainCurrent(current);
    if (constrainedCurrent != current) {
        state_.saturationCount++;
    }
    
    state_.targetCurrent = constrainedCurrent;
    
    if (!state_.enabled) {
        sendCurrentCommand(0);
        return RoboMasterStatus::OK;
    }
    
    sendCurrentCommand(constrainedCurrent);
    return RoboMasterStatus::OK;
}

RoboMasterStatus RoboMasterMotor::setControlMode(RoboMasterControlMode mode) {
    if (!initialized_) {
        return RoboMasterStatus::NOT_INITIALIZED;
    }
    
    state_.controlMode = mode;
    state_.lastCommandTime = getCurrentTimeMs();
    
    // Reset integrators when changing modes
    position_integral_ = 0.0f;
    velocity_integral_ = 0.0f;
    last_position_error_ = 0.0f;
    last_velocity_error_ = 0.0f;
    
    return RoboMasterStatus::OK;
}

RoboMasterStatus RoboMasterMotor::setEnabled(bool enabled) {
    if (!initialized_) {
        return RoboMasterStatus::NOT_INITIALIZED;
    }
    
    state_.enabled = enabled;
    state_.lastCommandTime = getCurrentTimeMs();
    
    if (!enabled) {
        sendCurrentCommand(0);
        // Reset control state
        position_integral_ = 0.0f;
        velocity_integral_ = 0.0f;
    }
    
    return RoboMasterStatus::OK;
}

RoboMasterStatus RoboMasterMotor::setConfig(const RoboMasterConfig& config) {
    RoboMasterStatus status = validateConfig(config);
    if (status != RoboMasterStatus::OK) {
        return status;
    }
    
    config_ = config;
    return RoboMasterStatus::OK;
}

RoboMasterLimits RoboMasterMotor::getLimits() const {
    RoboMasterLimits limits;
    limits.maxVelocityRPS = config_.maxVelocityRPS;
    limits.maxAccelerationRPS2 = config_.maxAccelerationRPS2;
    limits.maxCurrent = config_.maxCurrent;
    limits.minCurrent = config_.minCurrent;
    limits.minPositionRad = config_.minPositionRad;
    limits.maxPositionRad = config_.maxPositionRad;
    limits.maxTemperature = config_.maxTemperature;
    return limits;
}

void RoboMasterMotor::update() {
    if (!initialized_) {
        return;
    }
    
    uint32_t currentTime = getCurrentTimeMs();
    
    // Check for timeout
    if (currentTime - state_.lastCommandTime > config_.watchdogTimeoutMs) {
        handleTimeout();
        return;
    }
    
    // Check safety limits
    checkSafetyLimits();
    
    if (!state_.enabled) {
        return;
    }
    
    // Update control loop
    updateControlLoop();
    
    last_update_time_ = currentTime;
}

void RoboMasterMotor::updateControlLoop() {
    uint32_t currentTime = getCurrentTimeMs();
    
    // Handle potential overflow of HAL_GetTick() (wraps every ~49.7 days)
    int64_t deltaTimeMs = static_cast<int64_t>(currentTime) - static_cast<int64_t>(last_update_time_);
    
    // Handle overflow case (assumes updates happen more frequently than 24 days)
    if (deltaTimeMs < 0) {
        deltaTimeMs += UINT32_MAX;
    }
    
    float deltaTimeS = static_cast<float>(deltaTimeMs) / 1000.0f;
    
    // Sanity check: ignore extremely large deltas (likely due to initialization or overflow)
    if (deltaTimeS <= 0.0f || deltaTimeS > 1.0f) {
        last_update_time_ = currentTime;
        return;
    }
    
    int16_t outputCurrent = 0;
    
    switch (state_.controlMode) {
        case RoboMasterControlMode::POSITION: {
            // Position control with PID
            float positionError = state_.targetPositionRad - state_.currentPositionRad;
            
            // Proportional term
            float pTerm = config_.positionKp * positionError;
            
            // Integral term
            position_integral_ += positionError * deltaTimeS;
            // Anti-windup
            float maxIntegral = static_cast<float>(config_.maxCurrent) / (config_.positionKi + 1e-6f);
            position_integral_ = std::max(-maxIntegral, std::min(maxIntegral, position_integral_));
            float iTerm = config_.positionKi * position_integral_;
            
            // Derivative term
            float dTerm = config_.positionKd * (positionError - last_position_error_) / deltaTimeS;
            last_position_error_ = positionError;
            
            // Convert to velocity command (position PID output becomes velocity setpoint)
            float targetVelocity = pTerm + iTerm + dTerm;
            targetVelocity = constrainVelocity(targetVelocity);
            
            // Apply acceleration limiting
            applyRateLimiting(targetVelocity, deltaTimeS);
            
            // Velocity control as inner loop
            float velocityError = targetVelocity - state_.currentVelocityRPS;
            velocity_integral_ += velocityError * deltaTimeS;
            
            // Anti-windup for velocity integral
            maxIntegral = static_cast<float>(config_.maxCurrent) / (config_.velocityKi + 1e-6f);
            velocity_integral_ = std::max(-maxIntegral, std::min(maxIntegral, velocity_integral_));
            
            float velocityOutput = config_.velocityKp * velocityError + config_.velocityKi * velocity_integral_;
            outputCurrent = static_cast<int16_t>(constrainCurrent(static_cast<int16_t>(velocityOutput)));
            
            break;
        }
        
        case RoboMasterControlMode::VELOCITY: {
            // Velocity control with PI
            float targetVel = state_.targetVelocityRPS;
            applyRateLimiting(targetVel, deltaTimeS);
            
            float velocityError = targetVel - state_.currentVelocityRPS;
            
            // Proportional term
            float pTerm = config_.velocityKp * velocityError;
            
            // Integral term with anti-windup
            velocity_integral_ += velocityError * deltaTimeS;
            float maxIntegral = static_cast<float>(config_.maxCurrent) / (config_.velocityKi + 1e-6f);
            velocity_integral_ = std::max(-maxIntegral, std::min(maxIntegral, velocity_integral_));
            float iTerm = config_.velocityKi * velocity_integral_;
            
            outputCurrent = static_cast<int16_t>(constrainCurrent(static_cast<int16_t>(pTerm + iTerm)));
            
            break;
        }
        
        case RoboMasterControlMode::CURRENT:
        default:
            // Direct current control
            outputCurrent = constrainCurrent(state_.targetCurrent);
            break;
    }
    
    // Send current command
    sendCurrentCommand(outputCurrent);
    state_.targetCurrent = outputCurrent;
}

void RoboMasterMotor::resetWatchdog() {
    state_.lastCommandTime = getCurrentTimeMs();
    
    if (state_.status == RoboMasterStatus::TIMEOUT) {
        state_.status = RoboMasterStatus::OK;
    }
}

void RoboMasterMotor::processCANData(const uint8_t* data, uint8_t length) {
    if (length < 8) {
        return;
    }
    
    // Parse RoboMaster feedback data format
    // Bytes 0-1: Rotor angle (0-8191)
    // Bytes 2-3: Rotor speed (RPM)
    // Bytes 4-5: Torque current (mA)
    // Byte 6: Motor temperature (°C)
    // Byte 7: Reserved
    
    int16_t raw_angle = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t raw_speed = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t raw_current = (static_cast<int16_t>(data[4]) << 8) | data[5];
    uint8_t temperature = data[6];
    
    // Convert raw angle to radians
    float current_raw_position = static_cast<float>(raw_angle) * RAD_PER_COUNT;
    
    // Handle position wrapping and multi-turn tracking
    if (!zero_set_) {
        zero_position_rad_ = current_raw_position - config_.startupPositionRad - config_.positionOffsetRad;
        zero_set_ = true;
        last_raw_position_ = current_raw_position;
    }
    
    // Detect wrapping
    float position_delta = current_raw_position - last_raw_position_;
    if (position_delta > M_PI) {
        position_wraps_--;
    } else if (position_delta < -M_PI) {
        position_wraps_++;
    }
    last_raw_position_ = current_raw_position;
    
    // Calculate absolute position
    absolute_position_rad_ = current_raw_position + (static_cast<float>(position_wraps_) * 2.0f * M_PI);
    
    // Apply direction inversion and offset
    float position = absolute_position_rad_ - zero_position_rad_;
    if (config_.directionInverted) {
        position = -position;
    }
    state_.currentPositionRad = position;
    
    // Convert velocity (RPM to RPS)
    float velocity = static_cast<float>(raw_speed) / 60.0f;
    if (config_.directionInverted) {
        velocity = -velocity;
    }
    state_.currentVelocityRPS = velocity;
    
    // Current and temperature
    state_.currentMilliamps = raw_current;
    state_.temperatureCelsius = temperature;
}

RoboMasterStatus RoboMasterMotor::validateConfig(const RoboMasterConfig& config) const {
    if (config.maxVelocityRPS <= 0 || config.maxAccelerationRPS2 <= 0) {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    if (config.maxCurrent <= config.minCurrent) {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    if (config.positionLimitsEnabled && config.maxPositionRad <= config.minPositionRad) {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    if (config.maxTemperature < 40 || config.maxTemperature > 100) {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    return RoboMasterStatus::OK;
}

float RoboMasterMotor::constrainPosition(float positionRad) const {
    if (!config_.positionLimitsEnabled) {
        return positionRad;
    }
    
    return std::max(config_.minPositionRad, std::min(config_.maxPositionRad, positionRad));
}

float RoboMasterMotor::constrainVelocity(float velocityRPS) const {
    return std::max(-config_.maxVelocityRPS, std::min(config_.maxVelocityRPS, velocityRPS));
}

int16_t RoboMasterMotor::constrainCurrent(int16_t current) const {
    return std::max(config_.minCurrent, std::min(config_.maxCurrent, current));
}

void RoboMasterMotor::applyRateLimiting(float& targetVelocity, float deltaTimeS) {
    float maxVelocityChange = config_.maxAccelerationRPS2 * deltaTimeS;
    float velocityDiff = targetVelocity - last_target_velocity_;
    
    if (std::abs(velocityDiff) > maxVelocityChange) {
        if (velocityDiff > 0) {
            targetVelocity = last_target_velocity_ + maxVelocityChange;
        } else {
            targetVelocity = last_target_velocity_ - maxVelocityChange;
        }
    }
    
    last_target_velocity_ = targetVelocity;
}

void RoboMasterMotor::handleTimeout() {
    if (state_.status != RoboMasterStatus::TIMEOUT) {
        state_.status = RoboMasterStatus::TIMEOUT;
        state_.timeoutCount++;
        applyFailSafe();
    }
}

void RoboMasterMotor::applyFailSafe() {
    switch (config_.failSafeBehavior) {
        case RoboMasterFailSafeBehavior::HOLD_POSITION:
            // Keep current position as target
            state_.targetPositionRad = state_.currentPositionRad;
            break;
            
        case RoboMasterFailSafeBehavior::BRAKE:
            // Set velocity target to zero
            state_.targetVelocityRPS = 0.0f;
            break;
            
        case RoboMasterFailSafeBehavior::DISABLE_OUTPUT:
            state_.enabled = false;
            sendCurrentCommand(0);
            break;
    }
}

void RoboMasterMotor::checkSafetyLimits() {
    // Temperature check
    if (state_.temperatureCelsius > config_.maxTemperature) {
        if (state_.status != RoboMasterStatus::OVERHEAT) {
            state_.status = RoboMasterStatus::OVERHEAT;
            state_.overHeatCount++;
            applyFailSafe();
        }
    }
    
    // Current check
    if (std::abs(state_.currentMilliamps) > config_.maxCurrent) {
        if (state_.status != RoboMasterStatus::OVERCURRENT) {
            state_.status = RoboMasterStatus::OVERCURRENT;
            state_.errorCount++;
        }
    }
}

uint32_t RoboMasterMotor::getCurrentTimeMs() const {
    return HAL_GetTick();
}

void RoboMasterMotor::sendCurrentCommand(int16_t current) {
    if (can_manager_ != nullptr && initialized_) {
        can_manager_->sendCurrentCommand(motor_id_, current);
    }
}

RoboMasterStatus RoboMasterMotor::updateParameter(const char* param_name, float value) {
    if (!initialized_ || param_name == nullptr) {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    // Update configuration parameters based on name
    if (strcmp(param_name, "positionKp") == 0) {
        config_.positionKp = value;
    } else if (strcmp(param_name, "positionKi") == 0) {
        config_.positionKi = value;
    } else if (strcmp(param_name, "positionKd") == 0) {
        config_.positionKd = value;
    } else if (strcmp(param_name, "velocityKp") == 0) {
        config_.velocityKp = value;
    } else if (strcmp(param_name, "velocityKi") == 0) {
        config_.velocityKi = value;
    } else if (strcmp(param_name, "velocityKd") == 0) {
        config_.velocityKd = value;
    } else if (strcmp(param_name, "maxVelocityRPS") == 0) {
        config_.maxVelocityRPS = value;
    } else if (strcmp(param_name, "maxAccelerationRPS2") == 0) {
        config_.maxAccelerationRPS2 = value;
    } else if (strcmp(param_name, "maxCurrent") == 0) {
        config_.maxCurrent = static_cast<int16_t>(value);
    } else if (strcmp(param_name, "minCurrent") == 0) {
        config_.minCurrent = static_cast<int16_t>(value);
    } else if (strcmp(param_name, "maxTemperature") == 0) {
        config_.maxTemperature = static_cast<uint8_t>(value);
    } else if (strcmp(param_name, "watchdogTimeoutMs") == 0) {
        config_.watchdogTimeoutMs = static_cast<uint32_t>(value);
    } else if (strcmp(param_name, "minPositionRad") == 0) {
        config_.minPositionRad = value;
    } else if (strcmp(param_name, "maxPositionRad") == 0) {
        config_.maxPositionRad = value;
    } else if (strcmp(param_name, "positionLimitsEnabled") == 0) {
        config_.positionLimitsEnabled = (value > 0.5f);
    } else if (strcmp(param_name, "directionInverted") == 0) {
        config_.directionInverted = (value > 0.5f);
    } else if (strcmp(param_name, "positionOffsetRad") == 0) {
        config_.positionOffsetRad = value;
    } else {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    // Reset control integrators when PID gains change
    if (strstr(param_name, "Kp") || strstr(param_name, "Ki") || strstr(param_name, "Kd")) {
        position_integral_ = 0.0f;
        velocity_integral_ = 0.0f;
    }
    
    return RoboMasterStatus::OK;
}

float RoboMasterMotor::getParameter(const char* param_name) const {
    if (param_name == nullptr) {
        return 0.0f;
    }
    
    if (strcmp(param_name, "positionKp") == 0) {
        return config_.positionKp;
    } else if (strcmp(param_name, "positionKi") == 0) {
        return config_.positionKi;
    } else if (strcmp(param_name, "positionKd") == 0) {
        return config_.positionKd;
    } else if (strcmp(param_name, "velocityKp") == 0) {
        return config_.velocityKp;
    } else if (strcmp(param_name, "velocityKi") == 0) {
        return config_.velocityKi;
    } else if (strcmp(param_name, "velocityKd") == 0) {
        return config_.velocityKd;
    } else if (strcmp(param_name, "maxVelocityRPS") == 0) {
        return config_.maxVelocityRPS;
    } else if (strcmp(param_name, "maxAccelerationRPS2") == 0) {
        return config_.maxAccelerationRPS2;
    } else if (strcmp(param_name, "maxCurrent") == 0) {
        return static_cast<float>(config_.maxCurrent);
    } else if (strcmp(param_name, "minCurrent") == 0) {
        return static_cast<float>(config_.minCurrent);
    } else if (strcmp(param_name, "maxTemperature") == 0) {
        return static_cast<float>(config_.maxTemperature);
    } else if (strcmp(param_name, "watchdogTimeoutMs") == 0) {
        return static_cast<float>(config_.watchdogTimeoutMs);
    } else if (strcmp(param_name, "minPositionRad") == 0) {
        return config_.minPositionRad;
    } else if (strcmp(param_name, "maxPositionRad") == 0) {
        return config_.maxPositionRad;
    } else if (strcmp(param_name, "positionLimitsEnabled") == 0) {
        return config_.positionLimitsEnabled ? 1.0f : 0.0f;
    } else if (strcmp(param_name, "directionInverted") == 0) {
        return config_.directionInverted ? 1.0f : 0.0f;
    } else if (strcmp(param_name, "positionOffsetRad") == 0) {
        return config_.positionOffsetRad;
    }
    
    return 0.0f;  // Parameter not found
}