#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"
#include <algorithm>
#include <cstring>

RoboMasterMotor::RoboMasterMotor() {
    state_.status = RoboMasterStatus::NOT_INITIALIZED;
    state_.enabled = false;
    state_.currentPositionRad = 0.0f;
    state_.targetPositionRad = 0.0f;
    uint32_t currentTime = getCurrentTimeMs();
    state_.lastCommandTime = currentTime;
    state_.lastFeedbackTime = currentTime;  // Initialize feedback time
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
    uint32_t currentTime = getCurrentTimeMs();
    state_.lastCommandTime = currentTime;
    state_.lastFeedbackTime = currentTime;  // Initialize feedback time during init
    last_update_time_ = currentTime;
    
    // Reset control state
    position_integral_ = 0.0f;
    position_derivative_ = 0.0f;
    last_position_error_ = 0.0f;
    velocity_integral_ = 0.0f;
    last_velocity_error_ = 0.0f;
    current_integral_ = 0.0f;
    last_current_error_ = 0.0f;
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
    current_integral_ = 0.0f;
    last_position_error_ = 0.0f;
    last_velocity_error_ = 0.0f;
    last_current_error_ = 0.0f;
    
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
        current_integral_ = 0.0f;
    }

    return RoboMasterStatus::OK;
}

RoboMasterStatus RoboMasterMotor::setInitialPosition(float initialPositionRad) {
    if (!initialized_) {
        return RoboMasterStatus::NOT_INITIALIZED;
    }

    // Check if we have received at least one feedback packet
    if (!zero_set_) {
        return RoboMasterStatus::NOT_INITIALIZED;
    }

    state_.lastCommandTime = getCurrentTimeMs();
    resetWatchdog();

    // Calculate the offset needed to make current position equal to initialPositionRad
    // Current absolute position calculation: (current_raw_position - zero_position_rad_) + (position_wraps_ * 2π) + positionOffsetRad
    // We want this to equal initialPositionRad after applying direction inversion


    float current_raw_position = last_raw_position_;
    float unwrapped_position = current_raw_position - zero_position_rad_;
    unwrapped_position += (static_cast<float>(position_wraps_) * 2.0f * M_PI);

    // Calculate new offset to make current position equal to initialPositionRad
    float desired_position = config_.directionInverted ? -initialPositionRad : initialPositionRad;
    config_.positionOffsetRad = desired_position - unwrapped_position;

    // Update the current position immediately
    state_.currentPositionRad = initialPositionRad;
    state_.targetPositionRad = initialPositionRad;
    zero_position_rad_ = initialPositionRad;  // Update zero position to current raw position

    // Reset control integrators since we're changing the position reference
    position_integral_ = 0.0f;
    velocity_integral_ = 0.0f;
    current_integral_ = 0.0f;
    last_position_error_ = 0.0f;

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

    // Simplified timeout check that's less likely to be optimized incorrectly
    checkTimeoutStatus();

    // If timeout occurred, status will be TIMEOUT and we should not continue
    if (state_.status == RoboMasterStatus::TIMEOUT) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED ON for timeout
        return;  // Exit early if in timeout state
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // LED OFF for normal operation

    // Check safety limits
    checkSafetyLimits();

    if (!state_.enabled) {
        return;
    }

    // Update control loop
    updateControlLoop();

    last_update_time_ = currentTime;
}

// updateControlLoop関数の修正版（位置制御部分）
void RoboMasterMotor::updateControlLoop() {
    uint32_t currentTime = getCurrentTimeMs();

    // Handle potential overflow of HAL_GetTick()
    int64_t deltaTimeMs = static_cast<int64_t>(currentTime) - static_cast<int64_t>(last_update_time_);
    
    if (deltaTimeMs < 0) {
        deltaTimeMs += UINT32_MAX;
    }

    float deltaTimeS = static_cast<float>(deltaTimeMs) / 1000.0f;

    // Sanity check
    if (deltaTimeS <= 0.0f || deltaTimeS > 1.0f) {
        last_update_time_ = currentTime;
        return;
    }

    float targetVelocity = 0.0f;
    int16_t targetCurrent = 0;
    int16_t outputCurrent = 0;

    // [OUTER LOOP] Position PID → Target Velocity
    if (state_.controlMode == RoboMasterControlMode::POSITION) {
        float positionError = state_.targetPositionRad - state_.currentPositionRad;

        // デッドバンドの追加（オプション）
        const float POSITION_DEADBAND = 0.01f;  // 約0.57度
        if (std::abs(positionError) < POSITION_DEADBAND) {
            positionError = 0.0f;
            position_integral_ = 0.0f;  // 積分項もリセット
        }

        // Proportional term
        float pTerm = config_.positionKp * positionError;

        // Integral term with anti-windup（改善版）
        const float INTEGRAL_LIMIT_FACTOR = 0.3f;  // 積分項の制限を緩和
        float maxIntegral = (config_.maxVelocityRPS * INTEGRAL_LIMIT_FACTOR) / (config_.positionKi + 1e-6f);
        
        // 積分項の更新（エラーが小さい時のみ）
        if (std::abs(positionError) < 0.5f) {  // 約28度以内
            position_integral_ += positionError * deltaTimeS;
            position_integral_ = std::max(-maxIntegral, std::min(maxIntegral, position_integral_));
        } else {
            // 大きなエラーの場合は積分項をリセット
            position_integral_ *= 0.95f;  // 徐々に減衰
        }
        float iTerm = config_.positionKi * position_integral_;

        // Derivative term with filtering
        float rawDerivative = (positionError - last_position_error_) / deltaTimeS;
        
        // ローパスフィルタを適用（ノイズ低減）
        static float filtered_derivative = 0.0f;
        const float DERIVATIVE_FILTER_ALPHA = 0.2f;
        filtered_derivative = DERIVATIVE_FILTER_ALPHA * rawDerivative + 
                            (1.0f - DERIVATIVE_FILTER_ALPHA) * filtered_derivative;
        
        float dTerm = config_.positionKd * filtered_derivative;
        last_position_error_ = positionError;

        // Position PID output
        targetVelocity = pTerm + iTerm + dTerm;
        
        // 速度制限（位置制御モードでは少し控えめに）
        const float POSITION_MODE_VELOCITY_FACTOR = 0.8f;
        float maxVel = config_.maxVelocityRPS * POSITION_MODE_VELOCITY_FACTOR;
        targetVelocity = std::max(-maxVel, std::min(maxVel, targetVelocity));

        // Apply acceleration limiting
        applyRateLimiting(targetVelocity, deltaTimeS);
    } else {
        // Use directly commanded velocity for VELOCITY and CURRENT modes
        targetVelocity = state_.targetVelocityRPS;
        if (state_.controlMode == RoboMasterControlMode::VELOCITY) {
            applyRateLimiting(targetVelocity, deltaTimeS);
        }
    }

    // [INNER LOOP] Velocity PID → Target Current
    if (state_.controlMode == RoboMasterControlMode::POSITION ||
        state_.controlMode == RoboMasterControlMode::VELOCITY) {

        float velocityError = targetVelocity - state_.currentVelocityRPS;

        // Proportional term
        float pTerm = config_.velocityKp * velocityError;

        // Integral term with improved anti-windup
        const float VEL_INTEGRAL_LIMIT_FACTOR = 0.5f;
        float maxIntegral = static_cast<float>(config_.maxCurrent) * VEL_INTEGRAL_LIMIT_FACTOR / 
                           (config_.velocityKi + 1e-6f);
        
        velocity_integral_ += velocityError * deltaTimeS;
        velocity_integral_ = std::max(-maxIntegral, std::min(maxIntegral, velocity_integral_));
        float iTerm = config_.velocityKi * velocity_integral_;

        // Derivative term
        float dTerm = config_.velocityKd * (velocityError - last_velocity_error_) / deltaTimeS;
        last_velocity_error_ = velocityError;

        // Velocity PID output
        targetCurrent = static_cast<int16_t>(pTerm + iTerm + dTerm);
    } else {
        // Use directly commanded current for CURRENT mode
        targetCurrent = state_.targetCurrent;
    }

    // [INNERMOST LOOP] Current limiting and direction
    outputCurrent = constrainCurrent(targetCurrent);
    
    // 方向反転の適用
    if (config_.directionInverted) {
        outputCurrent = -outputCurrent;
    }

    // Send final motor command
    sendCurrentCommand(outputCurrent);
    state_.targetCurrent = targetCurrent;
}

void RoboMasterMotor::resetWatchdog() {
    // Reset command timeout (for when user sends commands)
    state_.lastCommandTime = getCurrentTimeMs();
    
    // Note: lastFeedbackTime is only reset when actual CAN feedback is received
    // in processCANData(). This ensures timeout detection works correctly.
    
    if (state_.status == RoboMasterStatus::TIMEOUT) {
        state_.status = RoboMasterStatus::OK;
    }
}

void RoboMasterMotor::processCANData(const uint8_t* data, uint8_t length) {
    if (length < 8) {
        return;
    }
    
    // Update feedback reception timestamp - this is critical for timeout detection
    state_.lastFeedbackTime = getCurrentTimeMs();
    
    // Parse RoboMaster feedback data format
    int16_t raw_angle = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t raw_speed = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t raw_current = (static_cast<int16_t>(data[4]) << 8) | data[5];
    uint8_t temperature = data[6];
    
    // Convert raw angle to radians (0-8191 -> 0-2π)
    float current_raw_position = static_cast<float>(raw_angle) * RAD_PER_COUNT;
    
    // Initialize zero position on first call
    if (!zero_set_) {
        zero_position_rad_ = current_raw_position;
        zero_set_ = true;
        last_raw_position_ = current_raw_position;
        absolute_position_rad_ = 0.0f;  // Start at zero
        position_wraps_ = 0;
        setInitialPosition(1.11903906f);
    } else {
        // Improved wrap detection with hysteresis
        float position_delta = current_raw_position - last_raw_position_;
        
        // Check for wrap-around (threshold at ±π with some margin)
        const float WRAP_THRESHOLD = M_PI * 1.5f;  // 270 degrees
        
        if (position_delta > WRAP_THRESHOLD) {
            // Wrapped from ~2π to ~0 (counter-clockwise)
            position_wraps_--;
        } else if (position_delta < -WRAP_THRESHOLD) {
            // Wrapped from ~0 to ~2π (clockwise)
            position_wraps_++;
        }
        
        last_raw_position_ = current_raw_position;
    }
    
    // Calculate absolute position with proper wrapping
    float unwrapped_position = current_raw_position - zero_position_rad_;
    unwrapped_position += (static_cast<float>(position_wraps_) * 2.0f * M_PI);
    
    // Apply offset AFTER unwrapping
    unwrapped_position += config_.positionOffsetRad;
    
    // Apply direction inversion
    if (config_.directionInverted) {
        unwrapped_position = -unwrapped_position;
    }
    
    state_.currentPositionRad = unwrapped_position;
    
    // Convert velocity (RPM to RPS)
    float velocity = static_cast<float>(raw_speed) / 60.0f;
    if (config_.directionInverted) {
        velocity = -velocity;
    }
    state_.currentVelocityRPS = velocity;
    
    // Current and temperature
    state_.currentMilliamps = config_.directionInverted ? -raw_current : raw_current;
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
        
        // Debug: Flash LED rapidly when timeout occurs
        for (int i = 0; i < 10; i++) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(50);
        }
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

void RoboMasterMotor::checkTimeoutStatus() {
    uint32_t currentTime = getCurrentTimeMs();
    
    // Check for timeout based on feedback reception time
    if (currentTime - state_.lastFeedbackTime > config_.watchdogTimeoutMs) {
        if (state_.status != RoboMasterStatus::TIMEOUT) {
            handleTimeout();
        }
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
    } else if (strcmp(param_name, "currentKp") == 0) {
        config_.currentKp = value;
    } else if (strcmp(param_name, "currentKi") == 0) {
        config_.currentKi = value;
    } else if (strcmp(param_name, "currentKd") == 0) {
        config_.currentKd = value;
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
    } else if (strcmp(param_name, "initialPositionRad") == 0) {
        config_.initialPositionRad = value;
    } else {
        return RoboMasterStatus::CONFIG_ERROR;
    }
    
    // Reset control integrators when PID gains change
    if (strstr(param_name, "Kp") || strstr(param_name, "Ki") || strstr(param_name, "Kd")) {
        position_integral_ = 0.0f;
        velocity_integral_ = 0.0f;
        current_integral_ = 0.0f;
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
    } else if (strcmp(param_name, "currentKp") == 0) {
        return config_.currentKp;
    } else if (strcmp(param_name, "currentKi") == 0) {
        return config_.currentKi;
    } else if (strcmp(param_name, "currentKd") == 0) {
        return config_.currentKd;
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
    } else if (strcmp(param_name, "initialPositionRad") == 0) {
        return config_.initialPositionRad;
    }

    return 0.0f;  // Parameter not found
}