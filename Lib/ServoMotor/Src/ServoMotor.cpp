#include "ServoMotor.hpp"
#include "ServoConfig.hpp"

ServoMotor::ServoMotor() {
    state_.status = ServoStatus::NOT_INITIALIZED;
    state_.enabled = false;
    state_.currentAngleDeg = 0.0f;
    state_.targetAngleDeg = 0.0f;
    // Initialize pulse to neutral position from default config
    state_.currentPulseUs = ServoConfig{}.pulseNeutralUs;
    state_.lastCommandTime = getCurrentTimeMs();
}

ServoStatus ServoMotor::create(uint8_t id, TIM_HandleTypeDef* htim, uint32_t channel) {
    if (htim == nullptr) {
        return ServoStatus::TIMER_ERROR;
    }

    id_ = id;
    htim_ = htim;
    channel_ = channel;
    
    return ServoStatus::OK;
}

ServoStatus ServoMotor::init() {
    return init(ServoConfig{});
}

ServoStatus ServoMotor::init(const ServoConfig& config) {
    if (htim_ == nullptr) {
        state_.status = ServoStatus::TIMER_ERROR;
        return state_.status;
    }

    ServoStatus configStatus = validateConfig(config);
    if (configStatus != ServoStatus::OK) {
        state_.status = configStatus;
        return state_.status;
    }

    config_ = config;

    ServoStatus timerStatus = retrieveTimerInfo();
    if (timerStatus != ServoStatus::OK) {
        state_.status = timerStatus;
        return state_.status;
    }

    if (HAL_TIM_PWM_Start(htim_, channel_) != HAL_OK) {
        state_.status = ServoStatus::TIMER_ERROR;
        return state_.status;
    }

    initialized_ = true;
    state_.enabled = !config_.startDisabled;
    state_.targetAngleDeg = config_.startupAngleDeg;
    state_.currentAngleDeg = config_.startupAngleDeg;
    state_.lastCommandTime = getCurrentTimeMs();
    lastUpdateTime_ = state_.lastCommandTime;
    lastAngleDeg_ = state_.currentAngleDeg;

    // Always calculate the correct startup pulse based on configuration
    uint16_t startupPulse = static_cast<uint16_t>(angleToPulse(config_.startupAngleDeg));
    
    if (state_.enabled) {
        updatePWM(startupPulse);
        state_.currentPulseUs = startupPulse;
    } else {
        updatePWM(0);
        // Even when disabled, store the correct pulse for state reporting
        state_.currentPulseUs = startupPulse;
    }

    state_.status = ServoStatus::OK;
    return state_.status;
}

ServoStatus ServoMotor::setAngleDeg(float angleDeg) {
    if (!initialized_) {
        return ServoStatus::NOT_INITIALIZED;
    }

    state_.lastCommandTime = getCurrentTimeMs();
    resetWatchdog();

    float constrainedAngle = constrainAngle(angleDeg);
    if (constrainedAngle != angleDeg) {
        state_.saturationCount++;
    }

    state_.targetAngleDeg = constrainedAngle;
    
    if (!state_.enabled) {
        return ServoStatus::OK;
    }

    return ServoStatus::OK;
}

ServoStatus ServoMotor::setAngleRad(float angleRad) {
    float angleDeg = angleRad * 180.0f / M_PI;
    return setAngleDeg(angleDeg);
}

ServoStatus ServoMotor::setPulseUs(uint16_t pulseUs) {
    if (!initialized_) {
        return ServoStatus::NOT_INITIALIZED;
    }

    state_.lastCommandTime = getCurrentTimeMs();
    resetWatchdog();

    uint16_t constrainedPulse = constrainPulse(pulseUs);
    if (constrainedPulse != pulseUs) {
        state_.saturationCount++;
    }

    float targetAngle = pulseToAngle(constrainedPulse);
    state_.targetAngleDeg = targetAngle;

    if (!state_.enabled) {
        return ServoStatus::OK;
    }

    return ServoStatus::OK;
}

ServoStatus ServoMotor::setEnabled(bool enabled) {
    if (!initialized_) {
        return ServoStatus::NOT_INITIALIZED;
    }

    state_.enabled = enabled;
    state_.lastCommandTime = getCurrentTimeMs();

    if (!enabled) {
        updatePWM(0);
        state_.currentPulseUs = 0;
    }

    return ServoStatus::OK;
}

ServoStatus ServoMotor::setConfig(const ServoConfig& config) {
    ServoStatus status = validateConfig(config);
    if (status != ServoStatus::OK) {
        return status;
    }

    config_ = config;
    return ServoStatus::OK;
}

ServoStatus ServoMotor::updateConfig(const ServoConfig& config) {
    return setConfig(config);
}

ServoStatus ServoMotor::loadConfigFromJson(const char* jsonString) {
    ServoConfig newConfig;
    ServoStatus status = ServoConfigParser::parseFromJson(jsonString, newConfig);
    if (status != ServoStatus::OK) {
        return status;
    }
    return setConfig(newConfig);
}

ServoStatus ServoMotor::loadConfigFromFile(const char* filePath) {
    ServoConfig newConfig;
    ServoStatus status = ServoConfigParser::loadFromFile(filePath, newConfig);
    if (status != ServoStatus::OK) {
        return status;
    }
    return setConfig(newConfig);
}

ServoStatus ServoMotor::loadConfigFromFileForId(const char* filePath) {
    ServoConfig newConfig;
    ServoStatus status = ServoConfigParser::loadFromFileForId(filePath, id_, newConfig);
    if (status != ServoStatus::OK) {
        return status;
    }
    return setConfig(newConfig);
}

ServoStatus ServoMotor::saveConfigToFile(const char* filePath) const {
    return ServoConfigParser::saveToFile(filePath, config_);
}

ServoLimits ServoMotor::getLimits() const {
    ServoLimits limits;
    limits.angleMinDeg = config_.angleMinDeg;
    limits.angleMaxDeg = config_.angleMaxDeg;
    limits.pulseMinUs = config_.pulseMinUs;
    limits.pulseMaxUs = config_.pulseMaxUs;
    limits.maxVelocityDegPerS = config_.maxVelocityDegPerS;
    limits.maxAccelerationDegPerS2 = config_.maxAccelerationDegPerS2;
    return limits;
}

void ServoMotor::update() {
    if (!initialized_) {
        return;
    }

    uint32_t currentTime = getCurrentTimeMs();

    if (currentTime - state_.lastCommandTime > config_.watchdogTimeoutMs) {
        handleTimeout();
        return;
    }

    if (!state_.enabled) {
        return;
    }

    float targetAngle = state_.targetAngleDeg;
    applyRateLimiting(targetAngle);

    state_.currentAngleDeg = targetAngle;
    lastAngleDeg_ = targetAngle;
    lastUpdateTime_ = currentTime;

    uint16_t pulseUs = static_cast<uint16_t>(angleToPulse(targetAngle));
    updatePWM(pulseUs);
    state_.currentPulseUs = pulseUs;
}

void ServoMotor::resetWatchdog() {
    state_.lastCommandTime = getCurrentTimeMs();
    
    if (state_.status == ServoStatus::TIMEOUT) {
        state_.status = ServoStatus::OK;
    }
}

ServoStatus ServoMotor::validateConfig(const ServoConfig& config) const {
    if (config.angleMinDeg >= config.angleMaxDeg) {
        return ServoStatus::CONFIG_ERROR;
    }
    
    if (config.pulseMinUs >= config.pulseMaxUs) {
        return ServoStatus::CONFIG_ERROR;
    }
    
    if (config.pulseNeutralUs < config.pulseMinUs || 
        config.pulseNeutralUs > config.pulseMaxUs) {
        return ServoStatus::CONFIG_ERROR;
    }
    
    if (config.maxVelocityDegPerS <= 0 || config.maxAccelerationDegPerS2 <= 0) {
        return ServoStatus::CONFIG_ERROR;
    }
    
    return ServoStatus::OK;
}

ServoStatus ServoMotor::retrieveTimerInfo() {
    if (htim_ == nullptr || htim_->Instance == nullptr) {
        return ServoStatus::TIMER_ERROR;
    }

    timerPeriod_ = htim_->Init.Period;
    timerPrescaler_ = htim_->Init.Prescaler;

    if (timerPeriod_ == 0 || timerPrescaler_ == 0) {
        return ServoStatus::TIMER_ERROR;
    }

    float timerFreqHz = static_cast<float>(timerClockHz_) / (timerPrescaler_ + 1);
    float pwmPeriodUs = 1000000.0f / (timerFreqHz / (timerPeriod_ + 1));
    tickToUs_ = pwmPeriodUs / (timerPeriod_ + 1);

    return ServoStatus::OK;
}

float ServoMotor::constrainAngle(float angleDeg) const {
    float adjustedAngle = angleDeg + config_.offsetDeg;
    
    if (config_.directionInverted) {
        adjustedAngle = -adjustedAngle;
    }
    
    if (adjustedAngle < config_.angleMinDeg) {
        return config_.angleMinDeg;
    }
    if (adjustedAngle > config_.angleMaxDeg) {
        return config_.angleMaxDeg;
    }
    
    return adjustedAngle;
}

uint16_t ServoMotor::constrainPulse(uint16_t pulseUs) const {
    if (pulseUs < config_.pulseMinUs) {
        return config_.pulseMinUs;
    }
    if (pulseUs > config_.pulseMaxUs) {
        return config_.pulseMaxUs;
    }
    return pulseUs;
}

float ServoMotor::angleToPulse(float angleDeg) const {
    float normalizedAngle = (angleDeg - config_.angleMinDeg) / 
                           (config_.angleMaxDeg - config_.angleMinDeg);
    
    float pulseRange = static_cast<float>(config_.pulseMaxUs - config_.pulseMinUs);
    float pulseUs = config_.pulseMinUs + normalizedAngle * pulseRange;
    
    return pulseUs;
}

float ServoMotor::pulseToAngle(uint16_t pulseUs) const {
    float normalizedPulse = static_cast<float>(pulseUs - config_.pulseMinUs) / 
                           static_cast<float>(config_.pulseMaxUs - config_.pulseMinUs);
    
    float angleRange = config_.angleMaxDeg - config_.angleMinDeg;
    float angleDeg = config_.angleMinDeg + normalizedPulse * angleRange;
    
    return angleDeg;
}

uint32_t ServoMotor::pulseToTicks(uint16_t pulseUs) const {
    if (tickToUs_ <= 0.0f) {
        return 0;
    }
    
    uint32_t ticks = static_cast<uint32_t>(pulseUs / tickToUs_);
    
    if (ticks > timerPeriod_) {
        ticks = timerPeriod_;
    }
    
    return ticks;
}

void ServoMotor::applyRateLimiting(float& targetAngleDeg) {
    uint32_t currentTime = getCurrentTimeMs();
    float deltaTimeS = static_cast<float>(currentTime - lastUpdateTime_) / 1000.0f;
    
    if (deltaTimeS <= 0.0f) {
        return;
    }
    
    float angleDiff = targetAngleDeg - lastAngleDeg_;
    float maxAngleChange = config_.maxVelocityDegPerS * deltaTimeS;
    
    if (std::abs(angleDiff) > maxAngleChange) {
        if (angleDiff > 0) {
            targetAngleDeg = lastAngleDeg_ + maxAngleChange;
        } else {
            targetAngleDeg = lastAngleDeg_ - maxAngleChange;
        }
    }
}

void ServoMotor::updatePWM(uint16_t pulseUs) {
    if (htim_ == nullptr) {
        return;
    }
    
    uint32_t ccr = pulseToTicks(pulseUs);
    __HAL_TIM_SET_COMPARE(htim_, channel_, ccr);
}

void ServoMotor::handleTimeout() {
    if (state_.status != ServoStatus::TIMEOUT) {
        state_.status = ServoStatus::TIMEOUT;
        state_.timeoutCount++;
        applyFailSafe();
    }
}

void ServoMotor::applyFailSafe() {
    switch (config_.failSafeBehavior) {
        case FailSafeBehavior::HOLD_POSITION:
            break;
            
        case FailSafeBehavior::NEUTRAL_POSITION:
            state_.targetAngleDeg = 0.0f;
            state_.currentAngleDeg = 0.0f;
            updatePWM(config_.pulseNeutralUs);
            state_.currentPulseUs = config_.pulseNeutralUs;
            break;
            
        case FailSafeBehavior::DISABLE_OUTPUT:
            state_.enabled = false;
            updatePWM(0);
            state_.currentPulseUs = 0;
            break;
    }
}

uint32_t ServoMotor::getCurrentTimeMs() const {
    return HAL_GetTick();
}
