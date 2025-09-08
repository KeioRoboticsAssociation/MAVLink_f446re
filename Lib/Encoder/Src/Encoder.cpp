#include "Encoder.hpp"
#include "EncoderConfig.hpp"

Encoder::Encoder() {
    state_.status = EncoderStatus::NOT_INITIALIZED;
    state_.enabled = true;
    state_.currentPosition = 0;
    state_.rawPosition = 0;
    state_.zDetected = false;
    state_.lastUpdateTime = getCurrentTimeMs();
    state_.currentAngleRad = 0.0f;
    state_.currentAngleDeg = 0.0f;
    state_.revolutions = 0;
}

EncoderStatus Encoder::create(uint8_t id, TIM_HandleTypeDef* htim) {
    if (htim == nullptr) {
        return EncoderStatus::TIMER_ERROR;
    }

    id_ = id;
    htim_ = htim;
    
    return EncoderStatus::OK;
}

EncoderStatus Encoder::init() {
    return init(EncoderConfig{});
}

EncoderStatus Encoder::init(const EncoderConfig& config) {
    if (htim_ == nullptr) {
        state_.status = EncoderStatus::TIMER_ERROR;
        return state_.status;
    }

    EncoderStatus configStatus = validateConfig(config);
    if (configStatus != EncoderStatus::OK) {
        state_.status = configStatus;
        return state_.status;
    }

    config_ = config;

    EncoderStatus timerStatus = initTimerEncoder();
    if (timerStatus != EncoderStatus::OK) {
        state_.status = timerStatus;
        return state_.status;
    }

    initialized_ = true;
    state_.enabled = true;
    state_.currentPosition = config_.offsetCounts;
    state_.rawPosition = 0;
    state_.lastUpdateTime = getCurrentTimeMs();
    lastUpdateTime_ = state_.lastUpdateTime;

    updateAngles();

    state_.status = EncoderStatus::OK;
    return state_.status;
}

EncoderStatus Encoder::setConfig(const EncoderConfig& config) {
    EncoderStatus status = validateConfig(config);
    if (status != EncoderStatus::OK) {
        return status;
    }

    config_ = config;
    updateAngles();
    return EncoderStatus::OK;
}

EncoderStatus Encoder::updateConfig(const EncoderConfig& config) {
    return setConfig(config);
}

EncoderStatus Encoder::loadConfigFromJson(const char* jsonString) {
    EncoderConfig newConfig;
    EncoderStatus status = EncoderConfigParser::parseFromJson(jsonString, newConfig);
    if (status != EncoderStatus::OK) {
        return status;
    }
    return setConfig(newConfig);
}

EncoderStatus Encoder::loadConfigFromFile(const char* filePath) {
    EncoderConfig newConfig;
    EncoderStatus status = EncoderConfigParser::loadFromFile(filePath, newConfig);
    if (status != EncoderStatus::OK) {
        return status;
    }
    return setConfig(newConfig);
}

EncoderStatus Encoder::loadConfigFromFileForId(const char* filePath) {
    EncoderConfig newConfig;
    EncoderStatus status = EncoderConfigParser::loadFromFileForId(filePath, id_, newConfig);
    if (status != EncoderStatus::OK) {
        return status;
    }
    return setConfig(newConfig);
}

EncoderStatus Encoder::saveConfigToFile(const char* filePath) const {
    return EncoderConfigParser::saveToFile(filePath, config_);
}

EncoderLimits Encoder::getLimits() const {
    EncoderLimits limits;
    limits.cpr = config_.cpr;
    limits.maxPosition = config_.wrapAround ? (config_.cpr - 1) : INT32_MAX;
    limits.minPosition = config_.wrapAround ? 0 : INT32_MIN;
    limits.maxAngleRad = 2.0f * M_PI;
    limits.maxAngleDeg = 360.0f;
    return limits;
}

EncoderStatus Encoder::reset() {
    if (!initialized_) {
        return EncoderStatus::NOT_INITIALIZED;
    }

    state_.currentPosition = config_.offsetCounts;
    state_.rawPosition = 0;
    state_.revolutions = 0;
    state_.zDetected = false;
    zIndexFound_ = false;
    
    if (htim_ != nullptr) {
        __HAL_TIM_SET_COUNTER(htim_, 0);
    }
    
    updateAngles();
    return EncoderStatus::OK;
}

EncoderStatus Encoder::setZeroPosition() {
    if (!initialized_) {
        return EncoderStatus::NOT_INITIALIZED;
    }

    state_.currentPosition = 0;
    state_.rawPosition = 0;
    state_.revolutions = 0;
    state_.zDetected = true;
    zIndexFound_ = true;
    
    if (htim_ != nullptr) {
        __HAL_TIM_SET_COUNTER(htim_, 0);
    }
    
    updateAngles();
    return EncoderStatus::OK;
}

EncoderStatus Encoder::setPosition(int32_t position) {
    if (!initialized_) {
        return EncoderStatus::NOT_INITIALIZED;
    }

    state_.currentPosition = position;
    state_.rawPosition = position;
    updateAngles();
    return EncoderStatus::OK;
}

void Encoder::update() {
    if (!initialized_ || !state_.enabled) {
        return;
    }

    uint32_t currentTime = getCurrentTimeMs();

    if (currentTime - state_.lastUpdateTime > config_.watchdogTimeoutMs) {
        if (state_.status != EncoderStatus::TIMEOUT) {
            state_.status = EncoderStatus::TIMEOUT;
            state_.errorCount++;
        }
        return;
    }

    updatePosition();
    updateAngles();
    detectRevolutions();

    state_.lastUpdateTime = currentTime;
    lastUpdateTime_ = currentTime;
}

void Encoder::resetWatchdog() {
    state_.lastUpdateTime = getCurrentTimeMs();
    
    if (state_.status == EncoderStatus::TIMEOUT) {
        state_.status = EncoderStatus::OK;
    }
}

EncoderStatus Encoder::validateConfig(const EncoderConfig& config) const {
    if (config.cpr == 0) {
        return EncoderStatus::CONFIG_ERROR;
    }
    
    if (config.watchdogTimeoutMs == 0) {
        return EncoderStatus::CONFIG_ERROR;
    }
    
    return EncoderStatus::OK;
}

EncoderStatus Encoder::initTimerEncoder() {
    if (htim_ == nullptr || htim_->Instance == nullptr) {
        return EncoderStatus::TIMER_ERROR;
    }

    if (config_.mode == EncoderMode::TIM_ENCODER_MODE) {
        if (HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL) != HAL_OK) {
            return EncoderStatus::TIMER_ERROR;
        }

        uint32_t period = htim_->Init.Period;
        __HAL_TIM_SET_COUNTER(htim_, period / 2);
        lastRawPosition_ = period / 2;
    }

    return EncoderStatus::OK;
}

void Encoder::updatePosition() {
    if (htim_ == nullptr) {
        return;
    }

    int32_t currentRaw = static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_));
    int32_t deltaRaw = currentRaw - lastRawPosition_;

    uint32_t period = htim_->Init.Period;
    int32_t halfPeriod = static_cast<int32_t>(period / 2);

    if (deltaRaw > halfPeriod) {
        deltaRaw -= static_cast<int32_t>(period + 1);
    } else if (deltaRaw < -halfPeriod) {
        deltaRaw += static_cast<int32_t>(period + 1);
    }

    if (config_.invertA) {
        deltaRaw = -deltaRaw;
    }

    state_.rawPosition += deltaRaw;
    state_.currentPosition = state_.rawPosition + config_.offsetCounts;
    
    if (config_.wrapAround) {
        handleWrapAround(state_.currentPosition);
    }

    lastRawPosition_ = currentRaw;
}

void Encoder::updateAngles() {
    float angleRad = (2.0f * M_PI * static_cast<float>(state_.currentPosition)) / static_cast<float>(config_.cpr);
    state_.currentAngleRad = angleRad;
    state_.currentAngleDeg = angleRad * 180.0f / M_PI;
}

void Encoder::handleWrapAround(int32_t& position) {
    int32_t cpr = static_cast<int32_t>(config_.cpr);
    
    while (position >= cpr) {
        position -= cpr;
    }
    while (position < 0) {
        position += cpr;
    }
}

void Encoder::detectRevolutions() {
    if (!config_.wrapAround) {
        state_.revolutions = static_cast<uint32_t>(state_.currentPosition / static_cast<int32_t>(config_.cpr));
    }
}

uint32_t Encoder::getCurrentTimeMs() const {
    return HAL_GetTick();
}