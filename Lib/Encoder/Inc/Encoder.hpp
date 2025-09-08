#pragma once

#include "stm32f4xx_hal.h"
#include <cstdint>
#include <cmath>

enum class EncoderStatus {
    OK = 0,
    NOT_INITIALIZED,
    TIMER_ERROR,
    OUT_OF_RANGE,
    TIMEOUT,
    CONFIG_ERROR
};

enum class EncoderMode {
    TIM_ENCODER_MODE = 0,
    EXTERNAL_COUNTER
};

struct EncoderConfig {
    uint16_t cpr = 1024;
    bool invertA = false;
    bool invertB = false;
    bool useZ = true;
    EncoderMode mode = EncoderMode::TIM_ENCODER_MODE;
    uint32_t watchdogTimeoutMs = 500;
    int32_t offsetCounts = 0;
    bool wrapAround = true;
};

struct EncoderState {
    int32_t currentPosition = 0;
    int32_t rawPosition = 0;
    bool zDetected = false;
    bool enabled = true;
    EncoderStatus status = EncoderStatus::NOT_INITIALIZED;
    uint32_t lastUpdateTime = 0;
    uint32_t errorCount = 0;
    uint32_t revolutions = 0;
    float currentAngleRad = 0.0f;
    float currentAngleDeg = 0.0f;
};

struct EncoderLimits {
    int32_t maxPosition;
    int32_t minPosition;
    uint16_t cpr;
    float maxAngleRad;
    float maxAngleDeg;
};

class Encoder {
public:
    Encoder();
    ~Encoder() = default;

    EncoderStatus create(uint8_t id, TIM_HandleTypeDef* htim);
    EncoderStatus init();
    EncoderStatus init(const EncoderConfig& config);

    EncoderStatus setConfig(const EncoderConfig& config);
    EncoderStatus updateConfig(const EncoderConfig& config);
    EncoderStatus loadConfigFromJson(const char* jsonString);
    EncoderStatus loadConfigFromFile(const char* filePath);
    EncoderStatus loadConfigFromFileForId(const char* filePath);
    EncoderStatus saveConfigToFile(const char* filePath) const;

    int32_t getPosition() const { return state_.currentPosition; }
    int32_t getRawPosition() const { return state_.rawPosition; }
    float getAngleRad() const { return state_.currentAngleRad; }
    float getAngleDeg() const { return state_.currentAngleDeg; }
    uint32_t getRevolutions() const { return state_.revolutions; }
    bool isZDetected() const { return state_.zDetected; }
    
    EncoderLimits getLimits() const;
    EncoderStatus getStatus() const { return state_.status; }
    const EncoderState& getState() const { return state_; }
    const EncoderConfig& getConfig() const { return config_; }
    uint8_t getId() const { return id_; }

    EncoderStatus reset();
    EncoderStatus setZeroPosition();
    EncoderStatus setPosition(int32_t position);
    
    void update();
    void resetWatchdog();

private:
    uint8_t id_ = 0;
    TIM_HandleTypeDef* htim_ = nullptr;
    bool initialized_ = false;

    EncoderConfig config_;
    EncoderState state_;

    int32_t lastRawPosition_ = 0;
    uint32_t lastUpdateTime_ = 0;
    bool zIndexFound_ = false;

    EncoderStatus validateConfig(const EncoderConfig& config) const;
    EncoderStatus initTimerEncoder();
    void updatePosition();
    void updateAngles();
    void handleWrapAround(int32_t& position);
    void detectRevolutions();
    uint32_t getCurrentTimeMs() const;
};