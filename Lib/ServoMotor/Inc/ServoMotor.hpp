#pragma once

#include "stm32f4xx_hal.h"
#include <cstdint>
#include <cmath>

enum class ServoStatus {
    OK = 0,
    NOT_INITIALIZED,
    TIMER_ERROR,
    OUT_OF_RANGE,
    TIMEOUT,
    CONFIG_ERROR
};

enum class FailSafeBehavior {
    HOLD_POSITION = 0,
    NEUTRAL_POSITION,
    DISABLE_OUTPUT
};

struct ServoConfig {
    float angleMinDeg = -90.0f;
    float angleMaxDeg = 90.0f;
    uint16_t pulseMinUs = 1000;
    uint16_t pulseMaxUs = 2000;
    uint16_t pulseNeutralUs = 1500;
    bool directionInverted = false;
    float offsetDeg = 0.0f;
    float maxVelocityDegPerS = 180.0f;
    float maxAccelerationDegPerS2 = 360.0f;
    uint32_t watchdogTimeoutMs = 500;
    FailSafeBehavior failSafeBehavior = FailSafeBehavior::NEUTRAL_POSITION;
    float startupAngleDeg = 0.0f;
    bool startDisabled = false;
};

struct ServoState {
    float currentAngleDeg = 0.0f;
    float targetAngleDeg = 0.0f;
    uint16_t currentPulseUs = 1500;
    bool enabled = false;
    ServoStatus status = ServoStatus::NOT_INITIALIZED;
    uint32_t lastCommandTime = 0;
    uint32_t saturationCount = 0;
    uint32_t timeoutCount = 0;
    uint32_t errorCount = 0;
};

struct ServoLimits {
    float angleMinDeg;
    float angleMaxDeg;
    uint16_t pulseMinUs;
    uint16_t pulseMaxUs;
    float maxVelocityDegPerS;
    float maxAccelerationDegPerS2;
};

class ServoMotor {
public:
    ServoMotor();
    ~ServoMotor() = default;

    ServoStatus create(uint8_t id, TIM_HandleTypeDef* htim, uint32_t channel);
    ServoStatus init();
    ServoStatus init(const ServoConfig& config);

    ServoStatus setAngleDeg(float angleDeg);
    ServoStatus setAngleRad(float angleRad);
    ServoStatus setPulseUs(uint16_t pulseUs);
    ServoStatus setEnabled(bool enabled);

    ServoStatus setConfig(const ServoConfig& config);
    ServoStatus updateConfig(const ServoConfig& config);
    ServoStatus loadConfigFromJson(const char* jsonString);
    ServoStatus loadConfigFromFile(const char* filePath);
    ServoStatus loadConfigFromFileForId(const char* filePath);
    ServoStatus saveConfigToFile(const char* filePath) const;

    float getAngleCmd() const { return state_.targetAngleDeg; }
    float getCurrentAngle() const { return state_.currentAngleDeg; }
    uint16_t getPulseUs() const { 
        // Return current pulse, or neutral if not initialized
        return initialized_ ? state_.currentPulseUs : config_.pulseNeutralUs; 
    }
    ServoLimits getLimits() const;
    ServoStatus getStatus() const { return state_.status; }
    const ServoState& getState() const { return state_; }
    uint8_t getId() const { return id_; }

    void update();
    void resetWatchdog();

private:
    uint8_t id_ = 0;
    TIM_HandleTypeDef* htim_ = nullptr;
    uint32_t channel_ = 0;
    bool initialized_ = false;

    ServoConfig config_;
    ServoState state_;

    uint32_t timerPeriod_ = 0;
    uint32_t timerPrescaler_ = 0;
    uint32_t timerClockHz_ = 84000000;
    float tickToUs_ = 0.0f;

    float lastAngleDeg_ = 0.0f;
    uint32_t lastUpdateTime_ = 0;

    ServoStatus validateConfig(const ServoConfig& config) const;
    ServoStatus retrieveTimerInfo();
    float constrainAngle(float angleDeg) const;
    uint16_t constrainPulse(uint16_t pulseUs) const;
    float angleToPulse(float angleDeg) const;
    float pulseToAngle(uint16_t pulseUs) const;
    uint32_t pulseToTicks(uint16_t pulseUs) const;
    void applyRateLimiting(float& targetAngleDeg);
    void updatePWM(uint16_t pulseUs);
    void handleTimeout();
    void applyFailSafe();
    uint32_t getCurrentTimeMs() const;
};