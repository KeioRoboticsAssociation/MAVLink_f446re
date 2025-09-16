#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stm32f4xx_hal_tim.h>
#include <stm32f446xx.h>

class DCMotor {
    public:
        DCMotor(TIM_HandleTypeDef *htim, uint16_t channel, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, bool direction, uint16_t pwm_resolution);
        void setDuty(float duty);
        void start();
        void stop();
        void brake();

        // Getters
        float getCurrentDuty() const { return current_duty_; }
        bool isStarted() const { return started_; }
        uint8_t getMotorId() const { return motor_id_; }
        void setMotorId(uint8_t id) { motor_id_ = id; }

    private:
        void setDirection(bool direction);
        TIM_HandleTypeDef *htim;
        uint16_t channel;
        uint16_t pwm_resolution;
        bool direction;

        GPIO_TypeDef* GPIO_Port;
        uint16_t GPIO_Pin;

        // State tracking
        float current_duty_;
        bool started_;
        uint8_t motor_id_;
};

#ifdef __cplusplus
}
#endif