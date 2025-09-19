#include "DCMotor.hpp"

DCMotor::DCMotor(TIM_HandleTypeDef *htim, uint16_t channel, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, bool direction) {
    this->htim = htim;
    this->channel = channel;
    this->direction = direction;
    this->GPIO_Port = GPIO_Port;
    this->GPIO_Pin = GPIO_Pin;

    // Initialize state
    this->current_duty_ = 0.0f;
    this->started_ = false;
    this->motor_id_ = 0;
}

void DCMotor::start() {
    HAL_TIM_PWM_Start(htim, channel);
    started_ = true;
}

void DCMotor::stop() {
    HAL_TIM_PWM_Stop(htim, channel);
    current_duty_ = 0.0f;
    started_ = false;
}

void DCMotor::brake() {
    setDuty(0.0f);
}

void DCMotor::setDuty(float duty) {
    uint16_t value;
    if (duty > 1.0f) {
        duty = 1.0f;
    }else if (duty < -1.0f) {
        duty = -1.0f;
    }

    current_duty_ = duty;

    // Use timer period as PWM resolution
    uint16_t pwm_period = htim->Init.Period;

    if (duty > 0){
        value = (uint16_t)(pwm_period * (duty));
        setDirection(1);
    }else{
        value = (uint16_t)(pwm_period * (-duty));
        setDirection(0);
    }
    __HAL_TIM_SET_COMPARE(htim, channel, value);
}

void DCMotor::setDirection(bool direction) {
    if (direction == this->direction) {
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
    }
}
