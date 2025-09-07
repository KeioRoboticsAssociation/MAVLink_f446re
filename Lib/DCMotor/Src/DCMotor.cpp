#include "DCMotor.hpp"

DCMotor::DCMotor(TIM_HandleTypeDef *htim, uint16_t channel, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, bool direction, uint16_t pwm_resolution) {
    this->htim = htim;
    this->channel = channel;
    this->direction = direction;
    this->pwm_resolution = pwm_resolution;
    this->GPIO_Port = GPIO_Port;
    this->GPIO_Pin = GPIO_Pin;
    
}

void DCMotor::start() {
    HAL_TIM_PWM_Start(htim, channel);
}

void DCMotor::setDuty(float duty) {
    uint16_t value;
    if (duty > 1.0f) {
        duty = 1.0f;
    }else if (duty < -1.0f) {
        duty = -1.0f;
    }
    
    if (duty > 0){
        value = (uint16_t)(pwm_resolution * (1.0f-duty));
        setDirection(1);
    }else{
        value = (uint16_t)(pwm_resolution * (1.0f+duty));
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
