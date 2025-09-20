#include "hardware_manager.hpp"
#include "../config/motor_config.hpp"

// External hardware handles from main.c
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;

namespace HAL {

// Global hardware manager instance
HardwareManager g_hardwareManager;

Config::Result<Config::ErrorCode> HardwareManager::initialize() {
    if (initialized_) {
        return Config::ErrorCode::OK;
    }

    // Initialize timer handles
    auto timerResult = initializeTimers();
    if (!timerResult) {
        return timerResult.error();
    }

    // Initialize UART handles
    auto uartResult = initializeUARTs();
    if (!uartResult) {
        return uartResult.error();
    }

    // Initialize CAN handles
    auto canResult = initializeCAN();
    if (!canResult) {
        return canResult.error();
    }

    initialized_ = true;
    return Config::ErrorCode::OK;
}

Config::Result<TimerHandle*> HardwareManager::getTimer(TimerID id) {
    if (!initialized_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto* handle = getTimerHandle(id);
    if (!handle || !handle->initialized) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    return handle;
}

Config::Result<Config::ErrorCode> HardwareManager::startPWM(TimerID id, uint32_t channel) {
    auto timerResult = getTimer(id);
    if (!timerResult) {
        return timerResult.error();
    }

    auto* timer = timerResult.get();
    if (HAL_TIM_PWM_Start(timer->htim, channel) != HAL_OK) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> HardwareManager::setPWMDutyCycle(TimerID id, uint32_t channel, uint32_t value) {
    auto timerResult = getTimer(id);
    if (!timerResult) {
        return timerResult.error();
    }

    auto* timer = timerResult.get();
    __HAL_TIM_SET_COMPARE(timer->htim, channel, value);

    return Config::ErrorCode::OK;
}

Config::Result<UARTHandle*> HardwareManager::getUART(UARTID id) {
    if (!initialized_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto* handle = getUARTHandle(id);
    if (!handle || !handle->initialized) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    return handle;
}

Config::Result<Config::ErrorCode> HardwareManager::transmitUART(UARTID id, const uint8_t* data, size_t length) {
    auto uartResult = getUART(id);
    if (!uartResult) {
        return uartResult.error();
    }

    auto* uart = uartResult.get();
    if (HAL_UART_Transmit(uart->huart, const_cast<uint8_t*>(data), length, 100) != HAL_OK) {
        return Config::ErrorCode::COMMUNICATION_ERROR;
    }

    return Config::ErrorCode::OK;
}

Config::Result<CANHandle*> HardwareManager::getCAN(CANID id) {
    if (!initialized_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    auto* handle = getCANHandle(id);
    if (!handle || !handle->initialized) {
        return Config::ErrorCode::HARDWARE_ERROR;
    }

    return handle;
}

Config::Result<bool> HardwareManager::readGPIO(GPIO_TypeDef* port, uint16_t pin) {
    if (!initialized_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);
    return (state == GPIO_PIN_SET);
}

Config::Result<Config::ErrorCode> HardwareManager::writeGPIO(GPIO_TypeDef* port, uint16_t pin, bool state) {
    if (!initialized_) {
        return Config::ErrorCode::NOT_INITIALIZED;
    }

    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return Config::ErrorCode::OK;
}

// Private implementation methods

Config::Result<Config::ErrorCode> HardwareManager::initializeTimers() {
    // Initialize TIM1
    timers_[0] = {&htim1, 0, true, nullptr};

    // Initialize TIM2
    timers_[1] = {&htim2, 0, true, nullptr};

    // Initialize TIM3
    timers_[2] = {&htim3, 0, true, nullptr};

    // Initialize TIM4
    timers_[3] = {&htim4, 0, true, nullptr};

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> HardwareManager::initializeUARTs() {
    // Initialize UART2
    uarts_[0] = {&huart2, true, nullptr, nullptr, nullptr};

    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> HardwareManager::initializeCAN() {
    // Initialize CAN1
    can_[0] = {&hcan1, true, nullptr, nullptr, nullptr};

    return Config::ErrorCode::OK;
}

TimerHandle* HardwareManager::getTimerHandle(TimerID id) {
    size_t index = static_cast<size_t>(id) - 1; // Convert to 0-based index
    if (index >= timers_.size()) {
        return nullptr;
    }
    return &timers_[index];
}

UARTHandle* HardwareManager::getUARTHandle(UARTID id) {
    if (id == UARTID::UART2) {
        return &uarts_[0];
    }
    return nullptr;
}

CANHandle* HardwareManager::getCANHandle(CANID id) {
    if (id == CANID::CAN1) {
        return &can_[0];
    }
    return nullptr;
}

void HardwareManager::handleTimerError(TimerID id) {
    auto* timer = getTimerHandle(id);
    if (timer && timer->errorCallback) {
        timer->errorCallback();
    }
}

void HardwareManager::handleUARTError(UARTID id) {
    auto* uart = getUARTHandle(id);
    if (uart && uart->errorCallback) {
        uart->errorCallback();
    }
}

void HardwareManager::handleCANError(CANID id) {
    auto* can = getCANHandle(id);
    if (can && can->errorCallback) {
        can->errorCallback();
    }
}

} // namespace HAL

// Hardware interrupt handlers implementation
extern "C" {

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    // Handle timer completion if needed
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) {
        HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM1);
    } else if (htim == &htim2) {
        HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM2);
    } else if (htim == &htim3) {
        HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM3);
    } else if (htim == &htim4) {
        HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM4);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        auto uartResult = HAL::g_hardwareManager.getUART(HAL::UARTID::UART2);
        if (uartResult && uartResult.get()->rxCallback) {
            // Handle received data - implementation depends on buffer management
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        auto uartResult = HAL::g_hardwareManager.getUART(HAL::UARTID::UART2);
        if (uartResult && uartResult.get()->txCompleteCallback) {
            uartResult.get()->txCompleteCallback();
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        HAL::g_hardwareManager.handleUARTError(HAL::UARTID::UART2);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        auto canResult = HAL::g_hardwareManager.getCAN(HAL::CANID::CAN1);
        if (canResult && canResult.get()->rxCallback) {
            // Handle CAN message reception
        }
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        auto canResult = HAL::g_hardwareManager.getCAN(HAL::CANID::CAN1);
        if (canResult && canResult.get()->txCompleteCallback) {
            canResult.get()->txCompleteCallback();
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        HAL::g_hardwareManager.handleCANError(HAL::CANID::CAN1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Handle GPIO interrupts (limit switches, etc.)
}

}