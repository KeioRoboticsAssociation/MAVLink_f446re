/**
 * @file hardware_manager.cpp
 * @brief Implements the Hardware Abstraction Layer (HAL) for managing hardware resources.
 */

#include "hardware_manager.hpp"
#include "../config/motor_config.hpp"

extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
extern UART_HandleTypeDef huart2;
extern CAN_HandleTypeDef hcan1;

namespace HAL {

HardwareManager g_hardwareManager;

/**
 * @brief Initializes the hardware manager and all underlying hardware peripherals.
 * @return Result of the operation.
 */
Config::Result<void> HardwareManager::initialize() {
    if (initialized_) {
        return Config::Result<void>();
    }

    auto timerResult = initializeTimers();
    if (!timerResult) {
        return Config::Result<void>(timerResult.error());
    }

    auto uartResult = initializeUARTs();
    if (!uartResult) {
        return Config::Result<void>(uartResult.error());
    }

    auto canResult = initializeCAN();
    if (!canResult) {
        return Config::Result<void>(canResult.error());
    }

    initialized_ = true;
    return Config::Result<void>();
}

/**
 * @brief Gets a handle to a hardware timer.
 * @param id The ID of the timer to get.
 * @return A Result containing a pointer to the TimerHandle or an error.
 */
Config::Result<TimerHandle*> HardwareManager::getTimer(TimerID id) {
    if (!initialized_) {
        return Config::Result<TimerHandle*>(Config::ErrorCode::NOT_INITIALIZED);
    }
    auto* handle = getTimerHandle(id);
    if (!handle || !handle->initialized) {
        return Config::Result<TimerHandle*>(Config::ErrorCode::HARDWARE_ERROR);
    }
    return handle;
}

/**
 * @brief Starts PWM on a timer channel.
 * @param id The ID of the timer.
 * @param channel The timer channel.
 * @return Result of the operation.
 */
Config::Result<void> HardwareManager::startPWM(TimerID id, uint32_t channel) {
    auto timerResult = getTimer(id);
    if (!timerResult) {
        return Config::Result<void>(timerResult.error());
    }
    if (HAL_TIM_PWM_Start(timerResult.get()->htim, channel) != HAL_OK) {
        return Config::Result<void>(Config::ErrorCode::HARDWARE_ERROR);
    }
    return Config::Result<void>();
}

/**
 * @brief Sets the PWM duty cycle on a timer channel.
 * @param id The ID of the timer.
 * @param channel The timer channel.
 * @param value The PWM duty cycle value.
 * @return Result of the operation.
 */
Config::Result<void> HardwareManager::setPWMDutyCycle(TimerID id, uint32_t channel, uint32_t value) {
    auto timerResult = getTimer(id);
    if (!timerResult) {
        return Config::Result<void>(timerResult.error());
    }
    __HAL_TIM_SET_COMPARE(timerResult.get()->htim, channel, value);
    return Config::Result<void>();
}

/**
 * @brief Gets a handle to a UART peripheral.
 * @param id The ID of the UART to get.
 * @return A Result containing a pointer to the UARTHandle or an error.
 */
Config::Result<UARTHandle*> HardwareManager::getUART(UARTID id) {
    if (!initialized_) {
        return Config::Result<UARTHandle*>(Config::ErrorCode::NOT_INITIALIZED);
    }
    auto* handle = getUARTHandle(id);
    if (!handle || !handle->initialized) {
        return Config::Result<UARTHandle*>(Config::ErrorCode::HARDWARE_ERROR);
    }
    return handle;
}

/**
 * @brief Transmits data over UART.
 * @param id The ID of the UART.
 * @param data A pointer to the data to transmit.
 * @param length The length of the data to transmit.
 * @return Result of the operation.
 */
Config::Result<void> HardwareManager::transmitUART(UARTID id, const uint8_t* data, size_t length) {
    auto uartResult = getUART(id);
    if (!uartResult) {
        return Config::Result<void>(uartResult.error());
    }
    if (HAL_UART_Transmit(uartResult.get()->huart, const_cast<uint8_t*>(data), length, 100) != HAL_OK) {
        return Config::Result<void>(Config::ErrorCode::COMMUNICATION_ERROR);
    }
    return Config::Result<void>();
}

/**
 * @brief Sets the callback function for UART RX.
 * @param id The ID of the UART.
 * @param callback The callback function.
 * @return Result of the operation.
 */
Config::Result<void> HardwareManager::setUARTRxCallback(UARTID id, std::function<void(uint8_t*, size_t)> callback) {
    auto uartResult = getUART(id);
    if (!uartResult) {
        return Config::Result<void>(uartResult.error());
    }
    uartResult.get()->rxCallback = callback;
    return Config::Result<void>();
}

/**
 * @brief Gets a handle to a CAN peripheral.
 * @param id The ID of the CAN peripheral to get.
 * @return A Result containing a pointer to the CANHandle or an error.
 */
Config::Result<CANHandle*> HardwareManager::getCAN(CANID id) {
    if (!initialized_) {
        return Config::Result<CANHandle*>(Config::ErrorCode::NOT_INITIALIZED);
    }
    auto* handle = getCANHandle(id);
    if (!handle || !handle->initialized) {
        return Config::Result<CANHandle*>(Config::ErrorCode::HARDWARE_ERROR);
    }
    return handle;
}

/**
 * @brief Reads the state of a GPIO pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @return A Result containing the state of the pin or an error.
 */
Config::Result<bool> HardwareManager::readGPIO(GPIO_TypeDef* port, uint16_t pin) {
    if (!initialized_) {
        return Config::Result<bool>(Config::ErrorCode::NOT_INITIALIZED);
    }
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET);
}

/**
 * @brief Writes a state to a GPIO pin.
 * @param port The GPIO port.
 * @param pin The GPIO pin.
 * @param state The state to write.
 * @return Result of the operation.
 */
Config::Result<void> HardwareManager::writeGPIO(GPIO_TypeDef* port, uint16_t pin, bool state) {
    if (!initialized_) {
        return Config::Result<void>(Config::ErrorCode::NOT_INITIALIZED);
    }
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return Config::Result<void>();
}

Config::Result<void> HardwareManager::initializeTimers() {
    timers_[0] = {&htim1, 0, true, nullptr};
    timers_[1] = {&htim2, 0, true, nullptr};
    timers_[2] = {&htim3, 0, true, nullptr};
    timers_[3] = {&htim4, 0, true, nullptr};
    return Config::Result<void>();
}

Config::Result<void> HardwareManager::initializeUARTs() {
    uarts_[0] = {&huart2, true, nullptr, nullptr, nullptr};
    return Config::Result<void>();
}

Config::Result<void> HardwareManager::initializeCAN() {
    can_[0] = {&hcan1, true, nullptr, nullptr, nullptr};
    return Config::Result<void>();
}

TimerHandle* HardwareManager::getTimerHandle(TimerID id) {
    size_t index = static_cast<size_t>(id) - 1;
    return (index < timers_.size()) ? &timers_[index] : nullptr;
}

UARTHandle* HardwareManager::getUARTHandle(UARTID id) {
    return (id == UARTID::UART_2) ? &uarts_[0] : nullptr;
}

CANHandle* HardwareManager::getCANHandle(CANID id) {
    return (id == CANID::CAN_1) ? &can_[0] : nullptr;
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

extern "C" {

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM_1);
    else if (htim == &htim2) HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM_2);
    else if (htim == &htim3) HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM_3);
    else if (htim == &htim4) HAL::g_hardwareManager.handleTimerError(HAL::TimerID::TIM_4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        auto uartResult = HAL::g_hardwareManager.getUART(HAL::UARTID::UART_2);
        if (uartResult && uartResult.get()->rxCallback) {
            // RX callback logic here
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        auto uartResult = HAL::g_hardwareManager.getUART(HAL::UARTID::UART_2);
        if (uartResult && uartResult.get()->txCompleteCallback) {
            uartResult.get()->txCompleteCallback();
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        HAL::g_hardwareManager.handleUARTError(HAL::UARTID::UART_2);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        auto canResult = HAL::g_hardwareManager.getCAN(HAL::CANID::CAN_1);
        if (canResult && canResult.get()->rxCallback) {
            // CAN RX callback logic here
        }
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        auto canResult = HAL::g_hardwareManager.getCAN(HAL::CANID::CAN_1);
        if (canResult && canResult.get()->txCompleteCallback) {
            canResult.get()->txCompleteCallback();
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        HAL::g_hardwareManager.handleCANError(HAL::CANID::CAN_1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}

}