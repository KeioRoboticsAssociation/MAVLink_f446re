/**
 * @file hardware_manager.hpp
 * @brief Defines the Hardware Abstraction Layer (HAL) for managing hardware resources.
 */

#pragma once

#include "stm32f4xx_hal.h"
#include "../config/system_config.hpp"
#include <array>
#include <functional>

namespace HAL {

/**
 * @brief Handle for a hardware timer.
 */
struct TimerHandle {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    bool initialized;
    std::function<void()> errorCallback;
};

/**
 * @brief Handle for a UART peripheral.
 */
struct UARTHandle {
    UART_HandleTypeDef* huart;
    bool initialized;
    std::function<void(uint8_t*, size_t)> rxCallback;
    std::function<void()> txCompleteCallback;
    std::function<void()> errorCallback;
};

/**
 * @brief Handle for a CAN peripheral.
 */
struct CANHandle {
    CAN_HandleTypeDef* hcan;
    bool initialized;
    std::function<void(CAN_RxHeaderTypeDef*, uint8_t*)> rxCallback;
    std::function<void()> txCompleteCallback;
    std::function<void()> errorCallback;
};

/**
 * @brief Handle for a GPIO pin.
 */
struct GPIOHandle {
    GPIO_TypeDef* port;
    uint16_t pin;
    GPIO_PinState activeState;
    std::function<void(bool)> changeCallback;
};

/**
 * @brief Type-safe identifiers for timers.
 */
enum class TimerID : uint8_t {
    TIM_1 = 1,
    TIM_2 = 2,
    TIM_3 = 3,
    TIM_4 = 4,
    MAX_TIMERS = 4
};

/**
 * @brief Type-safe identifiers for UARTs.
 */
enum class UARTID : uint8_t {
    UART_2 = 2,
    MAX_UARTS = 1
};

/**
 * @brief Type-safe identifiers for CAN peripherals.
 */
enum class CANID : uint8_t {
    CAN_1 = 1,
    MAX_CAN = 1
};

/**
 * @brief Central manager for all hardware resources.
 */
class HardwareManager {
private:
    std::array<TimerHandle, static_cast<size_t>(TimerID::MAX_TIMERS)> timers_;
    std::array<UARTHandle, static_cast<size_t>(UARTID::MAX_UARTS)> uarts_;
    std::array<CANHandle, static_cast<size_t>(CANID::MAX_CAN)> can_;
    bool initialized_ = false;

public:
    Config::Result<void> initialize();
    bool isInitialized() const { return initialized_; }

    Config::Result<TimerHandle*> getTimer(TimerID id);
    Config::Result<void> startPWM(TimerID id, uint32_t channel);
    Config::Result<void> setPWMDutyCycle(TimerID id, uint32_t channel, uint32_t value);
    Config::Result<void> setTimerCallback(TimerID id, std::function<void()> callback);

    Config::Result<UARTHandle*> getUART(UARTID id);
    Config::Result<void> transmitUART(UARTID id, const uint8_t* data, size_t length);
    Config::Result<void> setUARTRxCallback(UARTID id, std::function<void(uint8_t*, size_t)> callback);

    Config::Result<CANHandle*> getCAN(CANID id);
    Config::Result<void> transmitCAN(CANID id, uint32_t canId, const uint8_t* data, size_t length);
    Config::Result<void> setCANRxCallback(CANID id, std::function<void(CAN_RxHeaderTypeDef*, uint8_t*)> callback);

    Config::Result<bool> readGPIO(GPIO_TypeDef* port, uint16_t pin);
    Config::Result<void> writeGPIO(GPIO_TypeDef* port, uint16_t pin, bool state);
    Config::Result<void> setGPIOCallback(GPIO_TypeDef* port, uint16_t pin, std::function<void(bool)> callback);

    uint32_t getSystemTick() const { return HAL_GetTick(); }
    void delay(uint32_t ms) { HAL_Delay(ms); }

    void handleTimerError(TimerID id);
    void handleUARTError(UARTID id);
    void handleCANError(CANID id);

private:
    Config::Result<void> initializeTimers();
    Config::Result<void> initializeUARTs();
    Config::Result<void> initializeCAN();

    TimerHandle* getTimerHandle(TimerID id);
    UARTHandle* getUARTHandle(UARTID id);
    CANHandle* getCANHandle(CANID id);
};

extern HardwareManager g_hardwareManager;

extern "C" {
    void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
    void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);
    void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
    void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
    void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
}

} // namespace HAL