#pragma once

#include "stm32f4xx_hal.h"
#include "../config/system_config.hpp"
#include <array>
#include <functional>

namespace HAL {

// Hardware handle types
struct TimerHandle {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    bool initialized;
    std::function<void()> errorCallback;
};

struct UARTHandle {
    UART_HandleTypeDef* huart;
    bool initialized;
    std::function<void(uint8_t*, size_t)> rxCallback;
    std::function<void()> txCompleteCallback;
    std::function<void()> errorCallback;
};

struct CANHandle {
    CAN_HandleTypeDef* hcan;
    bool initialized;
    std::function<void(CAN_RxHeaderTypeDef*, uint8_t*)> rxCallback;
    std::function<void()> txCompleteCallback;
    std::function<void()> errorCallback;
};

struct GPIOHandle {
    GPIO_TypeDef* port;
    uint16_t pin;
    GPIO_PinState activeState;
    std::function<void(bool)> changeCallback;
};

// Timer IDs for type-safe access
enum class TimerID : uint8_t {
    TIMER1 = 1,
    TIMER2 = 2,
    TIMER3 = 3,
    TIMER4 = 4,
    MAX_TIMERS = 4
};

enum class UARTID : uint8_t {
    UART_2 = 2,
    MAX_UARTS = 1
};

enum class CANID : uint8_t {
    CAN_1 = 1,
    MAX_CAN = 1
};

// Central hardware manager
class HardwareManager {
private:
    std::array<TimerHandle, static_cast<size_t>(TimerID::MAX_TIMERS)> timers_;
    std::array<UARTHandle, static_cast<size_t>(UARTID::MAX_UARTS)> uarts_;
    std::array<CANHandle, static_cast<size_t>(CANID::MAX_CAN)> can_;

    bool initialized_ = false;

public:
    // Initialization
    Config::Result<Config::ErrorCode> initialize();
    bool isInitialized() const { return initialized_; }

    // Timer management
    Config::Result<TimerHandle*> getTimer(TimerID id);
    Config::Result<Config::ErrorCode> startPWM(TimerID id, uint32_t channel);
    Config::Result<Config::ErrorCode> setPWMDutyCycle(TimerID id, uint32_t channel, uint32_t value);
    Config::Result<Config::ErrorCode> setTimerCallback(TimerID id, std::function<void()> callback);

    // UART management
    Config::Result<UARTHandle*> getUART(UARTID id);
    Config::Result<Config::ErrorCode> transmitUART(UARTID id, const uint8_t* data, size_t length);
    Config::Result<Config::ErrorCode> setUARTRxCallback(UARTID id, std::function<void(uint8_t*, size_t)> callback);

    // CAN management
    Config::Result<CANHandle*> getCAN(CANID id);
    Config::Result<Config::ErrorCode> transmitCAN(CANID id, uint32_t canId, const uint8_t* data, size_t length);
    Config::Result<Config::ErrorCode> setCANRxCallback(CANID id, std::function<void(CAN_RxHeaderTypeDef*, uint8_t*)> callback);

    // GPIO management
    Config::Result<bool> readGPIO(GPIO_TypeDef* port, uint16_t pin);
    Config::Result<Config::ErrorCode> writeGPIO(GPIO_TypeDef* port, uint16_t pin, bool state);
    Config::Result<Config::ErrorCode> setGPIOCallback(GPIO_TypeDef* port, uint16_t pin, std::function<void(bool)> callback);

    // System utilities
    uint32_t getSystemTick() const { return HAL_GetTick(); }
    void delay(uint32_t ms) { HAL_Delay(ms); }

    // Error handling
    void handleTimerError(TimerID id);
    void handleUARTError(UARTID id);
    void handleCANError(CANID id);

private:
    Config::Result<Config::ErrorCode> initializeTimers();
    Config::Result<Config::ErrorCode> initializeUARTs();
    Config::Result<Config::ErrorCode> initializeCAN();

    TimerHandle* getTimerHandle(TimerID id);
    UARTHandle* getUARTHandle(UARTID id);
    CANHandle* getCANHandle(CANID id);
};

// Global hardware manager instance
extern HardwareManager g_hardwareManager;

// Hardware interrupt handlers (to be called from STM32 HAL callbacks)
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