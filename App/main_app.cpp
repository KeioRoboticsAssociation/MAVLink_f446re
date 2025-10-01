/**
 * @file main_app.cpp
 * @brief Main application entry points and system context initialization.
 */

#include "system_context.hpp"
#include "config/motor_config.hpp"
#include "motors/base/motor_factory.hpp"
#include "comm/unified_mavlink_handler.hpp"

extern "C" {
    #include "main.h"

    extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
    extern UART_HandleTypeDef huart2;
    extern CAN_HandleTypeDef hcan1;
}

/**
 * @brief Global system context instance.
 */
static System::SystemContext g_systemContext;

extern "C" {

/**
 * @brief C-compatible setup function, called once at the beginning of the program.
 *
 * This function initializes the main system context, which in turn initializes all
 * subsystems like hardware, communication, and motors. If initialization fails,
 * it puts the system into an emergency stop state.
 */
void cpp_setup() {
    auto result = g_systemContext.initialize();
    if (!result) {
        g_systemContext.reportError(result.error(), "System initialization failed");
        g_systemContext.setEmergencyStop(true);
        return;
    }
}

/**
 * @brief C-compatible main loop function, called repeatedly.
 *
 * This function is the heart of the application, responsible for updating all
 * subsystems in each iteration of the main loop. It handles errors that may
 * occur during the update process and continues to run while logging the error.
 */
void cpp_loop() {
    auto result = g_systemContext.update();
    if (!result) {
        g_systemContext.reportError(result.error(), "System update failed");
        return;
    }
}

/**
 * @brief C-compatible emergency stop function.
 *
 * This function is called to trigger an immediate emergency stop of the system.
 * It sets the emergency stop flag in the system context, which will be handled
 * by the relevant subsystems.
 */
void cpp_emergency_stop() {
    g_systemContext.setEmergencyStop(true);
}

/**
 * @brief C-compatible shutdown function.
 *
 * This function is called to perform a graceful shutdown of the system.
 * It calls the shutdown method of the system context to de-initialize all
 * subsystems in an orderly manner.
 */
void cpp_shutdown() {
    g_systemContext.shutdown();
}

} // extern "C"