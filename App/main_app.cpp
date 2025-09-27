#include "system_context.hpp"
#include "config/motor_config.hpp"
#include "motors/base/motor_factory.hpp"
#include "comm/unified_mavlink_handler.hpp"

// External C dependencies from STM32 HAL
extern "C" {
    #include "main.h"

    // HAL handles from main.c
    extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
    extern UART_HandleTypeDef huart2;
    extern CAN_HandleTypeDef hcan1;
}

// Global system context
static System::SystemContext g_systemContext;

extern "C" {

// C-compatible entry points called from main.c
void cpp_setup() {
    // Initialize the system context
    auto result = g_systemContext.initialize();
    if (!result) {
        // Handle initialization failure
        g_systemContext.reportError(result.error(), "System initialization failed");

        // Fall back to emergency mode
        g_systemContext.setEmergencyStop(true);
        return;
    }

    // System successfully initialized
}

void cpp_loop() {
    // Update all subsystems
    auto result = g_systemContext.update();
    if (!result) {
        // Handle update failure
        g_systemContext.reportError(result.error(), "System update failed");

        // Continue running but log the error
        return;
    }

    // Optional: Add small delay to prevent overwhelming the system
    // HAL_Delay(1); // 1ms delay for 1000Hz loop
}

void cpp_emergency_stop() {
    // Emergency stop handler
    g_systemContext.setEmergencyStop(true);
}

void cpp_shutdown() {
    // Graceful shutdown
    g_systemContext.shutdown();
}

} // extern "C"