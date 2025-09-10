#pragma once

/**
 * @file RoboMasterController.hpp
 * @brief Legacy RoboMaster Controller (DEPRECATED)
 * 
 * @deprecated This is the legacy monolithic RoboMaster controller.
 * 
 * For new projects, please use the improved individual motor control API:
 * - RoboMasterMotor.hpp - Individual motor control with full features
 * - RoboMasterCANManager.hpp - CAN communication manager
 * 
 * The new API provides:
 * - Individual motor configuration and control
 * - Better error handling and safety features
 * - Configurable PID gains per motor
 * - Position/velocity/current control modes
 * - Watchdog and thermal protection
 * - Rate limiting and saturation handling
 * 
 * See Examples/example_usage.cpp for migration guide.
 */

#include "stm32f4xx_hal.h"
#include <stm32f446xx.h>

// Include new API for users who want to migrate
#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"

namespace RoboMasterValues {
    enum ID {
        FirstFour = 0x200,
        SecondFour = 0x1FF,  // Fixed: was 0x144, should be 0x1FF for motors 5-8
        ReceiveZeroPoint = 0x201,
    };

    enum class ControlMode {
        Current = 0,
        Velocity = 1,
        Position = 2
    };

    const int RESOLUTION = 8192;
};

typedef struct {
    float pos_radian_absolute = 0;
    float pos_radian_estimated = 0;
    float vel_rps = 0;
    uint16_t current = 0;
    uint8_t temperature = 0;
} receive_data_t;

typedef struct {
    RoboMasterValues::ControlMode mode = RoboMasterValues::ControlMode::Velocity;
    float pos_radian = 0;
    float vel_rps = 0;
    uint16_t current = 0;
} target_data_t;

typedef struct {
    float speed_kp = 35.0f;
    float speed_ki = 0.15f;
    float pos_kp = 1.0f;
} pid_gain_t;

/**
 * @brief Legacy RoboMaster Controller Class
 * @deprecated Use RoboMasterMotor and RoboMasterCANManager instead
 * 
 * This class is maintained for backward compatibility but lacks many
 * features of the new individual motor API:
 * - No individual motor configuration
 * - Limited error handling
 * - No safety features (thermal protection, watchdog, etc.)
 * - Fixed PID implementation
 * - No rate limiting
 * 
 * Consider migrating to the new API for better performance and features.
 */
class [[deprecated("Use RoboMasterMotor and RoboMasterCANManager instead")]] RoboMasterController {
    public:
        RoboMasterController(CAN_HandleTypeDef* hcan_ptr, bool disable_first_four = false, bool disable_second_four = true);
        void start();
        void send(uint32_t id, uint8_t* data, uint32_t size);
        void send_force(uint32_t id, uint8_t* data, uint32_t size);

        void timer_callback();

        void setCurrentFirstFour(int16_t current0, int16_t current1, int16_t current2, int16_t current3);
        void setCurrentSecondFour(int16_t current4, int16_t current5, int16_t current6, int16_t current7);

        void onReceive(uint8_t id, uint8_t* data, uint8_t size);

        void setPIDGain(uint8_t id, float speed_kp, float pos_kp);
        void setControlMode(uint8_t id, RoboMasterValues::ControlMode mode);
        void setTargetSpeed(uint8_t id, float velocity);
        void setTargetPosition(uint8_t id, float position);
        void setTargetCurrent(uint8_t id, int16_t current);

        bool timer_flag = false;

        receive_data_t receive_data[8];

        bool disable_first_four, disable_second_four;
    private:
        CAN_HandleTypeDef* hcan_ptr;
        target_data_t target[8];
        pid_gain_t pid_gain[8];
        float speed_error_integral[8] = {0};
        int pos_counts[8] = {0};
        float initial_pos[8] = {0};
        bool first_receive[8] = {false};
};