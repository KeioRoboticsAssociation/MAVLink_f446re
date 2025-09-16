#include "MAVLinkDCMotorController.hpp"

MAVLinkDCMotorController::MAVLinkDCMotorController()
    : motor_(nullptr), encoder_(nullptr), uart_(nullptr), system_id_(1), initialized_(false),
      last_telemetry_send_(0), last_control_update_(0), previous_position_rad_(0.0f),
      previous_position_time_(0), filtered_velocity_rad_s_(0.0f) {

    // Initialize MAVLink parsing
    rx_status_ = {};
    rx_msg_ = {};

    // Initialize state
    state_.status = MotorStatus::NOT_INITIALIZED;
    state_.enabled = false;
}

MotorStatus MAVLinkDCMotorController::init(uint8_t motor_id, DCMotor* motor, Encoder* encoder,
                                          UART_HandleTypeDef* uart, uint8_t system_id) {
    if (motor == nullptr || encoder == nullptr || uart == nullptr) {
        return MotorStatus::CONFIG_ERROR;
    }

    motor_ = motor;
    encoder_ = encoder;
    uart_ = uart;
    system_id_ = system_id;

    config_.motor_id = motor_id;
    state_.motor_id = motor_id;

    // Start motor PWM
    motor_->start();

    // Initialize timing
    uint32_t current_time = getCurrentTimeMs();
    state_.last_update_time = current_time;
    state_.last_command_time = current_time;
    last_control_update_ = current_time;
    last_telemetry_send_ = current_time;
    previous_position_time_ = current_time;

    // Initialize position tracking
    previous_position_rad_ = encoder_->getAngleRad();
    state_.current_position_rad = previous_position_rad_;

    initialized_ = true;
    state_.status = MotorStatus::OK;
    state_.mode = MotorControlMode::DISABLED;

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::setConfig(const MotorConfig& config) {
    if (!initialized_) {
        return MotorStatus::NOT_INITIALIZED;
    }

    // Validate config parameters
    if (config.speed_kp < 0 || config.speed_ki < 0 || config.speed_kd < 0 ||
        config.position_kp < 0 || config.position_ki < 0 || config.position_kd < 0) {
        return MotorStatus::CONFIG_ERROR;
    }

    config_ = config;
    state_.motor_id = config.motor_id;

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::setMode(MotorControlMode mode) {
    if (!initialized_) {
        return MotorStatus::NOT_INITIALIZED;
    }

    // Reset PID states when changing modes
    state_.speed_error_integral = 0.0f;
    state_.speed_error_previous = 0.0f;
    state_.position_error_integral = 0.0f;
    state_.position_error_previous = 0.0f;

    state_.mode = mode;
    state_.last_command_time = getCurrentTimeMs();

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::setTargetPosition(float position_rad) {
    if (!initialized_) {
        return MotorStatus::NOT_INITIALIZED;
    }

    // Check position limits if enabled
    if (config_.use_position_limits && !checkPositionLimits(position_rad)) {
        return MotorStatus::POSITION_LIMIT;
    }

    state_.target_position_rad = position_rad;
    state_.last_command_time = getCurrentTimeMs();

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::setTargetSpeed(float speed_rad_s) {
    if (!initialized_) {
        return MotorStatus::NOT_INITIALIZED;
    }

    // Constrain speed to limits
    state_.target_speed_rad_s = constrainFloat(speed_rad_s, -config_.max_speed_rad_s, config_.max_speed_rad_s);
    state_.last_command_time = getCurrentTimeMs();

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::setTargetDutyCycle(float duty_cycle) {
    if (!initialized_) {
        return MotorStatus::NOT_INITIALIZED;
    }

    state_.target_duty_cycle = constrainFloat(duty_cycle, -1.0f, 1.0f);
    state_.last_command_time = getCurrentTimeMs();

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::enable() {
    if (!initialized_) {
        return MotorStatus::NOT_INITIALIZED;
    }

    state_.enabled = true;
    state_.last_command_time = getCurrentTimeMs();

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::disable() {
    if (!initialized_) {
        return MotorStatus::NOT_INITIALIZED;
    }

    state_.enabled = false;
    motor_->setDuty(0.0f);
    state_.current_duty_cycle = 0.0f;

    return MotorStatus::OK;
}

MotorStatus MAVLinkDCMotorController::emergencyStop() {
    disable();
    state_.mode = MotorControlMode::DISABLED;

    return MotorStatus::OK;
}

void MAVLinkDCMotorController::update() {
    if (!initialized_) {
        return;
    }

    uint32_t current_time = getCurrentTimeMs();

    // Update encoder
    encoder_->update();

    // Update current position and velocity
    state_.current_position_rad = encoder_->getAngleRad();
    updateVelocityEstimate();

    // Check watchdog
    if (!checkWatchdog()) {
        handleTimeout();
        return;
    }

    // Run control loop at specified rate
    if (current_time - last_control_update_ >= config_.control_period_ms) {
        updateControlLoop();
        last_control_update_ = current_time;
    }

    // Send telemetry at specified rate
    if (current_time - last_telemetry_send_ >= TELEMETRY_SEND_RATE_MS) {
        sendTelemetry();
        last_telemetry_send_ = current_time;
    }

    state_.last_update_time = current_time;
}

void MAVLinkDCMotorController::updateControlLoop() {
    if (!state_.enabled) {
        motor_->setDuty(0.0f);
        state_.current_duty_cycle = 0.0f;
        return;
    }

    float duty_cycle = 0.0f;
    uint32_t current_time = getCurrentTimeMs();
    float dt = (current_time - state_.last_update_time) / 1000.0f; // Convert to seconds

    if (dt <= 0.0f) dt = config_.control_period_ms / 1000.0f; // Fallback

    switch (state_.mode) {
        case MotorControlMode::OPEN_LOOP:
            duty_cycle = state_.target_duty_cycle;
            break;

        case MotorControlMode::SPEED_CONTROL:
            duty_cycle = calculateSpeedPID(state_.target_speed_rad_s, state_.current_speed_rad_s, dt);
            break;

        case MotorControlMode::POSITION_CONTROL: {
            // Position control with velocity feedforward
            float target_velocity = calculatePositionPID(state_.target_position_rad, state_.current_position_rad, dt);
            target_velocity = constrainFloat(target_velocity, -config_.max_speed_rad_s, config_.max_speed_rad_s);
            duty_cycle = calculateSpeedPID(target_velocity, state_.current_speed_rad_s, dt);
            break;
        }

        case MotorControlMode::DISABLED:
        default:
            duty_cycle = 0.0f;
            break;
    }

    // Apply limits and set motor duty cycle
    applyDutyCycleLimit(duty_cycle);
    motor_->setDuty(duty_cycle);
    state_.current_duty_cycle = duty_cycle;
}

float MAVLinkDCMotorController::calculateSpeedPID(float target_speed, float current_speed, float dt) {
    float error = target_speed - current_speed;

    // Proportional term
    float p_term = config_.speed_kp * error;

    // Integral term with windup protection
    state_.speed_error_integral += error * dt;
    state_.speed_error_integral = constrainFloat(state_.speed_error_integral,
                                                -config_.speed_max_integral, config_.speed_max_integral);
    float i_term = config_.speed_ki * state_.speed_error_integral;

    // Derivative term
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = config_.speed_kd * (error - state_.speed_error_previous) / dt;
    }
    state_.speed_error_previous = error;

    float output = p_term + i_term + d_term;
    return constrainFloat(output, -config_.speed_max_output, config_.speed_max_output);
}

float MAVLinkDCMotorController::calculatePositionPID(float target_position, float current_position, float dt) {
    float error = wrapAngle(target_position - current_position);

    // Proportional term
    float p_term = config_.position_kp * error;

    // Integral term with windup protection
    state_.position_error_integral += error * dt;
    state_.position_error_integral = constrainFloat(state_.position_error_integral,
                                                   -config_.position_max_integral, config_.position_max_integral);
    float i_term = config_.position_ki * state_.position_error_integral;

    // Derivative term
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = config_.position_kd * (error - state_.position_error_previous) / dt;
    }
    state_.position_error_previous = error;

    float output = p_term + i_term + d_term;
    return constrainFloat(output, -config_.position_max_output, config_.position_max_output);
}

void MAVLinkDCMotorController::updateVelocityEstimate() {
    uint32_t current_time = getCurrentTimeMs();
    float dt = (current_time - previous_position_time_) / 1000.0f;

    if (dt > 0.001f) { // Minimum 1ms interval
        float current_position = state_.current_position_rad;
        float velocity = wrapAngle(current_position - previous_position_rad_) / dt;

        // Apply low-pass filter
        filtered_velocity_rad_s_ = VELOCITY_FILTER_ALPHA * filtered_velocity_rad_s_ +
                                  (1.0f - VELOCITY_FILTER_ALPHA) * velocity;

        state_.current_speed_rad_s = filtered_velocity_rad_s_;

        previous_position_rad_ = current_position;
        previous_position_time_ = current_time;
    }
}

void MAVLinkDCMotorController::applyDutyCycleLimit(float& duty_cycle) {
    duty_cycle = constrainFloat(duty_cycle, -1.0f, 1.0f);
}

bool MAVLinkDCMotorController::checkPositionLimits(float position_rad) {
    if (!config_.use_position_limits) {
        return true;
    }

    return (position_rad >= config_.position_limit_min_rad &&
            position_rad <= config_.position_limit_max_rad);
}

bool MAVLinkDCMotorController::checkWatchdog() {
    uint32_t current_time = getCurrentTimeMs();
    return (current_time - state_.last_command_time) < config_.watchdog_timeout_ms;
}

void MAVLinkDCMotorController::handleTimeout() {
    state_.timeout_count++;
    state_.status = MotorStatus::TIMEOUT;

    // Apply emergency stop
    handleEmergencyStop();
}

void MAVLinkDCMotorController::handleEmergencyStop() {
    state_.enabled = false;
    state_.mode = MotorControlMode::DISABLED;
    motor_->setDuty(0.0f);
    state_.current_duty_cycle = 0.0f;
}

void MAVLinkDCMotorController::processReceivedByte(uint8_t byte) {
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg_, &rx_status_)) {
        handleMessage(&rx_msg_);
    }
}

void MAVLinkDCMotorController::handleMessage(mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleMotorCommand(msg);
            break;

        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
            handlePositionTarget(msg);
            break;

        // Add more message handlers as needed
        default:
            break;
    }
}

void MAVLinkDCMotorController::handleMotorCommand(mavlink_message_t* msg) {
    mavlink_command_long_t command;
    mavlink_msg_command_long_decode(msg, &command);

    // Check if command is for this motor
    if (static_cast<uint8_t>(command.param1) != state_.motor_id) {
        return;
    }

    switch (command.command) {
        case 31010: // Custom motor enable command
            if (command.param2 > 0.5f) {
                enable();
            } else {
                disable();
            }
            break;

        case 31011: // Custom motor mode command
            setMode(static_cast<MotorControlMode>(static_cast<uint8_t>(command.param2)));
            break;

        case 31012: // Custom motor position command
            setTargetPosition(command.param2);
            break;

        case 31013: // Custom motor speed command
            setTargetSpeed(command.param2);
            break;

        case 31014: // Custom motor duty cycle command
            setTargetDutyCycle(command.param2);
            break;

        case 400: // MAV_CMD_COMPONENT_ARM_DISARM
            if (command.param1 > 0.5f) {
                enable();
            } else {
                emergencyStop();
            }
            break;
    }
}

void MAVLinkDCMotorController::handlePositionTarget(mavlink_message_t* msg) {
    mavlink_set_position_target_local_ned_t target;
    mavlink_msg_set_position_target_local_ned_decode(msg, &target);

    // Use yaw for motor position control
    if (!(target.type_mask & (1 << 10))) { // yaw position not ignored
        setTargetPosition(target.yaw);
        if (state_.mode != MotorControlMode::POSITION_CONTROL) {
            setMode(MotorControlMode::POSITION_CONTROL);
        }
    }
}

void MAVLinkDCMotorController::sendTelemetry() {
    sendMotorStatus();
    sendPositionFeedback();
    sendSpeedFeedback();
}

void MAVLinkDCMotorController::sendMotorStatus() {
    mavlink_message_t msg;

    // Send custom motor status message (using SERVO_OUTPUT_RAW as template)
    mavlink_msg_servo_output_raw_pack(
        system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
        getCurrentTimeMs(),                                               // time_usec
        static_cast<uint8_t>(state_.motor_id),                           // port
        static_cast<uint16_t>((state_.current_duty_cycle + 1.0f) * 1000), // servo1_raw (current duty)
        static_cast<uint16_t>((state_.target_duty_cycle + 1.0f) * 1000),  // servo2_raw (target duty)
        static_cast<uint16_t>(state_.enabled ? 2000 : 1000),             // servo3_raw (enabled)
        static_cast<uint16_t>(static_cast<uint8_t>(state_.mode) * 500),   // servo4_raw (mode)
        static_cast<uint16_t>(static_cast<uint8_t>(state_.status) * 200), // servo5_raw (status)
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0                                  // servo6-16_raw
    );

    sendMessage(&msg);
}

void MAVLinkDCMotorController::sendPositionFeedback() {
    mavlink_message_t msg;

    // Send position data using ATTITUDE message
    mavlink_msg_attitude_pack(
        system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
        getCurrentTimeMs(),
        state_.current_position_rad,  // roll = current position
        state_.target_position_rad,   // pitch = target position
        0.0f,                        // yaw (unused)
        state_.current_speed_rad_s,  // rollspeed = current velocity
        0.0f,                        // pitchspeed (unused)
        0.0f                         // yawspeed (unused)
    );

    sendMessage(&msg);
}

void MAVLinkDCMotorController::sendSpeedFeedback() {
    mavlink_message_t msg;

    // Send speed data using LOCAL_POSITION_NED message
    mavlink_msg_local_position_ned_pack(
        system_id_, MAV_COMP_ID_AUTOPILOT1, &msg,
        getCurrentTimeMs(),
        state_.current_speed_rad_s,    // x = current speed
        state_.target_speed_rad_s,     // y = target speed
        0.0f,                         // z (unused)
        0.0f,                         // vx (unused)
        0.0f,                         // vy (unused)
        0.0f                          // vz (unused)
    );

    sendMessage(&msg);
}

void MAVLinkDCMotorController::sendMessage(mavlink_message_t* msg) {
    if (uart_ == nullptr) {
        return;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);

    HAL_UART_Transmit(uart_, buffer, len, 100);
}

uint32_t MAVLinkDCMotorController::getCurrentTimeMs() const {
    return HAL_GetTick();
}

float MAVLinkDCMotorController::constrainFloat(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

float MAVLinkDCMotorController::wrapAngle(float angle_rad) {
    while (angle_rad > M_PI) angle_rad -= 2.0f * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2.0f * M_PI;
    return angle_rad;
}