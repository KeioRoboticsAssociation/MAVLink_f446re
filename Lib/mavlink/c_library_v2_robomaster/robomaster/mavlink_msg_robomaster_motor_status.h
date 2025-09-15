#pragma once
// MESSAGE ROBOMASTER_MOTOR_STATUS PACKING

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS 181


typedef struct __mavlink_robomaster_motor_status_t {
 float current_position; /*<  Current position in radians*/
 float current_velocity; /*<  Current velocity in RPS*/
 float target_position; /*<  Target position in radians*/
 float target_velocity; /*<  Target velocity in RPS*/
 uint32_t last_command_time; /*<  Time since last command (ms)*/
 uint32_t last_feedback_time; /*<  Time since last feedback (ms)*/
 int16_t current_milliamps; /*<  Current in milliamps*/
 int16_t target_current; /*<  Target current in milliamps*/
 uint16_t error_count; /*<  Total error count*/
 uint16_t timeout_count; /*<  Total timeout count*/
 uint16_t overheat_count; /*<  Total overheat count*/
 uint8_t motor_id; /*<  Motor ID (1-8)*/
 uint8_t temperature; /*<  Temperature in Celsius*/
 uint8_t control_mode; /*<  Current control mode*/
 uint8_t enabled; /*<  Motor enabled status (0=disabled, 1=enabled)*/
 uint8_t status; /*<  Motor status code*/
} mavlink_robomaster_motor_status_t;

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN 39
#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN 39
#define MAVLINK_MSG_ID_181_LEN 39
#define MAVLINK_MSG_ID_181_MIN_LEN 39

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC 168
#define MAVLINK_MSG_ID_181_CRC 168



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_STATUS { \
    181, \
    "ROBOMASTER_MOTOR_STATUS", \
    16, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_robomaster_motor_status_t, motor_id) }, \
         { "current_position", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_status_t, current_position) }, \
         { "current_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_motor_status_t, current_velocity) }, \
         { "current_milliamps", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_robomaster_motor_status_t, current_milliamps) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_robomaster_motor_status_t, temperature) }, \
         { "target_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robomaster_motor_status_t, target_position) }, \
         { "target_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_robomaster_motor_status_t, target_velocity) }, \
         { "target_current", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_robomaster_motor_status_t, target_current) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_robomaster_motor_status_t, control_mode) }, \
         { "enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_robomaster_motor_status_t, enabled) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_robomaster_motor_status_t, status) }, \
         { "error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_robomaster_motor_status_t, error_count) }, \
         { "timeout_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_robomaster_motor_status_t, timeout_count) }, \
         { "overheat_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_robomaster_motor_status_t, overheat_count) }, \
         { "last_command_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_robomaster_motor_status_t, last_command_time) }, \
         { "last_feedback_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_robomaster_motor_status_t, last_feedback_time) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_STATUS { \
    "ROBOMASTER_MOTOR_STATUS", \
    16, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_robomaster_motor_status_t, motor_id) }, \
         { "current_position", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_status_t, current_position) }, \
         { "current_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_motor_status_t, current_velocity) }, \
         { "current_milliamps", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_robomaster_motor_status_t, current_milliamps) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_robomaster_motor_status_t, temperature) }, \
         { "target_position", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robomaster_motor_status_t, target_position) }, \
         { "target_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_robomaster_motor_status_t, target_velocity) }, \
         { "target_current", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_robomaster_motor_status_t, target_current) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_robomaster_motor_status_t, control_mode) }, \
         { "enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_robomaster_motor_status_t, enabled) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_robomaster_motor_status_t, status) }, \
         { "error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_robomaster_motor_status_t, error_count) }, \
         { "timeout_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_robomaster_motor_status_t, timeout_count) }, \
         { "overheat_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_robomaster_motor_status_t, overheat_count) }, \
         { "last_command_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_robomaster_motor_status_t, last_command_time) }, \
         { "last_feedback_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_robomaster_motor_status_t, last_feedback_time) }, \
         } \
}
#endif

/**
 * @brief Pack a robomaster_motor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-8)
 * @param current_position  Current position in radians
 * @param current_velocity  Current velocity in RPS
 * @param current_milliamps  Current in milliamps
 * @param temperature  Temperature in Celsius
 * @param target_position  Target position in radians
 * @param target_velocity  Target velocity in RPS
 * @param target_current  Target current in milliamps
 * @param control_mode  Current control mode
 * @param enabled  Motor enabled status (0=disabled, 1=enabled)
 * @param status  Motor status code
 * @param error_count  Total error count
 * @param timeout_count  Total timeout count
 * @param overheat_count  Total overheat count
 * @param last_command_time  Time since last command (ms)
 * @param last_feedback_time  Time since last feedback (ms)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, float current_position, float current_velocity, int16_t current_milliamps, uint8_t temperature, float target_position, float target_velocity, int16_t target_current, uint8_t control_mode, uint8_t enabled, uint8_t status, uint16_t error_count, uint16_t timeout_count, uint16_t overheat_count, uint32_t last_command_time, uint32_t last_feedback_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position);
    _mav_put_float(buf, 4, current_velocity);
    _mav_put_float(buf, 8, target_position);
    _mav_put_float(buf, 12, target_velocity);
    _mav_put_uint32_t(buf, 16, last_command_time);
    _mav_put_uint32_t(buf, 20, last_feedback_time);
    _mav_put_int16_t(buf, 24, current_milliamps);
    _mav_put_int16_t(buf, 26, target_current);
    _mav_put_uint16_t(buf, 28, error_count);
    _mav_put_uint16_t(buf, 30, timeout_count);
    _mav_put_uint16_t(buf, 32, overheat_count);
    _mav_put_uint8_t(buf, 34, motor_id);
    _mav_put_uint8_t(buf, 35, temperature);
    _mav_put_uint8_t(buf, 36, control_mode);
    _mav_put_uint8_t(buf, 37, enabled);
    _mav_put_uint8_t(buf, 38, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position = current_position;
    packet.current_velocity = current_velocity;
    packet.target_position = target_position;
    packet.target_velocity = target_velocity;
    packet.last_command_time = last_command_time;
    packet.last_feedback_time = last_feedback_time;
    packet.current_milliamps = current_milliamps;
    packet.target_current = target_current;
    packet.error_count = error_count;
    packet.timeout_count = timeout_count;
    packet.overheat_count = overheat_count;
    packet.motor_id = motor_id;
    packet.temperature = temperature;
    packet.control_mode = control_mode;
    packet.enabled = enabled;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
}

/**
 * @brief Pack a robomaster_motor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-8)
 * @param current_position  Current position in radians
 * @param current_velocity  Current velocity in RPS
 * @param current_milliamps  Current in milliamps
 * @param temperature  Temperature in Celsius
 * @param target_position  Target position in radians
 * @param target_velocity  Target velocity in RPS
 * @param target_current  Target current in milliamps
 * @param control_mode  Current control mode
 * @param enabled  Motor enabled status (0=disabled, 1=enabled)
 * @param status  Motor status code
 * @param error_count  Total error count
 * @param timeout_count  Total timeout count
 * @param overheat_count  Total overheat count
 * @param last_command_time  Time since last command (ms)
 * @param last_feedback_time  Time since last feedback (ms)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, float current_position, float current_velocity, int16_t current_milliamps, uint8_t temperature, float target_position, float target_velocity, int16_t target_current, uint8_t control_mode, uint8_t enabled, uint8_t status, uint16_t error_count, uint16_t timeout_count, uint16_t overheat_count, uint32_t last_command_time, uint32_t last_feedback_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position);
    _mav_put_float(buf, 4, current_velocity);
    _mav_put_float(buf, 8, target_position);
    _mav_put_float(buf, 12, target_velocity);
    _mav_put_uint32_t(buf, 16, last_command_time);
    _mav_put_uint32_t(buf, 20, last_feedback_time);
    _mav_put_int16_t(buf, 24, current_milliamps);
    _mav_put_int16_t(buf, 26, target_current);
    _mav_put_uint16_t(buf, 28, error_count);
    _mav_put_uint16_t(buf, 30, timeout_count);
    _mav_put_uint16_t(buf, 32, overheat_count);
    _mav_put_uint8_t(buf, 34, motor_id);
    _mav_put_uint8_t(buf, 35, temperature);
    _mav_put_uint8_t(buf, 36, control_mode);
    _mav_put_uint8_t(buf, 37, enabled);
    _mav_put_uint8_t(buf, 38, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position = current_position;
    packet.current_velocity = current_velocity;
    packet.target_position = target_position;
    packet.target_velocity = target_velocity;
    packet.last_command_time = last_command_time;
    packet.last_feedback_time = last_feedback_time;
    packet.current_milliamps = current_milliamps;
    packet.target_current = target_current;
    packet.error_count = error_count;
    packet.timeout_count = timeout_count;
    packet.overheat_count = overheat_count;
    packet.motor_id = motor_id;
    packet.temperature = temperature;
    packet.control_mode = control_mode;
    packet.enabled = enabled;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#endif
}

/**
 * @brief Pack a robomaster_motor_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  Motor ID (1-8)
 * @param current_position  Current position in radians
 * @param current_velocity  Current velocity in RPS
 * @param current_milliamps  Current in milliamps
 * @param temperature  Temperature in Celsius
 * @param target_position  Target position in radians
 * @param target_velocity  Target velocity in RPS
 * @param target_current  Target current in milliamps
 * @param control_mode  Current control mode
 * @param enabled  Motor enabled status (0=disabled, 1=enabled)
 * @param status  Motor status code
 * @param error_count  Total error count
 * @param timeout_count  Total timeout count
 * @param overheat_count  Total overheat count
 * @param last_command_time  Time since last command (ms)
 * @param last_feedback_time  Time since last feedback (ms)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,float current_position,float current_velocity,int16_t current_milliamps,uint8_t temperature,float target_position,float target_velocity,int16_t target_current,uint8_t control_mode,uint8_t enabled,uint8_t status,uint16_t error_count,uint16_t timeout_count,uint16_t overheat_count,uint32_t last_command_time,uint32_t last_feedback_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position);
    _mav_put_float(buf, 4, current_velocity);
    _mav_put_float(buf, 8, target_position);
    _mav_put_float(buf, 12, target_velocity);
    _mav_put_uint32_t(buf, 16, last_command_time);
    _mav_put_uint32_t(buf, 20, last_feedback_time);
    _mav_put_int16_t(buf, 24, current_milliamps);
    _mav_put_int16_t(buf, 26, target_current);
    _mav_put_uint16_t(buf, 28, error_count);
    _mav_put_uint16_t(buf, 30, timeout_count);
    _mav_put_uint16_t(buf, 32, overheat_count);
    _mav_put_uint8_t(buf, 34, motor_id);
    _mav_put_uint8_t(buf, 35, temperature);
    _mav_put_uint8_t(buf, 36, control_mode);
    _mav_put_uint8_t(buf, 37, enabled);
    _mav_put_uint8_t(buf, 38, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position = current_position;
    packet.current_velocity = current_velocity;
    packet.target_position = target_position;
    packet.target_velocity = target_velocity;
    packet.last_command_time = last_command_time;
    packet.last_feedback_time = last_feedback_time;
    packet.current_milliamps = current_milliamps;
    packet.target_current = target_current;
    packet.error_count = error_count;
    packet.timeout_count = timeout_count;
    packet.overheat_count = overheat_count;
    packet.motor_id = motor_id;
    packet.temperature = temperature;
    packet.control_mode = control_mode;
    packet.enabled = enabled;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
}

/**
 * @brief Encode a robomaster_motor_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robomaster_motor_status_t* robomaster_motor_status)
{
    return mavlink_msg_robomaster_motor_status_pack(system_id, component_id, msg, robomaster_motor_status->motor_id, robomaster_motor_status->current_position, robomaster_motor_status->current_velocity, robomaster_motor_status->current_milliamps, robomaster_motor_status->temperature, robomaster_motor_status->target_position, robomaster_motor_status->target_velocity, robomaster_motor_status->target_current, robomaster_motor_status->control_mode, robomaster_motor_status->enabled, robomaster_motor_status->status, robomaster_motor_status->error_count, robomaster_motor_status->timeout_count, robomaster_motor_status->overheat_count, robomaster_motor_status->last_command_time, robomaster_motor_status->last_feedback_time);
}

/**
 * @brief Encode a robomaster_motor_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robomaster_motor_status_t* robomaster_motor_status)
{
    return mavlink_msg_robomaster_motor_status_pack_chan(system_id, component_id, chan, msg, robomaster_motor_status->motor_id, robomaster_motor_status->current_position, robomaster_motor_status->current_velocity, robomaster_motor_status->current_milliamps, robomaster_motor_status->temperature, robomaster_motor_status->target_position, robomaster_motor_status->target_velocity, robomaster_motor_status->target_current, robomaster_motor_status->control_mode, robomaster_motor_status->enabled, robomaster_motor_status->status, robomaster_motor_status->error_count, robomaster_motor_status->timeout_count, robomaster_motor_status->overheat_count, robomaster_motor_status->last_command_time, robomaster_motor_status->last_feedback_time);
}

/**
 * @brief Encode a robomaster_motor_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_robomaster_motor_status_t* robomaster_motor_status)
{
    return mavlink_msg_robomaster_motor_status_pack_status(system_id, component_id, _status, msg,  robomaster_motor_status->motor_id, robomaster_motor_status->current_position, robomaster_motor_status->current_velocity, robomaster_motor_status->current_milliamps, robomaster_motor_status->temperature, robomaster_motor_status->target_position, robomaster_motor_status->target_velocity, robomaster_motor_status->target_current, robomaster_motor_status->control_mode, robomaster_motor_status->enabled, robomaster_motor_status->status, robomaster_motor_status->error_count, robomaster_motor_status->timeout_count, robomaster_motor_status->overheat_count, robomaster_motor_status->last_command_time, robomaster_motor_status->last_feedback_time);
}

/**
 * @brief Send a robomaster_motor_status message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (1-8)
 * @param current_position  Current position in radians
 * @param current_velocity  Current velocity in RPS
 * @param current_milliamps  Current in milliamps
 * @param temperature  Temperature in Celsius
 * @param target_position  Target position in radians
 * @param target_velocity  Target velocity in RPS
 * @param target_current  Target current in milliamps
 * @param control_mode  Current control mode
 * @param enabled  Motor enabled status (0=disabled, 1=enabled)
 * @param status  Motor status code
 * @param error_count  Total error count
 * @param timeout_count  Total timeout count
 * @param overheat_count  Total overheat count
 * @param last_command_time  Time since last command (ms)
 * @param last_feedback_time  Time since last feedback (ms)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robomaster_motor_status_send(mavlink_channel_t chan, uint8_t motor_id, float current_position, float current_velocity, int16_t current_milliamps, uint8_t temperature, float target_position, float target_velocity, int16_t target_current, uint8_t control_mode, uint8_t enabled, uint8_t status, uint16_t error_count, uint16_t timeout_count, uint16_t overheat_count, uint32_t last_command_time, uint32_t last_feedback_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position);
    _mav_put_float(buf, 4, current_velocity);
    _mav_put_float(buf, 8, target_position);
    _mav_put_float(buf, 12, target_velocity);
    _mav_put_uint32_t(buf, 16, last_command_time);
    _mav_put_uint32_t(buf, 20, last_feedback_time);
    _mav_put_int16_t(buf, 24, current_milliamps);
    _mav_put_int16_t(buf, 26, target_current);
    _mav_put_uint16_t(buf, 28, error_count);
    _mav_put_uint16_t(buf, 30, timeout_count);
    _mav_put_uint16_t(buf, 32, overheat_count);
    _mav_put_uint8_t(buf, 34, motor_id);
    _mav_put_uint8_t(buf, 35, temperature);
    _mav_put_uint8_t(buf, 36, control_mode);
    _mav_put_uint8_t(buf, 37, enabled);
    _mav_put_uint8_t(buf, 38, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position = current_position;
    packet.current_velocity = current_velocity;
    packet.target_position = target_position;
    packet.target_velocity = target_velocity;
    packet.last_command_time = last_command_time;
    packet.last_feedback_time = last_feedback_time;
    packet.current_milliamps = current_milliamps;
    packet.target_current = target_current;
    packet.error_count = error_count;
    packet.timeout_count = timeout_count;
    packet.overheat_count = overheat_count;
    packet.motor_id = motor_id;
    packet.temperature = temperature;
    packet.control_mode = control_mode;
    packet.enabled = enabled;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a robomaster_motor_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robomaster_motor_status_send_struct(mavlink_channel_t chan, const mavlink_robomaster_motor_status_t* robomaster_motor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robomaster_motor_status_send(chan, robomaster_motor_status->motor_id, robomaster_motor_status->current_position, robomaster_motor_status->current_velocity, robomaster_motor_status->current_milliamps, robomaster_motor_status->temperature, robomaster_motor_status->target_position, robomaster_motor_status->target_velocity, robomaster_motor_status->target_current, robomaster_motor_status->control_mode, robomaster_motor_status->enabled, robomaster_motor_status->status, robomaster_motor_status->error_count, robomaster_motor_status->timeout_count, robomaster_motor_status->overheat_count, robomaster_motor_status->last_command_time, robomaster_motor_status->last_feedback_time);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS, (const char *)robomaster_motor_status, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robomaster_motor_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, float current_position, float current_velocity, int16_t current_milliamps, uint8_t temperature, float target_position, float target_velocity, int16_t target_current, uint8_t control_mode, uint8_t enabled, uint8_t status, uint16_t error_count, uint16_t timeout_count, uint16_t overheat_count, uint32_t last_command_time, uint32_t last_feedback_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, current_position);
    _mav_put_float(buf, 4, current_velocity);
    _mav_put_float(buf, 8, target_position);
    _mav_put_float(buf, 12, target_velocity);
    _mav_put_uint32_t(buf, 16, last_command_time);
    _mav_put_uint32_t(buf, 20, last_feedback_time);
    _mav_put_int16_t(buf, 24, current_milliamps);
    _mav_put_int16_t(buf, 26, target_current);
    _mav_put_uint16_t(buf, 28, error_count);
    _mav_put_uint16_t(buf, 30, timeout_count);
    _mav_put_uint16_t(buf, 32, overheat_count);
    _mav_put_uint8_t(buf, 34, motor_id);
    _mav_put_uint8_t(buf, 35, temperature);
    _mav_put_uint8_t(buf, 36, control_mode);
    _mav_put_uint8_t(buf, 37, enabled);
    _mav_put_uint8_t(buf, 38, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#else
    mavlink_robomaster_motor_status_t *packet = (mavlink_robomaster_motor_status_t *)msgbuf;
    packet->current_position = current_position;
    packet->current_velocity = current_velocity;
    packet->target_position = target_position;
    packet->target_velocity = target_velocity;
    packet->last_command_time = last_command_time;
    packet->last_feedback_time = last_feedback_time;
    packet->current_milliamps = current_milliamps;
    packet->target_current = target_current;
    packet->error_count = error_count;
    packet->timeout_count = timeout_count;
    packet->overheat_count = overheat_count;
    packet->motor_id = motor_id;
    packet->temperature = temperature;
    packet->control_mode = control_mode;
    packet->enabled = enabled;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOMASTER_MOTOR_STATUS UNPACKING


/**
 * @brief Get field motor_id from robomaster_motor_status message
 *
 * @return  Motor ID (1-8)
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field current_position from robomaster_motor_status message
 *
 * @return  Current position in radians
 */
static inline float mavlink_msg_robomaster_motor_status_get_current_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field current_velocity from robomaster_motor_status message
 *
 * @return  Current velocity in RPS
 */
static inline float mavlink_msg_robomaster_motor_status_get_current_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field current_milliamps from robomaster_motor_status message
 *
 * @return  Current in milliamps
 */
static inline int16_t mavlink_msg_robomaster_motor_status_get_current_milliamps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field temperature from robomaster_motor_status message
 *
 * @return  Temperature in Celsius
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field target_position from robomaster_motor_status message
 *
 * @return  Target position in radians
 */
static inline float mavlink_msg_robomaster_motor_status_get_target_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field target_velocity from robomaster_motor_status message
 *
 * @return  Target velocity in RPS
 */
static inline float mavlink_msg_robomaster_motor_status_get_target_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field target_current from robomaster_motor_status message
 *
 * @return  Target current in milliamps
 */
static inline int16_t mavlink_msg_robomaster_motor_status_get_target_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field control_mode from robomaster_motor_status message
 *
 * @return  Current control mode
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field enabled from robomaster_motor_status message
 *
 * @return  Motor enabled status (0=disabled, 1=enabled)
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field status from robomaster_motor_status message
 *
 * @return  Motor status code
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field error_count from robomaster_motor_status message
 *
 * @return  Total error count
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_get_error_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field timeout_count from robomaster_motor_status message
 *
 * @return  Total timeout count
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_get_timeout_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field overheat_count from robomaster_motor_status message
 *
 * @return  Total overheat count
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_get_overheat_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field last_command_time from robomaster_motor_status message
 *
 * @return  Time since last command (ms)
 */
static inline uint32_t mavlink_msg_robomaster_motor_status_get_last_command_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field last_feedback_time from robomaster_motor_status message
 *
 * @return  Time since last feedback (ms)
 */
static inline uint32_t mavlink_msg_robomaster_motor_status_get_last_feedback_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Decode a robomaster_motor_status message into a struct
 *
 * @param msg The message to decode
 * @param robomaster_motor_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_robomaster_motor_status_decode(const mavlink_message_t* msg, mavlink_robomaster_motor_status_t* robomaster_motor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robomaster_motor_status->current_position = mavlink_msg_robomaster_motor_status_get_current_position(msg);
    robomaster_motor_status->current_velocity = mavlink_msg_robomaster_motor_status_get_current_velocity(msg);
    robomaster_motor_status->target_position = mavlink_msg_robomaster_motor_status_get_target_position(msg);
    robomaster_motor_status->target_velocity = mavlink_msg_robomaster_motor_status_get_target_velocity(msg);
    robomaster_motor_status->last_command_time = mavlink_msg_robomaster_motor_status_get_last_command_time(msg);
    robomaster_motor_status->last_feedback_time = mavlink_msg_robomaster_motor_status_get_last_feedback_time(msg);
    robomaster_motor_status->current_milliamps = mavlink_msg_robomaster_motor_status_get_current_milliamps(msg);
    robomaster_motor_status->target_current = mavlink_msg_robomaster_motor_status_get_target_current(msg);
    robomaster_motor_status->error_count = mavlink_msg_robomaster_motor_status_get_error_count(msg);
    robomaster_motor_status->timeout_count = mavlink_msg_robomaster_motor_status_get_timeout_count(msg);
    robomaster_motor_status->overheat_count = mavlink_msg_robomaster_motor_status_get_overheat_count(msg);
    robomaster_motor_status->motor_id = mavlink_msg_robomaster_motor_status_get_motor_id(msg);
    robomaster_motor_status->temperature = mavlink_msg_robomaster_motor_status_get_temperature(msg);
    robomaster_motor_status->control_mode = mavlink_msg_robomaster_motor_status_get_control_mode(msg);
    robomaster_motor_status->enabled = mavlink_msg_robomaster_motor_status_get_enabled(msg);
    robomaster_motor_status->status = mavlink_msg_robomaster_motor_status_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN;
        memset(robomaster_motor_status, 0, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
    memcpy(robomaster_motor_status, _MAV_PAYLOAD(msg), len);
#endif
}
