#pragma once
// MESSAGE ROBOMASTER_MOTOR_STATUS PACKING

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS 12001


typedef struct __mavlink_robomaster_motor_status_t {
 float current_position_rad; /*<  Current position in radians*/
 float current_speed_rad_s; /*<  Current speed in rad/s*/
 float current_duty_cycle; /*<  Current duty cycle (-1.0 to 1.0)*/
 float position_error_rad; /*<  Position error in radians (target - current)*/
 uint32_t timestamp_ms; /*<  Timestamp in milliseconds*/
 uint8_t motor_id; /*<  Motor ID (1-255)*/
 uint8_t control_mode; /*<  Current control mode (see MOTOR_CONTROL_MODE enum)*/
 uint8_t status; /*<  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)*/
} mavlink_robomaster_motor_status_t;

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN 23
#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN 23
#define MAVLINK_MSG_ID_12001_LEN 23
#define MAVLINK_MSG_ID_12001_MIN_LEN 23

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC 168
#define MAVLINK_MSG_ID_12001_CRC 168



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_STATUS { \
    12001, \
    "ROBOMASTER_MOTOR_STATUS", \
    8, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_robomaster_motor_status_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_robomaster_motor_status_t, control_mode) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_robomaster_motor_status_t, status) }, \
         { "current_position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_status_t, current_position_rad) }, \
         { "current_speed_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_motor_status_t, current_speed_rad_s) }, \
         { "current_duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robomaster_motor_status_t, current_duty_cycle) }, \
         { "position_error_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_robomaster_motor_status_t, position_error_rad) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_robomaster_motor_status_t, timestamp_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_STATUS { \
    "ROBOMASTER_MOTOR_STATUS", \
    8, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_robomaster_motor_status_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_robomaster_motor_status_t, control_mode) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_robomaster_motor_status_t, status) }, \
         { "current_position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_status_t, current_position_rad) }, \
         { "current_speed_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_motor_status_t, current_speed_rad_s) }, \
         { "current_duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robomaster_motor_status_t, current_duty_cycle) }, \
         { "position_error_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_robomaster_motor_status_t, position_error_rad) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_robomaster_motor_status_t, timestamp_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a robomaster_motor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (see MOTOR_CONTROL_MODE enum)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param current_position_rad  Current position in radians
 * @param current_speed_rad_s  Current speed in rad/s
 * @param current_duty_cycle  Current duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (target - current)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, uint8_t status, float current_position_rad, float current_speed_rad_s, float current_duty_cycle, float position_error_rad, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rad);
    _mav_put_float(buf, 4, current_speed_rad_s);
    _mav_put_float(buf, 8, current_duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, control_mode);
    _mav_put_uint8_t(buf, 22, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position_rad = current_position_rad;
    packet.current_speed_rad_s = current_speed_rad_s;
    packet.current_duty_cycle = current_duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
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
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (see MOTOR_CONTROL_MODE enum)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param current_position_rad  Current position in radians
 * @param current_speed_rad_s  Current speed in rad/s
 * @param current_duty_cycle  Current duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (target - current)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, uint8_t status, float current_position_rad, float current_speed_rad_s, float current_duty_cycle, float position_error_rad, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rad);
    _mav_put_float(buf, 4, current_speed_rad_s);
    _mav_put_float(buf, 8, current_duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, control_mode);
    _mav_put_uint8_t(buf, 22, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position_rad = current_position_rad;
    packet.current_speed_rad_s = current_speed_rad_s;
    packet.current_duty_cycle = current_duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
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
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (see MOTOR_CONTROL_MODE enum)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param current_position_rad  Current position in radians
 * @param current_speed_rad_s  Current speed in rad/s
 * @param current_duty_cycle  Current duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (target - current)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint8_t control_mode,uint8_t status,float current_position_rad,float current_speed_rad_s,float current_duty_cycle,float position_error_rad,uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rad);
    _mav_put_float(buf, 4, current_speed_rad_s);
    _mav_put_float(buf, 8, current_duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, control_mode);
    _mav_put_uint8_t(buf, 22, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position_rad = current_position_rad;
    packet.current_speed_rad_s = current_speed_rad_s;
    packet.current_duty_cycle = current_duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
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
    return mavlink_msg_robomaster_motor_status_pack(system_id, component_id, msg, robomaster_motor_status->motor_id, robomaster_motor_status->control_mode, robomaster_motor_status->status, robomaster_motor_status->current_position_rad, robomaster_motor_status->current_speed_rad_s, robomaster_motor_status->current_duty_cycle, robomaster_motor_status->position_error_rad, robomaster_motor_status->timestamp_ms);
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
    return mavlink_msg_robomaster_motor_status_pack_chan(system_id, component_id, chan, msg, robomaster_motor_status->motor_id, robomaster_motor_status->control_mode, robomaster_motor_status->status, robomaster_motor_status->current_position_rad, robomaster_motor_status->current_speed_rad_s, robomaster_motor_status->current_duty_cycle, robomaster_motor_status->position_error_rad, robomaster_motor_status->timestamp_ms);
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
    return mavlink_msg_robomaster_motor_status_pack_status(system_id, component_id, _status, msg,  robomaster_motor_status->motor_id, robomaster_motor_status->control_mode, robomaster_motor_status->status, robomaster_motor_status->current_position_rad, robomaster_motor_status->current_speed_rad_s, robomaster_motor_status->current_duty_cycle, robomaster_motor_status->position_error_rad, robomaster_motor_status->timestamp_ms);
}

/**
 * @brief Send a robomaster_motor_status message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (see MOTOR_CONTROL_MODE enum)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param current_position_rad  Current position in radians
 * @param current_speed_rad_s  Current speed in rad/s
 * @param current_duty_cycle  Current duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (target - current)
 * @param timestamp_ms  Timestamp in milliseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robomaster_motor_status_send(mavlink_channel_t chan, uint8_t motor_id, uint8_t control_mode, uint8_t status, float current_position_rad, float current_speed_rad_s, float current_duty_cycle, float position_error_rad, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rad);
    _mav_put_float(buf, 4, current_speed_rad_s);
    _mav_put_float(buf, 8, current_duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, control_mode);
    _mav_put_uint8_t(buf, 22, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#else
    mavlink_robomaster_motor_status_t packet;
    packet.current_position_rad = current_position_rad;
    packet.current_speed_rad_s = current_speed_rad_s;
    packet.current_duty_cycle = current_duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
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
    mavlink_msg_robomaster_motor_status_send(chan, robomaster_motor_status->motor_id, robomaster_motor_status->control_mode, robomaster_motor_status->status, robomaster_motor_status->current_position_rad, robomaster_motor_status->current_speed_rad_s, robomaster_motor_status->current_duty_cycle, robomaster_motor_status->position_error_rad, robomaster_motor_status->timestamp_ms);
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
static inline void mavlink_msg_robomaster_motor_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint8_t control_mode, uint8_t status, float current_position_rad, float current_speed_rad_s, float current_duty_cycle, float position_error_rad, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, current_position_rad);
    _mav_put_float(buf, 4, current_speed_rad_s);
    _mav_put_float(buf, 8, current_duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, control_mode);
    _mav_put_uint8_t(buf, 22, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_CRC);
#else
    mavlink_robomaster_motor_status_t *packet = (mavlink_robomaster_motor_status_t *)msgbuf;
    packet->current_position_rad = current_position_rad;
    packet->current_speed_rad_s = current_speed_rad_s;
    packet->current_duty_cycle = current_duty_cycle;
    packet->position_error_rad = position_error_rad;
    packet->timestamp_ms = timestamp_ms;
    packet->motor_id = motor_id;
    packet->control_mode = control_mode;
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
 * @return  Motor ID (1-255)
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field control_mode from robomaster_motor_status message
 *
 * @return  Current control mode (see MOTOR_CONTROL_MODE enum)
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field status from robomaster_motor_status message
 *
 * @return  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 */
static inline uint8_t mavlink_msg_robomaster_motor_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field current_position_rad from robomaster_motor_status message
 *
 * @return  Current position in radians
 */
static inline float mavlink_msg_robomaster_motor_status_get_current_position_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field current_speed_rad_s from robomaster_motor_status message
 *
 * @return  Current speed in rad/s
 */
static inline float mavlink_msg_robomaster_motor_status_get_current_speed_rad_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field current_duty_cycle from robomaster_motor_status message
 *
 * @return  Current duty cycle (-1.0 to 1.0)
 */
static inline float mavlink_msg_robomaster_motor_status_get_current_duty_cycle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field position_error_rad from robomaster_motor_status message
 *
 * @return  Position error in radians (target - current)
 */
static inline float mavlink_msg_robomaster_motor_status_get_position_error_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field timestamp_ms from robomaster_motor_status message
 *
 * @return  Timestamp in milliseconds
 */
static inline uint32_t mavlink_msg_robomaster_motor_status_get_timestamp_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
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
    robomaster_motor_status->current_position_rad = mavlink_msg_robomaster_motor_status_get_current_position_rad(msg);
    robomaster_motor_status->current_speed_rad_s = mavlink_msg_robomaster_motor_status_get_current_speed_rad_s(msg);
    robomaster_motor_status->current_duty_cycle = mavlink_msg_robomaster_motor_status_get_current_duty_cycle(msg);
    robomaster_motor_status->position_error_rad = mavlink_msg_robomaster_motor_status_get_position_error_rad(msg);
    robomaster_motor_status->timestamp_ms = mavlink_msg_robomaster_motor_status_get_timestamp_ms(msg);
    robomaster_motor_status->motor_id = mavlink_msg_robomaster_motor_status_get_motor_id(msg);
    robomaster_motor_status->control_mode = mavlink_msg_robomaster_motor_status_get_control_mode(msg);
    robomaster_motor_status->status = mavlink_msg_robomaster_motor_status_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN;
        memset(robomaster_motor_status, 0, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_LEN);
    memcpy(robomaster_motor_status, _MAV_PAYLOAD(msg), len);
#endif
}
