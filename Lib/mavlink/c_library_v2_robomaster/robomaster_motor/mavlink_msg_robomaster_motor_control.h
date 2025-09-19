#pragma once
// MESSAGE ROBOMASTER_MOTOR_CONTROL PACKING

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL 12000


typedef struct __mavlink_robomaster_motor_control_t {
 float duty_cycle; /*<  Duty cycle (-1.0 to 1.0), used in DUTY_TO_POSITION mode*/
 float target_position_rad; /*<  Target position in radians*/
 float target_speed_rad_s; /*<  Target speed in rad/s*/
 uint32_t timeout_ms; /*<  Timeout in milliseconds for DUTY_TO_POSITION mode*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t motor_id; /*<  Motor ID (1-255)*/
 uint8_t control_mode; /*<  Control mode (see MOTOR_CONTROL_MODE enum)*/
} mavlink_robomaster_motor_control_t;

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN 20
#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN 20
#define MAVLINK_MSG_ID_12000_LEN 20
#define MAVLINK_MSG_ID_12000_MIN_LEN 20

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC 148
#define MAVLINK_MSG_ID_12000_CRC 148



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_CONTROL { \
    12000, \
    "ROBOMASTER_MOTOR_CONTROL", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_robomaster_motor_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_robomaster_motor_control_t, target_component) }, \
         { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_robomaster_motor_control_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_robomaster_motor_control_t, control_mode) }, \
         { "duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_control_t, duty_cycle) }, \
         { "target_position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_motor_control_t, target_position_rad) }, \
         { "target_speed_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robomaster_motor_control_t, target_speed_rad_s) }, \
         { "timeout_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_robomaster_motor_control_t, timeout_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_CONTROL { \
    "ROBOMASTER_MOTOR_CONTROL", \
    8, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_robomaster_motor_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_robomaster_motor_control_t, target_component) }, \
         { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_robomaster_motor_control_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_robomaster_motor_control_t, control_mode) }, \
         { "duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_control_t, duty_cycle) }, \
         { "target_position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_motor_control_t, target_position_rad) }, \
         { "target_speed_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_robomaster_motor_control_t, target_speed_rad_s) }, \
         { "timeout_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_robomaster_motor_control_t, timeout_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a robomaster_motor_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (see MOTOR_CONTROL_MODE enum)
 * @param duty_cycle  Duty cycle (-1.0 to 1.0), used in DUTY_TO_POSITION mode
 * @param target_position_rad  Target position in radians
 * @param target_speed_rad_s  Target speed in rad/s
 * @param timeout_ms  Timeout in milliseconds for DUTY_TO_POSITION mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float duty_cycle, float target_position_rad, float target_speed_rad_s, uint32_t timeout_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, duty_cycle);
    _mav_put_float(buf, 4, target_position_rad);
    _mav_put_float(buf, 8, target_speed_rad_s);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, motor_id);
    _mav_put_uint8_t(buf, 19, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.duty_cycle = duty_cycle;
    packet.target_position_rad = target_position_rad;
    packet.target_speed_rad_s = target_speed_rad_s;
    packet.timeout_ms = timeout_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
}

/**
 * @brief Pack a robomaster_motor_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (see MOTOR_CONTROL_MODE enum)
 * @param duty_cycle  Duty cycle (-1.0 to 1.0), used in DUTY_TO_POSITION mode
 * @param target_position_rad  Target position in radians
 * @param target_speed_rad_s  Target speed in rad/s
 * @param timeout_ms  Timeout in milliseconds for DUTY_TO_POSITION mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float duty_cycle, float target_position_rad, float target_speed_rad_s, uint32_t timeout_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, duty_cycle);
    _mav_put_float(buf, 4, target_position_rad);
    _mav_put_float(buf, 8, target_speed_rad_s);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, motor_id);
    _mav_put_uint8_t(buf, 19, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.duty_cycle = duty_cycle;
    packet.target_position_rad = target_position_rad;
    packet.target_speed_rad_s = target_speed_rad_s;
    packet.timeout_ms = timeout_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a robomaster_motor_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (see MOTOR_CONTROL_MODE enum)
 * @param duty_cycle  Duty cycle (-1.0 to 1.0), used in DUTY_TO_POSITION mode
 * @param target_position_rad  Target position in radians
 * @param target_speed_rad_s  Target speed in rad/s
 * @param timeout_ms  Timeout in milliseconds for DUTY_TO_POSITION mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t motor_id,uint8_t control_mode,float duty_cycle,float target_position_rad,float target_speed_rad_s,uint32_t timeout_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, duty_cycle);
    _mav_put_float(buf, 4, target_position_rad);
    _mav_put_float(buf, 8, target_speed_rad_s);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, motor_id);
    _mav_put_uint8_t(buf, 19, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.duty_cycle = duty_cycle;
    packet.target_position_rad = target_position_rad;
    packet.target_speed_rad_s = target_speed_rad_s;
    packet.timeout_ms = timeout_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
}

/**
 * @brief Encode a robomaster_motor_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robomaster_motor_control_t* robomaster_motor_control)
{
    return mavlink_msg_robomaster_motor_control_pack(system_id, component_id, msg, robomaster_motor_control->target_system, robomaster_motor_control->target_component, robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->duty_cycle, robomaster_motor_control->target_position_rad, robomaster_motor_control->target_speed_rad_s, robomaster_motor_control->timeout_ms);
}

/**
 * @brief Encode a robomaster_motor_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robomaster_motor_control_t* robomaster_motor_control)
{
    return mavlink_msg_robomaster_motor_control_pack_chan(system_id, component_id, chan, msg, robomaster_motor_control->target_system, robomaster_motor_control->target_component, robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->duty_cycle, robomaster_motor_control->target_position_rad, robomaster_motor_control->target_speed_rad_s, robomaster_motor_control->timeout_ms);
}

/**
 * @brief Encode a robomaster_motor_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_robomaster_motor_control_t* robomaster_motor_control)
{
    return mavlink_msg_robomaster_motor_control_pack_status(system_id, component_id, _status, msg,  robomaster_motor_control->target_system, robomaster_motor_control->target_component, robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->duty_cycle, robomaster_motor_control->target_position_rad, robomaster_motor_control->target_speed_rad_s, robomaster_motor_control->timeout_ms);
}

/**
 * @brief Send a robomaster_motor_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (see MOTOR_CONTROL_MODE enum)
 * @param duty_cycle  Duty cycle (-1.0 to 1.0), used in DUTY_TO_POSITION mode
 * @param target_position_rad  Target position in radians
 * @param target_speed_rad_s  Target speed in rad/s
 * @param timeout_ms  Timeout in milliseconds for DUTY_TO_POSITION mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robomaster_motor_control_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float duty_cycle, float target_position_rad, float target_speed_rad_s, uint32_t timeout_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, duty_cycle);
    _mav_put_float(buf, 4, target_position_rad);
    _mav_put_float(buf, 8, target_speed_rad_s);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, motor_id);
    _mav_put_uint8_t(buf, 19, control_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.duty_cycle = duty_cycle;
    packet.target_position_rad = target_position_rad;
    packet.target_speed_rad_s = target_speed_rad_s;
    packet.timeout_ms = timeout_ms;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#endif
}

/**
 * @brief Send a robomaster_motor_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robomaster_motor_control_send_struct(mavlink_channel_t chan, const mavlink_robomaster_motor_control_t* robomaster_motor_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robomaster_motor_control_send(chan, robomaster_motor_control->target_system, robomaster_motor_control->target_component, robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->duty_cycle, robomaster_motor_control->target_position_rad, robomaster_motor_control->target_speed_rad_s, robomaster_motor_control->timeout_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, (const char *)robomaster_motor_control, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robomaster_motor_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float duty_cycle, float target_position_rad, float target_speed_rad_s, uint32_t timeout_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, duty_cycle);
    _mav_put_float(buf, 4, target_position_rad);
    _mav_put_float(buf, 8, target_speed_rad_s);
    _mav_put_uint32_t(buf, 12, timeout_ms);
    _mav_put_uint8_t(buf, 16, target_system);
    _mav_put_uint8_t(buf, 17, target_component);
    _mav_put_uint8_t(buf, 18, motor_id);
    _mav_put_uint8_t(buf, 19, control_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#else
    mavlink_robomaster_motor_control_t *packet = (mavlink_robomaster_motor_control_t *)msgbuf;
    packet->duty_cycle = duty_cycle;
    packet->target_position_rad = target_position_rad;
    packet->target_speed_rad_s = target_speed_rad_s;
    packet->timeout_ms = timeout_ms;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->motor_id = motor_id;
    packet->control_mode = control_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, (const char *)packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOMASTER_MOTOR_CONTROL UNPACKING


/**
 * @brief Get field target_system from robomaster_motor_control message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_robomaster_motor_control_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field target_component from robomaster_motor_control message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_robomaster_motor_control_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field motor_id from robomaster_motor_control message
 *
 * @return  Motor ID (1-255)
 */
static inline uint8_t mavlink_msg_robomaster_motor_control_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field control_mode from robomaster_motor_control message
 *
 * @return  Control mode (see MOTOR_CONTROL_MODE enum)
 */
static inline uint8_t mavlink_msg_robomaster_motor_control_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field duty_cycle from robomaster_motor_control message
 *
 * @return  Duty cycle (-1.0 to 1.0), used in DUTY_TO_POSITION mode
 */
static inline float mavlink_msg_robomaster_motor_control_get_duty_cycle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field target_position_rad from robomaster_motor_control message
 *
 * @return  Target position in radians
 */
static inline float mavlink_msg_robomaster_motor_control_get_target_position_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field target_speed_rad_s from robomaster_motor_control message
 *
 * @return  Target speed in rad/s
 */
static inline float mavlink_msg_robomaster_motor_control_get_target_speed_rad_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field timeout_ms from robomaster_motor_control message
 *
 * @return  Timeout in milliseconds for DUTY_TO_POSITION mode
 */
static inline uint32_t mavlink_msg_robomaster_motor_control_get_timeout_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Decode a robomaster_motor_control message into a struct
 *
 * @param msg The message to decode
 * @param robomaster_motor_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_robomaster_motor_control_decode(const mavlink_message_t* msg, mavlink_robomaster_motor_control_t* robomaster_motor_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robomaster_motor_control->duty_cycle = mavlink_msg_robomaster_motor_control_get_duty_cycle(msg);
    robomaster_motor_control->target_position_rad = mavlink_msg_robomaster_motor_control_get_target_position_rad(msg);
    robomaster_motor_control->target_speed_rad_s = mavlink_msg_robomaster_motor_control_get_target_speed_rad_s(msg);
    robomaster_motor_control->timeout_ms = mavlink_msg_robomaster_motor_control_get_timeout_ms(msg);
    robomaster_motor_control->target_system = mavlink_msg_robomaster_motor_control_get_target_system(msg);
    robomaster_motor_control->target_component = mavlink_msg_robomaster_motor_control_get_target_component(msg);
    robomaster_motor_control->motor_id = mavlink_msg_robomaster_motor_control_get_motor_id(msg);
    robomaster_motor_control->control_mode = mavlink_msg_robomaster_motor_control_get_control_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN;
        memset(robomaster_motor_control, 0, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
    memcpy(robomaster_motor_control, _MAV_PAYLOAD(msg), len);
#endif
}
