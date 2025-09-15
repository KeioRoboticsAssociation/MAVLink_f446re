#pragma once
// MESSAGE ROBOMASTER_MOTOR_CONTROL PACKING

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL 180


typedef struct __mavlink_robomaster_motor_control_t {
 float control_value; /*<  Control value (units depend on mode: mA, RPS, or radians)*/
 uint8_t motor_id; /*<  Motor ID (1-8)*/
 uint8_t control_mode; /*<  Control mode: 0=current, 1=velocity, 2=position*/
 uint8_t enable; /*<  Enable motor (0=disable, 1=enable)*/
} mavlink_robomaster_motor_control_t;

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN 7
#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN 7
#define MAVLINK_MSG_ID_180_LEN 7
#define MAVLINK_MSG_ID_180_MIN_LEN 7

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC 81
#define MAVLINK_MSG_ID_180_CRC 81



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_CONTROL { \
    180, \
    "ROBOMASTER_MOTOR_CONTROL", \
    4, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_robomaster_motor_control_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_robomaster_motor_control_t, control_mode) }, \
         { "control_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_control_t, control_value) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_robomaster_motor_control_t, enable) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_CONTROL { \
    "ROBOMASTER_MOTOR_CONTROL", \
    4, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_robomaster_motor_control_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_robomaster_motor_control_t, control_mode) }, \
         { "control_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_control_t, control_value) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_robomaster_motor_control_t, enable) }, \
         } \
}
#endif

/**
 * @brief Pack a robomaster_motor_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-8)
 * @param control_mode  Control mode: 0=current, 1=velocity, 2=position
 * @param control_value  Control value (units depend on mode: mA, RPS, or radians)
 * @param enable  Enable motor (0=disable, 1=enable)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, float control_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, control_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.control_value = control_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

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
 * @param motor_id  Motor ID (1-8)
 * @param control_mode  Control mode: 0=current, 1=velocity, 2=position
 * @param control_value  Control value (units depend on mode: mA, RPS, or radians)
 * @param enable  Enable motor (0=disable, 1=enable)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, float control_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, control_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.control_value = control_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

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
 * @param motor_id  Motor ID (1-8)
 * @param control_mode  Control mode: 0=current, 1=velocity, 2=position
 * @param control_value  Control value (units depend on mode: mA, RPS, or radians)
 * @param enable  Enable motor (0=disable, 1=enable)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint8_t control_mode,float control_value,uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, control_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.control_value = control_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

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
    return mavlink_msg_robomaster_motor_control_pack(system_id, component_id, msg, robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->control_value, robomaster_motor_control->enable);
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
    return mavlink_msg_robomaster_motor_control_pack_chan(system_id, component_id, chan, msg, robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->control_value, robomaster_motor_control->enable);
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
    return mavlink_msg_robomaster_motor_control_pack_status(system_id, component_id, _status, msg,  robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->control_value, robomaster_motor_control->enable);
}

/**
 * @brief Send a robomaster_motor_control message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (1-8)
 * @param control_mode  Control mode: 0=current, 1=velocity, 2=position
 * @param control_value  Control value (units depend on mode: mA, RPS, or radians)
 * @param enable  Enable motor (0=disable, 1=enable)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robomaster_motor_control_send(mavlink_channel_t chan, uint8_t motor_id, uint8_t control_mode, float control_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, control_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#else
    mavlink_robomaster_motor_control_t packet;
    packet.control_value = control_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

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
    mavlink_msg_robomaster_motor_control_send(chan, robomaster_motor_control->motor_id, robomaster_motor_control->control_mode, robomaster_motor_control->control_value, robomaster_motor_control->enable);
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
static inline void mavlink_msg_robomaster_motor_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint8_t control_mode, float control_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, control_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#else
    mavlink_robomaster_motor_control_t *packet = (mavlink_robomaster_motor_control_t *)msgbuf;
    packet->control_value = control_value;
    packet->motor_id = motor_id;
    packet->control_mode = control_mode;
    packet->enable = enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL, (const char *)packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOMASTER_MOTOR_CONTROL UNPACKING


/**
 * @brief Get field motor_id from robomaster_motor_control message
 *
 * @return  Motor ID (1-8)
 */
static inline uint8_t mavlink_msg_robomaster_motor_control_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field control_mode from robomaster_motor_control message
 *
 * @return  Control mode: 0=current, 1=velocity, 2=position
 */
static inline uint8_t mavlink_msg_robomaster_motor_control_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field control_value from robomaster_motor_control message
 *
 * @return  Control value (units depend on mode: mA, RPS, or radians)
 */
static inline float mavlink_msg_robomaster_motor_control_get_control_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field enable from robomaster_motor_control message
 *
 * @return  Enable motor (0=disable, 1=enable)
 */
static inline uint8_t mavlink_msg_robomaster_motor_control_get_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
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
    robomaster_motor_control->control_value = mavlink_msg_robomaster_motor_control_get_control_value(msg);
    robomaster_motor_control->motor_id = mavlink_msg_robomaster_motor_control_get_motor_id(msg);
    robomaster_motor_control->control_mode = mavlink_msg_robomaster_motor_control_get_control_mode(msg);
    robomaster_motor_control->enable = mavlink_msg_robomaster_motor_control_get_enable(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN;
        memset(robomaster_motor_control, 0, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_LEN);
    memcpy(robomaster_motor_control, _MAV_PAYLOAD(msg), len);
#endif
}
