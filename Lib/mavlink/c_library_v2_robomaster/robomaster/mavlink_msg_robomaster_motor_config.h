#pragma once
// MESSAGE ROBOMASTER_MOTOR_CONFIG PACKING

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG 182


typedef struct __mavlink_robomaster_motor_config_t {
 float param_value; /*<  Parameter value*/
 uint8_t motor_id; /*<  Motor ID (1-8)*/
 uint8_t param_id; /*<  Parameter ID (0-15)*/
 uint8_t save_to_flash; /*<  Save to flash memory (0=no, 1=yes)*/
} mavlink_robomaster_motor_config_t;

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN 7
#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN 7
#define MAVLINK_MSG_ID_182_LEN 7
#define MAVLINK_MSG_ID_182_MIN_LEN 7

#define MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC 35
#define MAVLINK_MSG_ID_182_CRC 35



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_CONFIG { \
    182, \
    "ROBOMASTER_MOTOR_CONFIG", \
    4, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_robomaster_motor_config_t, motor_id) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_robomaster_motor_config_t, param_id) }, \
         { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_config_t, param_value) }, \
         { "save_to_flash", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_robomaster_motor_config_t, save_to_flash) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_MOTOR_CONFIG { \
    "ROBOMASTER_MOTOR_CONFIG", \
    4, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_robomaster_motor_config_t, motor_id) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_robomaster_motor_config_t, param_id) }, \
         { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_robomaster_motor_config_t, param_value) }, \
         { "save_to_flash", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_robomaster_motor_config_t, save_to_flash) }, \
         } \
}
#endif

/**
 * @brief Pack a robomaster_motor_config message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-8)
 * @param param_id  Parameter ID (0-15)
 * @param param_value  Parameter value
 * @param save_to_flash  Save to flash memory (0=no, 1=yes)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_config_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t param_id, float param_value, uint8_t save_to_flash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, param_id);
    _mav_put_uint8_t(buf, 6, save_to_flash);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
#else
    mavlink_robomaster_motor_config_t packet;
    packet.param_value = param_value;
    packet.motor_id = motor_id;
    packet.param_id = param_id;
    packet.save_to_flash = save_to_flash;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
}

/**
 * @brief Pack a robomaster_motor_config message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-8)
 * @param param_id  Parameter ID (0-15)
 * @param param_value  Parameter value
 * @param save_to_flash  Save to flash memory (0=no, 1=yes)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_config_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t param_id, float param_value, uint8_t save_to_flash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, param_id);
    _mav_put_uint8_t(buf, 6, save_to_flash);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
#else
    mavlink_robomaster_motor_config_t packet;
    packet.param_value = param_value;
    packet.motor_id = motor_id;
    packet.param_id = param_id;
    packet.save_to_flash = save_to_flash;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
#endif
}

/**
 * @brief Pack a robomaster_motor_config message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  Motor ID (1-8)
 * @param param_id  Parameter ID (0-15)
 * @param param_value  Parameter value
 * @param save_to_flash  Save to flash memory (0=no, 1=yes)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_motor_config_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint8_t param_id,float param_value,uint8_t save_to_flash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, param_id);
    _mav_put_uint8_t(buf, 6, save_to_flash);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
#else
    mavlink_robomaster_motor_config_t packet;
    packet.param_value = param_value;
    packet.motor_id = motor_id;
    packet.param_id = param_id;
    packet.save_to_flash = save_to_flash;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
}

/**
 * @brief Encode a robomaster_motor_config struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_config_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robomaster_motor_config_t* robomaster_motor_config)
{
    return mavlink_msg_robomaster_motor_config_pack(system_id, component_id, msg, robomaster_motor_config->motor_id, robomaster_motor_config->param_id, robomaster_motor_config->param_value, robomaster_motor_config->save_to_flash);
}

/**
 * @brief Encode a robomaster_motor_config struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_config_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robomaster_motor_config_t* robomaster_motor_config)
{
    return mavlink_msg_robomaster_motor_config_pack_chan(system_id, component_id, chan, msg, robomaster_motor_config->motor_id, robomaster_motor_config->param_id, robomaster_motor_config->param_value, robomaster_motor_config->save_to_flash);
}

/**
 * @brief Encode a robomaster_motor_config struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_motor_config C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_motor_config_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_robomaster_motor_config_t* robomaster_motor_config)
{
    return mavlink_msg_robomaster_motor_config_pack_status(system_id, component_id, _status, msg,  robomaster_motor_config->motor_id, robomaster_motor_config->param_id, robomaster_motor_config->param_value, robomaster_motor_config->save_to_flash);
}

/**
 * @brief Send a robomaster_motor_config message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (1-8)
 * @param param_id  Parameter ID (0-15)
 * @param param_value  Parameter value
 * @param save_to_flash  Save to flash memory (0=no, 1=yes)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robomaster_motor_config_send(mavlink_channel_t chan, uint8_t motor_id, uint8_t param_id, float param_value, uint8_t save_to_flash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN];
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, param_id);
    _mav_put_uint8_t(buf, 6, save_to_flash);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
#else
    mavlink_robomaster_motor_config_t packet;
    packet.param_value = param_value;
    packet.motor_id = motor_id;
    packet.param_id = param_id;
    packet.save_to_flash = save_to_flash;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG, (const char *)&packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
#endif
}

/**
 * @brief Send a robomaster_motor_config message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robomaster_motor_config_send_struct(mavlink_channel_t chan, const mavlink_robomaster_motor_config_t* robomaster_motor_config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robomaster_motor_config_send(chan, robomaster_motor_config->motor_id, robomaster_motor_config->param_id, robomaster_motor_config->param_value, robomaster_motor_config->save_to_flash);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG, (const char *)robomaster_motor_config, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robomaster_motor_config_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint8_t param_id, float param_value, uint8_t save_to_flash)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, param_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, param_id);
    _mav_put_uint8_t(buf, 6, save_to_flash);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG, buf, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
#else
    mavlink_robomaster_motor_config_t *packet = (mavlink_robomaster_motor_config_t *)msgbuf;
    packet->param_value = param_value;
    packet->motor_id = motor_id;
    packet->param_id = param_id;
    packet->save_to_flash = save_to_flash;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG, (const char *)packet, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOMASTER_MOTOR_CONFIG UNPACKING


/**
 * @brief Get field motor_id from robomaster_motor_config message
 *
 * @return  Motor ID (1-8)
 */
static inline uint8_t mavlink_msg_robomaster_motor_config_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field param_id from robomaster_motor_config message
 *
 * @return  Parameter ID (0-15)
 */
static inline uint8_t mavlink_msg_robomaster_motor_config_get_param_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field param_value from robomaster_motor_config message
 *
 * @return  Parameter value
 */
static inline float mavlink_msg_robomaster_motor_config_get_param_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field save_to_flash from robomaster_motor_config message
 *
 * @return  Save to flash memory (0=no, 1=yes)
 */
static inline uint8_t mavlink_msg_robomaster_motor_config_get_save_to_flash(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a robomaster_motor_config message into a struct
 *
 * @param msg The message to decode
 * @param robomaster_motor_config C-struct to decode the message contents into
 */
static inline void mavlink_msg_robomaster_motor_config_decode(const mavlink_message_t* msg, mavlink_robomaster_motor_config_t* robomaster_motor_config)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robomaster_motor_config->param_value = mavlink_msg_robomaster_motor_config_get_param_value(msg);
    robomaster_motor_config->motor_id = mavlink_msg_robomaster_motor_config_get_motor_id(msg);
    robomaster_motor_config->param_id = mavlink_msg_robomaster_motor_config_get_param_id(msg);
    robomaster_motor_config->save_to_flash = mavlink_msg_robomaster_motor_config_get_save_to_flash(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN? msg->len : MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN;
        memset(robomaster_motor_config, 0, MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_LEN);
    memcpy(robomaster_motor_config, _MAV_PAYLOAD(msg), len);
#endif
}
