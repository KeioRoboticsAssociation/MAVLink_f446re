#pragma once
// MESSAGE ROBOMASTER_TELEMETRY PACKING

#define MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY 183


typedef struct __mavlink_robomaster_telemetry_t {
 uint32_t system_uptime; /*<  System uptime in milliseconds*/
 float supply_voltage; /*<  Supply voltage in volts*/
 uint16_t can_error_count; /*<  CAN error count*/
 uint8_t motors_active; /*<  Number of active motors*/
 uint8_t can_status; /*<  CAN bus status (0=error, 1=ok)*/
 uint8_t emergency_stop; /*<  Emergency stop status (0=normal, 1=emergency)*/
 uint8_t motor_mask; /*<  Bitmask of registered motors (bit 0 = motor 1, etc.)*/
} mavlink_robomaster_telemetry_t;

#define MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN 14
#define MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN 14
#define MAVLINK_MSG_ID_183_LEN 14
#define MAVLINK_MSG_ID_183_MIN_LEN 14

#define MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC 194
#define MAVLINK_MSG_ID_183_CRC 194



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_TELEMETRY { \
    183, \
    "ROBOMASTER_TELEMETRY", \
    7, \
    {  { "motors_active", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_robomaster_telemetry_t, motors_active) }, \
         { "can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_robomaster_telemetry_t, can_status) }, \
         { "can_error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_robomaster_telemetry_t, can_error_count) }, \
         { "system_uptime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_robomaster_telemetry_t, system_uptime) }, \
         { "emergency_stop", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_robomaster_telemetry_t, emergency_stop) }, \
         { "supply_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_telemetry_t, supply_voltage) }, \
         { "motor_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_robomaster_telemetry_t, motor_mask) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROBOMASTER_TELEMETRY { \
    "ROBOMASTER_TELEMETRY", \
    7, \
    {  { "motors_active", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_robomaster_telemetry_t, motors_active) }, \
         { "can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_robomaster_telemetry_t, can_status) }, \
         { "can_error_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_robomaster_telemetry_t, can_error_count) }, \
         { "system_uptime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_robomaster_telemetry_t, system_uptime) }, \
         { "emergency_stop", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_robomaster_telemetry_t, emergency_stop) }, \
         { "supply_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_robomaster_telemetry_t, supply_voltage) }, \
         { "motor_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_robomaster_telemetry_t, motor_mask) }, \
         } \
}
#endif

/**
 * @brief Pack a robomaster_telemetry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motors_active  Number of active motors
 * @param can_status  CAN bus status (0=error, 1=ok)
 * @param can_error_count  CAN error count
 * @param system_uptime  System uptime in milliseconds
 * @param emergency_stop  Emergency stop status (0=normal, 1=emergency)
 * @param supply_voltage  Supply voltage in volts
 * @param motor_mask  Bitmask of registered motors (bit 0 = motor 1, etc.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_telemetry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motors_active, uint8_t can_status, uint16_t can_error_count, uint32_t system_uptime, uint8_t emergency_stop, float supply_voltage, uint8_t motor_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN];
    _mav_put_uint32_t(buf, 0, system_uptime);
    _mav_put_float(buf, 4, supply_voltage);
    _mav_put_uint16_t(buf, 8, can_error_count);
    _mav_put_uint8_t(buf, 10, motors_active);
    _mav_put_uint8_t(buf, 11, can_status);
    _mav_put_uint8_t(buf, 12, emergency_stop);
    _mav_put_uint8_t(buf, 13, motor_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
#else
    mavlink_robomaster_telemetry_t packet;
    packet.system_uptime = system_uptime;
    packet.supply_voltage = supply_voltage;
    packet.can_error_count = can_error_count;
    packet.motors_active = motors_active;
    packet.can_status = can_status;
    packet.emergency_stop = emergency_stop;
    packet.motor_mask = motor_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
}

/**
 * @brief Pack a robomaster_telemetry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motors_active  Number of active motors
 * @param can_status  CAN bus status (0=error, 1=ok)
 * @param can_error_count  CAN error count
 * @param system_uptime  System uptime in milliseconds
 * @param emergency_stop  Emergency stop status (0=normal, 1=emergency)
 * @param supply_voltage  Supply voltage in volts
 * @param motor_mask  Bitmask of registered motors (bit 0 = motor 1, etc.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_telemetry_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motors_active, uint8_t can_status, uint16_t can_error_count, uint32_t system_uptime, uint8_t emergency_stop, float supply_voltage, uint8_t motor_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN];
    _mav_put_uint32_t(buf, 0, system_uptime);
    _mav_put_float(buf, 4, supply_voltage);
    _mav_put_uint16_t(buf, 8, can_error_count);
    _mav_put_uint8_t(buf, 10, motors_active);
    _mav_put_uint8_t(buf, 11, can_status);
    _mav_put_uint8_t(buf, 12, emergency_stop);
    _mav_put_uint8_t(buf, 13, motor_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
#else
    mavlink_robomaster_telemetry_t packet;
    packet.system_uptime = system_uptime;
    packet.supply_voltage = supply_voltage;
    packet.can_error_count = can_error_count;
    packet.motors_active = motors_active;
    packet.can_status = can_status;
    packet.emergency_stop = emergency_stop;
    packet.motor_mask = motor_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
#endif
}

/**
 * @brief Pack a robomaster_telemetry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motors_active  Number of active motors
 * @param can_status  CAN bus status (0=error, 1=ok)
 * @param can_error_count  CAN error count
 * @param system_uptime  System uptime in milliseconds
 * @param emergency_stop  Emergency stop status (0=normal, 1=emergency)
 * @param supply_voltage  Supply voltage in volts
 * @param motor_mask  Bitmask of registered motors (bit 0 = motor 1, etc.)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_robomaster_telemetry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motors_active,uint8_t can_status,uint16_t can_error_count,uint32_t system_uptime,uint8_t emergency_stop,float supply_voltage,uint8_t motor_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN];
    _mav_put_uint32_t(buf, 0, system_uptime);
    _mav_put_float(buf, 4, supply_voltage);
    _mav_put_uint16_t(buf, 8, can_error_count);
    _mav_put_uint8_t(buf, 10, motors_active);
    _mav_put_uint8_t(buf, 11, can_status);
    _mav_put_uint8_t(buf, 12, emergency_stop);
    _mav_put_uint8_t(buf, 13, motor_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
#else
    mavlink_robomaster_telemetry_t packet;
    packet.system_uptime = system_uptime;
    packet.supply_voltage = supply_voltage;
    packet.can_error_count = can_error_count;
    packet.motors_active = motors_active;
    packet.can_status = can_status;
    packet.emergency_stop = emergency_stop;
    packet.motor_mask = motor_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
}

/**
 * @brief Encode a robomaster_telemetry struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_telemetry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_robomaster_telemetry_t* robomaster_telemetry)
{
    return mavlink_msg_robomaster_telemetry_pack(system_id, component_id, msg, robomaster_telemetry->motors_active, robomaster_telemetry->can_status, robomaster_telemetry->can_error_count, robomaster_telemetry->system_uptime, robomaster_telemetry->emergency_stop, robomaster_telemetry->supply_voltage, robomaster_telemetry->motor_mask);
}

/**
 * @brief Encode a robomaster_telemetry struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_telemetry_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_robomaster_telemetry_t* robomaster_telemetry)
{
    return mavlink_msg_robomaster_telemetry_pack_chan(system_id, component_id, chan, msg, robomaster_telemetry->motors_active, robomaster_telemetry->can_status, robomaster_telemetry->can_error_count, robomaster_telemetry->system_uptime, robomaster_telemetry->emergency_stop, robomaster_telemetry->supply_voltage, robomaster_telemetry->motor_mask);
}

/**
 * @brief Encode a robomaster_telemetry struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param robomaster_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_robomaster_telemetry_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_robomaster_telemetry_t* robomaster_telemetry)
{
    return mavlink_msg_robomaster_telemetry_pack_status(system_id, component_id, _status, msg,  robomaster_telemetry->motors_active, robomaster_telemetry->can_status, robomaster_telemetry->can_error_count, robomaster_telemetry->system_uptime, robomaster_telemetry->emergency_stop, robomaster_telemetry->supply_voltage, robomaster_telemetry->motor_mask);
}

/**
 * @brief Send a robomaster_telemetry message
 * @param chan MAVLink channel to send the message
 *
 * @param motors_active  Number of active motors
 * @param can_status  CAN bus status (0=error, 1=ok)
 * @param can_error_count  CAN error count
 * @param system_uptime  System uptime in milliseconds
 * @param emergency_stop  Emergency stop status (0=normal, 1=emergency)
 * @param supply_voltage  Supply voltage in volts
 * @param motor_mask  Bitmask of registered motors (bit 0 = motor 1, etc.)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_robomaster_telemetry_send(mavlink_channel_t chan, uint8_t motors_active, uint8_t can_status, uint16_t can_error_count, uint32_t system_uptime, uint8_t emergency_stop, float supply_voltage, uint8_t motor_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN];
    _mav_put_uint32_t(buf, 0, system_uptime);
    _mav_put_float(buf, 4, supply_voltage);
    _mav_put_uint16_t(buf, 8, can_error_count);
    _mav_put_uint8_t(buf, 10, motors_active);
    _mav_put_uint8_t(buf, 11, can_status);
    _mav_put_uint8_t(buf, 12, emergency_stop);
    _mav_put_uint8_t(buf, 13, motor_mask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY, buf, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
#else
    mavlink_robomaster_telemetry_t packet;
    packet.system_uptime = system_uptime;
    packet.supply_voltage = supply_voltage;
    packet.can_error_count = can_error_count;
    packet.motors_active = motors_active;
    packet.can_status = can_status;
    packet.emergency_stop = emergency_stop;
    packet.motor_mask = motor_mask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY, (const char *)&packet, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
#endif
}

/**
 * @brief Send a robomaster_telemetry message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_robomaster_telemetry_send_struct(mavlink_channel_t chan, const mavlink_robomaster_telemetry_t* robomaster_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_robomaster_telemetry_send(chan, robomaster_telemetry->motors_active, robomaster_telemetry->can_status, robomaster_telemetry->can_error_count, robomaster_telemetry->system_uptime, robomaster_telemetry->emergency_stop, robomaster_telemetry->supply_voltage, robomaster_telemetry->motor_mask);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY, (const char *)robomaster_telemetry, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_robomaster_telemetry_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motors_active, uint8_t can_status, uint16_t can_error_count, uint32_t system_uptime, uint8_t emergency_stop, float supply_voltage, uint8_t motor_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, system_uptime);
    _mav_put_float(buf, 4, supply_voltage);
    _mav_put_uint16_t(buf, 8, can_error_count);
    _mav_put_uint8_t(buf, 10, motors_active);
    _mav_put_uint8_t(buf, 11, can_status);
    _mav_put_uint8_t(buf, 12, emergency_stop);
    _mav_put_uint8_t(buf, 13, motor_mask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY, buf, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
#else
    mavlink_robomaster_telemetry_t *packet = (mavlink_robomaster_telemetry_t *)msgbuf;
    packet->system_uptime = system_uptime;
    packet->supply_voltage = supply_voltage;
    packet->can_error_count = can_error_count;
    packet->motors_active = motors_active;
    packet->can_status = can_status;
    packet->emergency_stop = emergency_stop;
    packet->motor_mask = motor_mask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY, (const char *)packet, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_CRC);
#endif
}
#endif

#endif

// MESSAGE ROBOMASTER_TELEMETRY UNPACKING


/**
 * @brief Get field motors_active from robomaster_telemetry message
 *
 * @return  Number of active motors
 */
static inline uint8_t mavlink_msg_robomaster_telemetry_get_motors_active(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field can_status from robomaster_telemetry message
 *
 * @return  CAN bus status (0=error, 1=ok)
 */
static inline uint8_t mavlink_msg_robomaster_telemetry_get_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field can_error_count from robomaster_telemetry message
 *
 * @return  CAN error count
 */
static inline uint16_t mavlink_msg_robomaster_telemetry_get_can_error_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field system_uptime from robomaster_telemetry message
 *
 * @return  System uptime in milliseconds
 */
static inline uint32_t mavlink_msg_robomaster_telemetry_get_system_uptime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field emergency_stop from robomaster_telemetry message
 *
 * @return  Emergency stop status (0=normal, 1=emergency)
 */
static inline uint8_t mavlink_msg_robomaster_telemetry_get_emergency_stop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field supply_voltage from robomaster_telemetry message
 *
 * @return  Supply voltage in volts
 */
static inline float mavlink_msg_robomaster_telemetry_get_supply_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field motor_mask from robomaster_telemetry message
 *
 * @return  Bitmask of registered motors (bit 0 = motor 1, etc.)
 */
static inline uint8_t mavlink_msg_robomaster_telemetry_get_motor_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a robomaster_telemetry message into a struct
 *
 * @param msg The message to decode
 * @param robomaster_telemetry C-struct to decode the message contents into
 */
static inline void mavlink_msg_robomaster_telemetry_decode(const mavlink_message_t* msg, mavlink_robomaster_telemetry_t* robomaster_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    robomaster_telemetry->system_uptime = mavlink_msg_robomaster_telemetry_get_system_uptime(msg);
    robomaster_telemetry->supply_voltage = mavlink_msg_robomaster_telemetry_get_supply_voltage(msg);
    robomaster_telemetry->can_error_count = mavlink_msg_robomaster_telemetry_get_can_error_count(msg);
    robomaster_telemetry->motors_active = mavlink_msg_robomaster_telemetry_get_motors_active(msg);
    robomaster_telemetry->can_status = mavlink_msg_robomaster_telemetry_get_can_status(msg);
    robomaster_telemetry->emergency_stop = mavlink_msg_robomaster_telemetry_get_emergency_stop(msg);
    robomaster_telemetry->motor_mask = mavlink_msg_robomaster_telemetry_get_motor_mask(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN? msg->len : MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN;
        memset(robomaster_telemetry, 0, MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_LEN);
    memcpy(robomaster_telemetry, _MAV_PAYLOAD(msg), len);
#endif
}
