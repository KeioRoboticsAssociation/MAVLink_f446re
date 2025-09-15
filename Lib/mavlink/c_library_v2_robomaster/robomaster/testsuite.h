/** @file
 *    @brief MAVLink comm protocol testsuite generated from robomaster.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ROBOMASTER_TESTSUITE_H
#define ROBOMASTER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_robomaster(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_robomaster(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_robomaster_motor_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robomaster_motor_control_t packet_in = {
        17.0,17,84,151
    };
    mavlink_robomaster_motor_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.control_value = packet_in.control_value;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
        packet1.enable = packet_in.enable;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robomaster_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_pack(system_id, component_id, &msg , packet1.motor_id , packet1.control_mode , packet1.control_value , packet1.enable );
    mavlink_msg_robomaster_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.control_mode , packet1.control_value , packet1.enable );
    mavlink_msg_robomaster_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robomaster_motor_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.control_mode , packet1.control_value , packet1.enable );
    mavlink_msg_robomaster_motor_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOMASTER_MOTOR_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL) != NULL);
#endif
}

static void mavlink_test_robomaster_motor_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robomaster_motor_status_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504,18483,18587,18691,18795,18899,235,46,113,180,247
    };
    mavlink_robomaster_motor_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current_position = packet_in.current_position;
        packet1.current_velocity = packet_in.current_velocity;
        packet1.target_position = packet_in.target_position;
        packet1.target_velocity = packet_in.target_velocity;
        packet1.last_command_time = packet_in.last_command_time;
        packet1.last_feedback_time = packet_in.last_feedback_time;
        packet1.current_milliamps = packet_in.current_milliamps;
        packet1.target_current = packet_in.target_current;
        packet1.error_count = packet_in.error_count;
        packet1.timeout_count = packet_in.timeout_count;
        packet1.overheat_count = packet_in.overheat_count;
        packet1.motor_id = packet_in.motor_id;
        packet1.temperature = packet_in.temperature;
        packet1.control_mode = packet_in.control_mode;
        packet1.enabled = packet_in.enabled;
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robomaster_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_pack(system_id, component_id, &msg , packet1.motor_id , packet1.current_position , packet1.current_velocity , packet1.current_milliamps , packet1.temperature , packet1.target_position , packet1.target_velocity , packet1.target_current , packet1.control_mode , packet1.enabled , packet1.status , packet1.error_count , packet1.timeout_count , packet1.overheat_count , packet1.last_command_time , packet1.last_feedback_time );
    mavlink_msg_robomaster_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.current_position , packet1.current_velocity , packet1.current_milliamps , packet1.temperature , packet1.target_position , packet1.target_velocity , packet1.target_current , packet1.control_mode , packet1.enabled , packet1.status , packet1.error_count , packet1.timeout_count , packet1.overheat_count , packet1.last_command_time , packet1.last_feedback_time );
    mavlink_msg_robomaster_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robomaster_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.current_position , packet1.current_velocity , packet1.current_milliamps , packet1.temperature , packet1.target_position , packet1.target_velocity , packet1.target_current , packet1.control_mode , packet1.enabled , packet1.status , packet1.error_count , packet1.timeout_count , packet1.overheat_count , packet1.last_command_time , packet1.last_feedback_time );
    mavlink_msg_robomaster_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOMASTER_MOTOR_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS) != NULL);
#endif
}

static void mavlink_test_robomaster_motor_config(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robomaster_motor_config_t packet_in = {
        17.0,17,84,151
    };
    mavlink_robomaster_motor_config_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_value = packet_in.param_value;
        packet1.motor_id = packet_in.motor_id;
        packet1.param_id = packet_in.param_id;
        packet1.save_to_flash = packet_in.save_to_flash;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_config_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robomaster_motor_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_config_pack(system_id, component_id, &msg , packet1.motor_id , packet1.param_id , packet1.param_value , packet1.save_to_flash );
    mavlink_msg_robomaster_motor_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_config_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.param_id , packet1.param_value , packet1.save_to_flash );
    mavlink_msg_robomaster_motor_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robomaster_motor_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_config_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.param_id , packet1.param_value , packet1.save_to_flash );
    mavlink_msg_robomaster_motor_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOMASTER_MOTOR_CONFIG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG) != NULL);
#endif
}

static void mavlink_test_robomaster_telemetry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robomaster_telemetry_t packet_in = {
        963497464,45.0,17651,163,230,41,108
    };
    mavlink_robomaster_telemetry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.system_uptime = packet_in.system_uptime;
        packet1.supply_voltage = packet_in.supply_voltage;
        packet1.can_error_count = packet_in.can_error_count;
        packet1.motors_active = packet_in.motors_active;
        packet1.can_status = packet_in.can_status;
        packet1.emergency_stop = packet_in.emergency_stop;
        packet1.motor_mask = packet_in.motor_mask;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_telemetry_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robomaster_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_telemetry_pack(system_id, component_id, &msg , packet1.motors_active , packet1.can_status , packet1.can_error_count , packet1.system_uptime , packet1.emergency_stop , packet1.supply_voltage , packet1.motor_mask );
    mavlink_msg_robomaster_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_telemetry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motors_active , packet1.can_status , packet1.can_error_count , packet1.system_uptime , packet1.emergency_stop , packet1.supply_voltage , packet1.motor_mask );
    mavlink_msg_robomaster_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robomaster_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_telemetry_send(MAVLINK_COMM_1 , packet1.motors_active , packet1.can_status , packet1.can_error_count , packet1.system_uptime , packet1.emergency_stop , packet1.supply_voltage , packet1.motor_mask );
    mavlink_msg_robomaster_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOMASTER_TELEMETRY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY) != NULL);
#endif
}

static void mavlink_test_robomaster(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_robomaster_motor_control(system_id, component_id, last_msg);
    mavlink_test_robomaster_motor_status(system_id, component_id, last_msg);
    mavlink_test_robomaster_motor_config(system_id, component_id, last_msg);
    mavlink_test_robomaster_telemetry(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ROBOMASTER_TESTSUITE_H
