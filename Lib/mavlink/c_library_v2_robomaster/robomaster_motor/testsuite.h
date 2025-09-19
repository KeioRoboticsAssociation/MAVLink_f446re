/** @file
 *    @brief MAVLink comm protocol testsuite generated from robomaster_motor.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ROBOMASTER_MOTOR_TESTSUITE_H
#define ROBOMASTER_MOTOR_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_robomaster_motor(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_robomaster_motor(system_id, component_id, last_msg);
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
        17.0,45.0,73.0,963498088,53,120,187,254
    };
    mavlink_robomaster_motor_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.duty_cycle = packet_in.duty_cycle;
        packet1.target_position_rad = packet_in.target_position_rad;
        packet1.target_speed_rad_s = packet_in.target_speed_rad_s;
        packet1.timeout_ms = packet_in.timeout_ms;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
        
        
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
    mavlink_msg_robomaster_motor_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.duty_cycle , packet1.target_position_rad , packet1.target_speed_rad_s , packet1.timeout_ms );
    mavlink_msg_robomaster_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.duty_cycle , packet1.target_position_rad , packet1.target_speed_rad_s , packet1.timeout_ms );
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
    mavlink_msg_robomaster_motor_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.duty_cycle , packet1.target_position_rad , packet1.target_speed_rad_s , packet1.timeout_ms );
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
        17.0,45.0,73.0,101.0,963498296,65,132,199
    };
    mavlink_robomaster_motor_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current_position_rad = packet_in.current_position_rad;
        packet1.current_speed_rad_s = packet_in.current_speed_rad_s;
        packet1.current_duty_cycle = packet_in.current_duty_cycle;
        packet1.position_error_rad = packet_in.position_error_rad;
        packet1.timestamp_ms = packet_in.timestamp_ms;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
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
    mavlink_msg_robomaster_motor_status_pack(system_id, component_id, &msg , packet1.motor_id , packet1.control_mode , packet1.status , packet1.current_position_rad , packet1.current_speed_rad_s , packet1.current_duty_cycle , packet1.position_error_rad , packet1.timestamp_ms );
    mavlink_msg_robomaster_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.control_mode , packet1.status , packet1.current_position_rad , packet1.current_speed_rad_s , packet1.current_duty_cycle , packet1.position_error_rad , packet1.timestamp_ms );
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
    mavlink_msg_robomaster_motor_status_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.control_mode , packet1.status , packet1.current_position_rad , packet1.current_speed_rad_s , packet1.current_duty_cycle , packet1.position_error_rad , packet1.timestamp_ms );
    mavlink_msg_robomaster_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOMASTER_MOTOR_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS) != NULL);
#endif
}

static void mavlink_test_robomaster_motor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_robomaster_motor_control(system_id, component_id, last_msg);
    mavlink_test_robomaster_motor_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ROBOMASTER_MOTOR_TESTSUITE_H
