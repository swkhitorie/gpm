#include "mavlink_vehicle.h"
#include <ctime>
#include <chrono>
#include <stdlib.h>

void MavlinkVehicle::parsing(uint8_t c)
{
    int res = mavlink_frame_char(chan, c, &rxmsg, &rxstatus);
    switch (res) {
    case MAVLINK_FRAMING_INCOMPLETE: break;
    case MAVLINK_FRAMING_BAD_CRC: break;
    case MAVLINK_FRAMING_OK: handle_msg_recv(&rxmsg); break;
    }
}

void MavlinkVehicle::handle_msg_recv(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    
    switch (_rrxmsg.msgid) {
    case MAVLINK_MSG_ID_PING: handle_msg_ping(msg); break; 
    case MAVLINK_MSG_ID_HEARTBEAT: handle_msg_heartbeat(msg); break;
    case MAVLINK_MSG_ID_TIMESYNC: handle_msg_timesync(msg); break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: handle_msg_param_request_list(msg); break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: handle_msg_param_request_read(msg); break;
    case MAVLINK_MSG_ID_PARAM_SET: handle_msg_param_set(msg); break;
    case MAVLINK_MSG_ID_COMMAND_LONG: handle_msg_command(msg); break;
    case MAVLINK_MSG_ID_COMMAND_ACK: handle_msg_command_ack(msg); break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: handle_msg_mission_download_from_vehicle(msg); break;
    case MAVLINK_MSG_ID_MISSION_COUNT: handle_msg_mission_start_upload_to_vehicle(msg); break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: handle_msg_mission_upload_to_vehicle(msg); break;
    case MAVLINK_MSG_ID_SET_MODE: handle_msg_set_mode(msg); break;
    case MAVLINK_MSG_ID_SYSTEM_TIME: handle_msg_systime(msg); break;
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: handle_msg_ftp(msg); break;
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: handle_msg_request_data_stream(msg); break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST : handle_msg_log_request_list(msg); break;
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA: handle_msg_log_request_data(msg); break;
    case MAVLINK_MSG_ID_LOG_ERASE : handle_msg_log_erase_all(msg); break;
    default : GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "unsupport msg id(#%d)", _rrxmsg.msgid); break;
    }
}

void MavlinkVehicle::handle_msg_timesync(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_timesync_t ty_t;
    mavlink_msg_timesync_decode(&_rrxmsg, &ty_t);
}

void MavlinkVehicle::handle_msg_heartbeat(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_heartbeat_t hb_t;
    mavlink_msg_heartbeat_decode(&_rrxmsg, &hb_t);
    
    rx_heartbeat_cnt = rtime_boot_ms;
}

void MavlinkVehicle::handle_msg_ping(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_ping_t p_t;
    mavlink_msg_ping_decode(&_rrxmsg, &p_t);
}

void MavlinkVehicle::handle_msg_set_mode(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_set_mode_t setmode_t;
    mavlink_msg_set_mode_decode(&_rrxmsg, &setmode_t);  
    _msgpak.set_heartbeat_mode(setmode_t.base_mode, setmode_t.custom_mode);
}

void MavlinkVehicle::handle_msg_systime(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_system_time_t systime_t;
    mavlink_msg_system_time_decode(&_rrxmsg, &systime_t);

    time_t tv_sec = systime_t.time_unix_usec / 1000000ULL;
    long tv_nsec = (systime_t.time_unix_usec % 1000000ULL) * 1000ULL;
    tv_sec += 8 * 60 * 60; // change to Beijing timezone
    
    std::tm *now = std::localtime(&tv_sec);
    
    int year = now->tm_year + 1900;
    int month = now->tm_mon + 1;
    int day = now->tm_mday;
    int hour = now->tm_hour;
    int min = now->tm_min;
    int sec = now->tm_sec;
    
    mavlink_log_info(MAVLINK_COMM_1, "date: %d-%.2d-%.2d %d:%.2d:%.2d", year, month, day, hour, min, sec);
}

void MavlinkVehicle::handle_msg_request_data_stream(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_request_data_stream_t req_t;
    mavlink_msg_request_data_stream_decode(&_rrxmsg, &req_t);  
    
    uint8_t stream_id = req_t.req_stream_id;
    uint16_t msg_rate = req_t.req_message_rate;
    uint8_t start_stop = req_t.start_stop;
    
    /*
    1 SYS_STATUS
    2 SYSTEM_TIME 
    3
    6 CHANGE_OPERATOR_CONTROL_ACK
    10
    11 SET_MODE
    12
    */
    mavlink_msg_data_stream_send(chan, stream_id, msg_rate, 0);
}
