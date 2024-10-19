#include "mavlink_vehicle.h"

/**
    MAVLINK_MSG_ID_SYS_STATUS   1hz
    MAVLINK_MSG_ID_TIMESYNC     0.1hz
    MAVLINK_MSG_ID_RAW_IMU
    MAVLINK_MSG_ID_HOME_POSITION
*/
void MavlinkVehicle::broadcast(uint64_t time_boot_ms)
{
    rtime_boot_ms = time_boot_ms;

    broadcast_msg_freqsplit(time_boot_ms, 1, 1000, MAVLINK_MSG_ID_HEARTBEAT);
    broadcast_msg_freqsplit(time_boot_ms, 2, 1000, MAVLINK_MSG_ID_SYSTEM_TIME);
    broadcast_msg_freqsplit(time_boot_ms, 13, 10000, MAVLINK_MSG_ID_TIMESYNC);
    
    if (!handle_msg_param_download_m2()) {
        if ((rtime_boot_ms - rx_heartbeat_cnt) <= 3000) {
            broadcast_msg_freqsplit(time_boot_ms, 3, 20, MAVLINK_MSG_ID_HIGHRES_IMU);
            broadcast_msg_freqsplit(time_boot_ms, 4, 20, MAVLINK_MSG_ID_ATTITUDE);
            broadcast_msg_freqsplit(time_boot_ms, 5, 200, MAVLINK_MSG_ID_GPS_RAW_INT);
            broadcast_msg_freqsplit(time_boot_ms, 6, 50, MAVLINK_MSG_ID_LOCAL_POSITION_NED);
            broadcast_msg_freqsplit(time_boot_ms, 7, 100, MAVLINK_MSG_ID_ALTITUDE);
            broadcast_msg_freqsplit(time_boot_ms, 8, 200, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
            broadcast_msg_freqsplit(time_boot_ms, 9, 1000, MAVLINK_MSG_ID_MEMINFO);
            broadcast_msg_freqsplit(time_boot_ms, 10, 20, MAVLINK_MSG_ID_DISTANCE_SENSOR);
            broadcast_msg_freqsplit(time_boot_ms, 11, 500, MAVLINK_MSG_ID_RC_CHANNELS);
            broadcast_msg_freqsplit(time_boot_ms, 12, 500, MAVLINK_MSG_ID_BATTERY_STATUS);
        }
    }
    handle_msg_command_report();

    #if MAVLINK_LOGFILE_TRANSFER_METHOD == (2)
        if (++log_file_trans_cnt > 50) {
            log_file_trans_cnt = 1; 
            mavlink_log_data_process(this);
        }
    #endif
}

void MavlinkVehicle::broadcast_msg_freqsplit(uint64_t time_boot_ms, uint8_t slice_id, uint16_t period, uint16_t msg_id)
{
    if (slice_id > 29 || slice_id < 0) return;
    if (period < 1) return; 
    
    if ((time_boot_ms - freq_slice[slice_id]) > period) {
        freq_slice[slice_id] = time_boot_ms;
        _msgpak.send_msg(chan, msg_id);
    }
}
