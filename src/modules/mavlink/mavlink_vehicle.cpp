#include "mavlink_vehicle.h"
mavlink_system_t mavlink_system = { 1, 1 };
MavlinkVehicle::MavlinkVehicle(mavlink_channel_t channel, uint8_t sysid, uint8_t compid) : 
    MavlinkStream(channel), rtime_boot_ms(0), link_sysid(sysid), link_compid(compid)
{
    _now_upmission.active = false;
    _now_dwparams.active = false;
    start_cal_accel_sample = false;
    state_machine = TRANSFER_IDLE;
    _upload_times = 0;
    
    _now_logfile.isfilefound = false;
    _now_logfile.istransfering = false;
    log_file_trans_cnt = 0;
    
    GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "mavlink reset");
    _msgpak.pack_heartbeat(MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, /* ARMD */
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | 
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | 
        MAV_MODE_FLAG_STABILIZE_ENABLED | 
        MAV_MODE_FLAG_GUIDED_ENABLED | 
        MAV_PROTOCOL_CAPABILITY_FTP, 
        //MAV_MODE_FLAG_SAFETY_ARMED,
        0, 
        MAV_STATE_STANDBY);
}


