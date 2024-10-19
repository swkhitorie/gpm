#include "mavlink_vehicle.h"

void MavlinkVehicle::handle_msg_mission_download_from_vehicle(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_mission_request_list_t missionrq_t;
    mavlink_msg_mission_request_list_decode(&_rrxmsg, &missionrq_t);
    //reject all
    _msgpak.pack_missionack(_rrxmsg.sysid, _rrxmsg.compid, MAV_MISSION_UNSUPPORTED, missionrq_t.mission_type, 0);    
    _msgpak.send_msg(chan, MAVLINK_MSG_ID_MISSION_ACK);
}

void MavlinkVehicle::handle_msg_mission_start_upload_to_vehicle(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_mission_count_t missioncnt_t;    
    mavlink_msg_mission_count_decode(&_rrxmsg, &missioncnt_t);
    
    if (missioncnt_t.target_component == mavlink_system.compid && 
        missioncnt_t.target_system == mavlink_system.sysid &&
        missioncnt_t.mission_type == MAV_MISSION_TYPE_MISSION) {
        _now_upmission.mission_sysid = _rrxmsg.sysid;
        _now_upmission.mission_compid = _rrxmsg.compid;
        _now_upmission.mission_count = missioncnt_t.count;
        _now_upmission.mission_type = missioncnt_t.mission_type;
        _now_upmission.mission_seq = 0;
            
        _msgpak.pack_missionrequestint(_now_upmission.mission_sysid, _now_upmission.mission_compid, 
            _now_upmission.mission_seq, _now_upmission.mission_type);
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_MISSION_REQUEST_INT);
        _now_upmission.active = true;
        _now_upmission.mission_seq++;
    }
    
    if (missioncnt_t.mission_type == MAV_MISSION_TYPE_FENCE && 
        missioncnt_t.mission_type == MAV_MISSION_TYPE_RALLY) {
        //reject all
        _msgpak.pack_missionack(_rrxmsg.sysid, _rrxmsg.compid, MAV_MISSION_UNSUPPORTED, missioncnt_t.mission_type, 0); 
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_MISSION_ACK);            
    }
}

void MavlinkVehicle::handle_msg_mission_upload_to_vehicle(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_mission_item_int_t missionitem_t;
    mavlink_msg_mission_item_int_decode(&_rrxmsg, &missionitem_t);
    char result[50];
    if (missionitem_t.target_component == mavlink_system.compid && 
        missionitem_t.target_system == mavlink_system.sysid && _now_upmission.active) {
        upload_mission[missionitem_t.seq] = missionitem_t;
        
        if (missionitem_t.seq == _now_upmission.mission_count - 1) {
            _msgpak.pack_missionack(_now_upmission.mission_sysid, _now_upmission.mission_compid, 
                MAV_MISSION_ACCEPTED, _now_upmission.mission_type, 0); 
            _msgpak.send_msg(chan, MAVLINK_MSG_ID_MISSION_ACK);
            
            for (int i = 0; i <= missionitem_t.seq; i++) {
				switch(upload_mission[i].command) {
				case MAV_CMD_NAV_TAKEOFF: sprintf(result, "step takeoff, seq: %d, count: %d", 
					upload_mission[i].seq, _now_upmission.mission_count); break;
				case MAV_CMD_NAV_RETURN_TO_LAUNCH: sprintf(result, "step return to launch, seq: %d, count: %d",
					upload_mission[i].seq, _now_upmission.mission_count); break;
				case MAV_CMD_NAV_WAYPOINT: sprintf(result, "x: %d, y: %d, z:%.1f", upload_mission[i].x, 
						upload_mission[i].y, upload_mission[i].z); break;
				}
                mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, result, 0, 0);
            }
            _now_upmission.active = false;
            _upload_times++;
            return;
        } else {
            _msgpak.pack_missionrequestint(_now_upmission.mission_sysid, _now_upmission.mission_compid, 
                _now_upmission.mission_seq, _now_upmission.mission_type);
            _msgpak.send_msg(chan, MAVLINK_MSG_ID_MISSION_REQUEST_INT);
            _now_upmission.mission_seq++;
        }
    }
}

void MavlinkVehicle::handle_msg_mission_get(mavlink_mission_item_int_t *p, uint8_t *size, uint16_t *times)
{
    p = &upload_mission[0];
    *size = _now_upmission.mission_count;
    *times = _upload_times;
}