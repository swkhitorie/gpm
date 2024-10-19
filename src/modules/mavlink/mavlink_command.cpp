#include "mavlink_vehicle.h"

void MavlinkVehicle::handle_msg_command_report()
{
    if (state_machine == TRANSFER_IDLE) return;
    switch (state_machine) {
    case GYRO_CALIBRATION:
        calibration_gyro_process(); 
        break; 
    case ACCEL_CALIBTATION: 
        calibration_accel_process();
        break;
    case MAG_CALIBRATION: 
        calibration_compass_process();
        break;
    default: break;
    }
}

void MavlinkVehicle::handle_msg_command(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_command_long_t cmd_mavlink; 
    mavlink_msg_command_long_decode(&_rrxmsg, &cmd_mavlink);
    gcs_sysid = _rrxmsg.sysid;
    gcs_compid = _rrxmsg.compid;  
    char tmpstring[50];
    
    switch (cmd_mavlink.command) {
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
        handle_msg_command_autopilot_version(msg);
        break;
    case MAV_CMD_SET_MESSAGE_INTERVAL:
        handle_msg_command_set_message_interval(msg);
        break;
    case MAV_CMD_REQUEST_MESSAGE:
        handle_msg_command_request_message(msg);
        break;
    case MAV_CMD_REQUEST_PROTOCOL_VERSION: 
        handle_msg_command_request_protocol_version(msg);
        break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
        handle_msg_command_preflight_calibration(msg);
        break;
    case MAV_CMD_DO_START_MAG_CAL:
    case MAV_CMD_DO_CANCEL_MAG_CAL:
    case MAV_CMD_DO_ACCEPT_MAG_CAL:
    case MAV_CMD_FIXED_MAG_CAL:
    case MAV_CMD_FIXED_MAG_CAL_FIELD:
        handle_msg_command_mag_cal(msg);
        break;
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: Kernel::reboot(); break;
    default:
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "unhandle id(#%d)", cmd_mavlink.command);
        break;
    }
}

void MavlinkVehicle::handle_msg_command_autopilot_version(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    char tmpstring[50];
    uint64_t capabilities =  MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
    MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
    MAV_PROTOCOL_CAPABILITY_MISSION_INT |
    MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
    MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
    MAV_PROTOCOL_CAPABILITY_TERRAIN |
    MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
    MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION |
    MAV_PROTOCOL_CAPABILITY_FTP |
    MAV_PROTOCOL_CAPABILITY_MAVLINK2;
    //MAV_PROTOCOL_CAPABILITY_MISSION_FENCE |
    //MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;

    mavlink_msg_autopilot_version_send(chan, capabilities, 
        __flight_sw_version, __middleware_sw_version, __os_sw_version, __board_version, 
        __flight_custom_version, __middleware_custom_version, __os_custom_version,
        __vendor_id, __product_id, __uid, &__uid2[0]);

    mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &__flight_sw_version_string[0], 0, 0);
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &__os_sw_version_string[0], 0, 0);
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &__board_uid_string[0], 0, 0);
    sprintf(&tmpstring[0], "RCOut: PWM:1-14");
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &tmpstring[0], 0, 0);
    
    _msgpak.pack_commandack(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, MAV_RESULT_ACCEPTED, 255, 0, _rrxmsg.sysid, _rrxmsg.compid);
    _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);
}

void MavlinkVehicle::handle_msg_command_request_protocol_version(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    _msgpak.pack_commandack(MAV_CMD_REQUEST_PROTOCOL_VERSION, MAV_RESULT_UNSUPPORTED, 255, 0, _rrxmsg.sysid, _rrxmsg.compid);
    _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);  
}

void MavlinkVehicle::handle_msg_command_set_message_interval(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_command_long_t cmd_mavlink; 
    mavlink_msg_command_long_decode(&_rrxmsg, &cmd_mavlink);
    
    if (static_cast<int>(cmd_mavlink.param1) == MAVLINK_MSG_ID_HOME_POSITION) {
        _msgpak._rq_homepos = true;
    }else {
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "request id: %f", cmd_mavlink.param1);
    }
    _msgpak.pack_commandack(MAV_CMD_SET_MESSAGE_INTERVAL, MAV_RESULT_ACCEPTED, 255, 0, _rrxmsg.sysid, _rrxmsg.compid);
    _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);
}

void MavlinkVehicle::handle_msg_command_request_message(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_command_long_t cmd_mavlink; 
    mavlink_msg_command_long_decode(&_rrxmsg, &cmd_mavlink);

    if(static_cast<int>(cmd_mavlink.param1) == MAVLINK_MSG_ID_COMPONENT_INFORMATION || 
       static_cast<int>(cmd_mavlink.param1) == MAVLINK_MSG_ID_COMPONENT_METADATA) {
        _msgpak.pack_commandack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_UNSUPPORTED, 0, 0, _rrxmsg.sysid, _rrxmsg.compid);
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK); 
         
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "MAV_CMD_REQUEST_MESSAGE banned id: %f", cmd_mavlink.param1);
        return;
    } else if (static_cast<int>(cmd_mavlink.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
        handle_msg_command_autopilot_version(msg);
        _msgpak.pack_commandack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, 255, 0, _rrxmsg.sysid, _rrxmsg.compid);
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);  
        return;        
    } else {
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "MAV_CMD_REQUEST_MESSAGE other id: %f", cmd_mavlink.param1);           
    }

    _msgpak.pack_commandack(MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_UNSUPPORTED, 255, 0, _rrxmsg.sysid, _rrxmsg.compid);
    _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);
}

void MavlinkVehicle::handle_msg_command_preflight_calibration(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    gcs_sysid = _rrxmsg.sysid;
    gcs_compid = _rrxmsg.compid;
    
    mavlink_command_long_t cmd_mavlink; 
    mavlink_msg_command_long_decode(&_rrxmsg, &cmd_mavlink);

    if (static_cast<int>(cmd_mavlink.param1) == 1 || static_cast<int>(cmd_mavlink.param1) == 3) {
        state_machine = GYRO_CALIBRATION;
        _gyro_calibration->start();
        
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "gyro calibration start "); 
    } else if ( static_cast<int>(cmd_mavlink.param5) == 1 || 
                static_cast<int>(cmd_mavlink.param5) == 2 ||
                static_cast<int>(cmd_mavlink.param5) == 3 ||
                static_cast<int>(cmd_mavlink.param5) == 4) {
                    
        state_machine = ACCEL_CALIBTATION;
        cnt_cal_accel_pos = ACCELCAL_VEHICLE_POS_LEVEL;
        _msgpak.pack_commandlong(gcs_sysid, gcs_compid, MAV_CMD_ACCELCAL_VEHICLE_POS, 0, cnt_cal_accel_pos, 0, 0, 0, 0, 0, 0);
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_LONG);
        _accel_calibration->wait_orientation();
                    
    } else if (static_cast<int>(cmd_mavlink.param2) == 1) {
        state_machine = MAG_CALIBRATION;
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "preflight calibration mag"); 
    }
}

void MavlinkVehicle::handle_msg_command_ack(mavlink_message_t *msg)
{
    char tmpstring[50];
    mavlink_message_t _rrxmsg = *msg;
    mavlink_command_ack_t cmd_ack; 
    mavlink_msg_command_ack_decode(&_rrxmsg, &cmd_ack);

    if (cmd_ack.result == MAV_RESULT_TEMPORARILY_REJECTED) {
        switch (state_machine) {
        case TRANSFER_IDLE:break;
        case GYRO_CALIBRATION: break;
        case ACCEL_CALIBTATION:
            if (cnt_cal_accel_pos <= ACCELCAL_VEHICLE_POS_BACK) {
                start_cal_accel_sample = true;
                GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "(0) acc calib start ");
            }
            break;
        }
    }
}

void MavlinkVehicle::handle_msg_command_mag_cal(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_command_long_t cmd_mavlink; 
    mavlink_msg_command_long_decode(&_rrxmsg, &cmd_mavlink);
    
    switch (cmd_mavlink.command) {
    case MAV_CMD_DO_CANCEL_MAG_CAL:
        _msgpak.pack_commandack(MAV_CMD_DO_CANCEL_MAG_CAL, MAV_RESULT_ACCEPTED, 255, 0, _rrxmsg.sysid, _rrxmsg.compid);
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);
        break;   
    case MAV_CMD_DO_START_MAG_CAL:
        uint8_t mag_mask = cmd_mavlink.param1;
        bool retry = !math::isZero(cmd_mavlink.param2);
        bool autosave = !math::isZero(cmd_mavlink.param3);
        float delay = cmd_mavlink.param4;
    
        _compass_calibration->set_orientation(ROTATION_NONE, false, false, false);
        _compass_calibration->start(retry, delay, 1800, 0, 60);

        state_machine = MAG_CALIBRATION;
    
        _msgpak.pack_commandack(MAV_CMD_DO_START_MAG_CAL, MAV_RESULT_ACCEPTED, 255, 0, _rrxmsg.sysid, _rrxmsg.compid);
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);
        break;
    }
}

void MavlinkVehicle::calibration_accel_process()
{
    if (start_cal_accel_sample) {
        if (_accel_calibration->state() == calibration::AccelCalibratorBak::ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            _accel_calibration->next();
            GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "(1) acc calib next");
        } else if (_accel_calibration->complete(_accel_calibration->now_pos())) {
            start_cal_accel_sample = false;
            _accel_calibration->wait_orientation();
            
            if (cnt_cal_accel_pos == ACCELCAL_VEHICLE_POS_BACK) {
                _accel_calibration->clear();
                _accel_calibration->commit_params();
                
                _msgpak.pack_commandlong(gcs_sysid, gcs_compid, MAV_CMD_ACCELCAL_VEHICLE_POS, 0, ACCELCAL_VEHICLE_POS_SUCCESS, 0, 0, 0, 0, 0, 0);
                _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_LONG);
                GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "(final) %.3f %.3f %.3f, %.3f %.3f %.3f", _accel_calibration->offset(0), 
                        _accel_calibration->offset(1), _accel_calibration->offset(2), _accel_calibration->scale(0), 
                        _accel_calibration->scale(1), _accel_calibration->scale(2));
            } else {
                cnt_cal_accel_pos++;
                _msgpak.pack_commandlong(gcs_sysid, gcs_compid, MAV_CMD_ACCELCAL_VEHICLE_POS, 0, cnt_cal_accel_pos, 0, 0, 0, 0, 0, 0);
                _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_LONG); 
                GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "(2) acc %d %d %.3f, %.3f, %.3f", _accel_calibration->_orientaion_collected, 
                        _accel_calibration->_samples_completed[_accel_calibration->_orientaion_collected], 
                        _accel_calibration->_sample[_accel_calibration->_orientaion_collected](0), 
                        _accel_calibration->_sample[_accel_calibration->_orientaion_collected](1), 
                        _accel_calibration->_sample[_accel_calibration->_orientaion_collected](2));
            }
        }
    } 
}

void MavlinkVehicle::calibration_gyro_process()
{
    if (_gyro_calibration->state() == calibration::GyroscopeCalibrator::GYRO_CAL_SUCCESS) {
        _gyro_calibration->clear();
        _gyro_calibration->commit_params();
        
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "gyro offset: %.3f, %.3f, %.3f", 
        _gyro_calibration->offset(0), _gyro_calibration->offset(1), _gyro_calibration->offset(2));
        state_machine = TRANSFER_IDLE; 
        _msgpak.pack_commandack(MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_ACCEPTED, 255, 0, gcs_sysid, gcs_compid);
        _msgpak.send_msg(chan, MAVLINK_MSG_ID_COMMAND_ACK);
    }
}

void MavlinkVehicle::calibration_compass_process()
{
    static uint16_t slice = 1;
    if (++slice > 20) {
        slice = 1;
        _compass_calibration->update();
        if (_compass_calibration->running()) {
            struct calibration::CompassCalibrator::State _state = _compass_calibration->get_state();
            mavlink_msg_mag_cal_progress_send(chan, 0x00, 0x01, static_cast<uint8_t>(_state.status), 
                _state.attempt, static_cast<uint8_t>(_state.completion_pct), 
                _state.completion_mask, 0, 0, 0);  
        }
        
        if (_compass_calibration->get_state().status == calibration::CompassCalibrator::Status::SUCCESS ||
            _compass_calibration->get_state().status == calibration::CompassCalibrator::Status::FAILED) {

            char tmpstring[50];
            sprintf(&tmpstring[0], "mag diag %.3f, %.3f, %.3f", _compass_calibration->_params.diag(0), _compass_calibration->_params.diag(1), _compass_calibration->_params.diag(2));
            mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &tmpstring[0], 0, 0);
            sprintf(&tmpstring[0], "mag offdiag %.3f, %.3f, %.3f", _compass_calibration->_params.offdiag(0), _compass_calibration->_params.offdiag(1), _compass_calibration->_params.offdiag(2));
            mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &tmpstring[0], 0, 0);  
            sprintf(&tmpstring[0], "mag offset %.3f, %.3f, %.3f", _compass_calibration->_params.offset(0), _compass_calibration->_params.offset(1), _compass_calibration->_params.offset(2));
            mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &tmpstring[0], 0, 0);

            _compass_calibration->commit_params();

            struct calibration::CompassCalibrator::Report _report = _compass_calibration->get_report();
            mavlink_msg_mag_cal_report_send(chan, 0x00, 0x01, (uint8_t)_report.status, 0,
                _report.fitness,
                _report.ofs(0), _report.ofs(1), _report.ofs(2),
                _report.diag(0), _report.diag(1), _report.diag(2),
                _report.offdiag(0), _report.offdiag(1), _report.offdiag(2),
                _report.orientation_confidence,
                _report.original_orientation,
                _report.orientation,
                _report.scale_factor);
            _compass_calibration->stop();
        }
    }
}