#ifndef MAVLINKMSGPACK_H_
#define MAVLINKMSGPACK_H_

#include "mavlinkstream.hpp"

struct MavPacking
{
public:
	enum {
		MAX_ITEM_NUM = 15,
	};
	
    typedef enum {
        DISTANCE_0 = 0x00,
        DISTANCE_1,
        DISTANCE_2,
        DISTANCE_3,
        DISTANCE_4,
        DISTANCE_5,
        DISTANCE_6,
        DISTANCE_7,
        DISTANCE_8,
        DISTANCE_9,
        DISTANCE_NUM
    } distance_sensor_idx;
    
	MavPacking() : distance_sensor_step(0) {}
	~MavPacking() = default;

	void send_msg(mavlink_channel_t _chan, uint16_t msg_id)
	{
		switch (msg_id) {
		case MAVLINK_MSG_ID_PING:                   mavlink_msg_ping_send_struct(_chan, &mping); break;
		case MAVLINK_MSG_ID_AUTOPILOT_VERSION:      mavlink_msg_autopilot_version_send_struct(_chan, &mauversion); break;
		case MAVLINK_MSG_ID_STATUSTEXT:             mavlink_msg_statustext_send_struct(_chan, &mstatustext); break; 

        case MAVLINK_MSG_ID_HOME_POSITION:          if (_rq_homepos) {mavlink_msg_home_position_send_struct(_chan, &mhomepos);} break;
        
        case MAVLINK_MSG_ID_COMMAND_LONG:           mavlink_msg_command_long_send_struct(_chan, &cmd_long); break;
        case MAVLINK_MSG_ID_COMMAND_ACK:            mavlink_msg_command_ack_send_struct(_chan, &cmd_ack); break;
        case MAVLINK_MSG_ID_MISSION_ACK:            mavlink_msg_mission_ack_send_struct(_chan, &mission_ack); break; 
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:    mavlink_msg_mission_request_int_send_struct(_chan, &mission_request); break; 
                
        case MAVLINK_MSG_ID_MEMINFO:                mavlink_msg_meminfo_send_struct(_chan, &mmeminfo); break;
		case MAVLINK_MSG_ID_HEARTBEAT:              mavlink_msg_heartbeat_send_struct(_chan, &mheartbeat); break;
		case MAVLINK_MSG_ID_TIMESYNC:               mavlink_msg_timesync_send_struct(_chan, &mtimesync); break;
		case MAVLINK_MSG_ID_SYSTEM_TIME:            mavlink_msg_system_time_send_struct(_chan, &msystemtime); break;
		case MAVLINK_MSG_ID_SYS_STATUS:             mavlink_msg_sys_status_send_struct(_chan, &msystemstatus); break;
			
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:     mavlink_msg_local_position_ned_send_struct(_chan, &mlpned); break;
        case MAVLINK_MSG_ID_ALTITUDE:               mavlink_msg_altitude_send_struct(_chan, &maltitude); break;
		case MAVLINK_MSG_ID_ATTITUDE:               mavlink_msg_attitude_send_struct(_chan, &mattitude); break;
		case MAVLINK_MSG_ID_BATTERY_STATUS:         mavlink_msg_battery_status_send_struct(_chan, &mbattery); break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:            mavlink_msg_gps_raw_int_send_struct(_chan, &mgpsraw); break;
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:    mavlink_msg_global_position_int_send_struct(_chan, &mglobalposition); break;
		case MAVLINK_MSG_ID_RAW_IMU:                mavlink_msg_raw_imu_send_struct(_chan, &mrawimu); break;
        case MAVLINK_MSG_ID_HIGHRES_IMU:            mavlink_msg_highres_imu_send_struct(_chan, &mhighimu); break;
		case MAVLINK_MSG_ID_RC_CHANNELS:            mavlink_msg_rc_channels_send_struct(_chan, &mrcchannels); break;        
		case MAVLINK_MSG_ID_SCALED_IMU2:            mavlink_msg_scaled_imu2_send_struct(_chan, &mscaledimu2); break;
		case MAVLINK_MSG_ID_SCALED_PRESSURE:        mavlink_msg_scaled_pressure_send_struct(_chan, &mscaledpressure); break;
		case MAVLINK_MSG_ID_DISTANCE_SENSOR:
			{
				mavlink_msg_distance_sensor_send_struct(_chan, &mdistancesensor[distance_sensor_step]);
				distance_sensor_step++;
				if (distance_sensor_step > DISTANCE_NUM - 1) distance_sensor_step = DISTANCE_0;
				break;
			}
		default: break;
		}
	}

	void set_heartbeat_mode(uint8_t base_mode, uint32_t custom_mode)
	{
		mheartbeat.base_mode = base_mode;
		mheartbeat.custom_mode = custom_mode;	
	}
	
	void get_heartbeat_mode(uint8_t &base_mode, uint32_t &custom_mode, uint8_t &system_status)
    {
		base_mode = mheartbeat.base_mode;
		custom_mode = mheartbeat.custom_mode;        
        system_status = mheartbeat.system_status;
    }
	
    void pack_autopilotversion(uint64_t capabilities, uint32_t flight_sw_version, uint32_t middleware_sw_version, uint32_t os_sw_version, uint32_t board_version, const uint8_t *flight_custom_version, const uint8_t *middleware_custom_version, const uint8_t *os_custom_version, uint16_t vendor_id, uint16_t product_id, uint64_t uid, const uint8_t *uid2)
    {
        mauversion.flight_sw_version = flight_sw_version;
        mauversion.middleware_sw_version = middleware_sw_version;
        mauversion.os_sw_version = os_sw_version;
        mauversion.board_version = board_version;
        mauversion.vendor_id = vendor_id;
        mauversion.product_id = product_id;
        mauversion.uid = uid;
        for (int i = 0; i < 8; i++) {
            mauversion.flight_custom_version[i] = flight_custom_version[i];
            mauversion.middleware_custom_version[i] = middleware_custom_version[i];
            mauversion.os_custom_version[i] = os_custom_version[i];
        }
        
        for (int i = 0; i < 18; i++)
            mauversion.uid2[i] = uid2[i];
    }

    void pack_heartbeat(uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
    {
		mheartbeat.type = type;
		mheartbeat.autopilot = autopilot;
		mheartbeat.base_mode = base_mode;
		mheartbeat.custom_mode = custom_mode;
		mheartbeat.system_status = system_status;
		mheartbeat.mavlink_version = 3;
    }
    void pack_timesync(int64_t tc1, int64_t ts1)
    {
		mtimesync.tc1 = tc1;
		mtimesync.ts1 = ts1;
    }
    
    void pack_systemtime(uint64_t time_unix_usec, uint32_t time_boot_ms)
    {
		msystemtime.time_unix_usec = time_unix_usec;
		msystemtime.time_boot_ms = time_boot_ms;
    }
    
    void pack_commandlong(uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
    {
        cmd_long.target_system = target_system;
        cmd_long.target_component = target_component;
        cmd_long.command = command;
        cmd_long.confirmation = confirmation;
        
        cmd_long.param1 = param1;
        cmd_long.param2 = param2;
        cmd_long.param3 = param3;
        cmd_long.param4 = param4;
        cmd_long.param5 = param5;
        cmd_long.param6 = param6;
        cmd_long.param7 = param7;
    }
    
    void pack_commandack(uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component)
    {
        cmd_ack.command = command;
        cmd_ack.result = result;
        cmd_ack.progress = progress;
        cmd_ack.result_param2 = result_param2;
        cmd_ack.target_system = target_system;
        cmd_ack.target_component = target_component;   
    }
    
    void pack_missionack(uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mission_type, uint32_t opaque_id)
    {
        mission_ack.target_system = target_system;
        mission_ack.target_component = target_component;
        mission_ack.type = type;
        mission_ack.mission_type = mission_type;
        mission_ack.opaque_id = opaque_id;
    }
    
    void pack_missionrequestint(uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t mission_type)
    {
        mission_request.target_system = target_system;
        mission_request.target_component = target_component;
        mission_request.seq = seq;
        mission_request.mission_type = mission_type;
    }
    
    void pack_sys_status(uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, 
            uint32_t onboard_control_sensors_health, uint16_t load, uint16_t voltage_battery, int16_t current_battery, 
            int8_t battery_remaining, uint16_t drop_rate_comm = 0, uint16_t errors_comm = 0, uint16_t errors_count1 = 0, 
            uint16_t errors_count2 = 0, uint16_t errors_count3 = 0, uint16_t errors_count4 = 0, 
            uint32_t onboard_control_sensors_present_extended = 0, uint32_t onboard_control_sensors_enabled_extended = 0, 
            uint32_t onboard_control_sensors_health_extended = 0)
	{
		msystemstatus.onboard_control_sensors_present = onboard_control_sensors_present;
		msystemstatus.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
		msystemstatus.onboard_control_sensors_health = onboard_control_sensors_health;
		msystemstatus.load = load;
		msystemstatus.voltage_battery = voltage_battery;
		msystemstatus.current_battery = current_battery;
		msystemstatus.battery_remaining = battery_remaining;
		
		msystemstatus.drop_rate_comm = drop_rate_comm;
		msystemstatus.errors_comm = errors_comm;
		msystemstatus.errors_count1 = errors_count1;
		msystemstatus.errors_count2 = errors_count2;
		msystemstatus.errors_count3 = errors_count3;
		msystemstatus.errors_count4 = errors_count4;
		msystemstatus.onboard_control_sensors_present_extended = onboard_control_sensors_present_extended;
		msystemstatus.onboard_control_sensors_enabled_extended = onboard_control_sensors_enabled_extended;
		msystemstatus.onboard_control_sensors_health_extended = onboard_control_sensors_health_extended;      
	}
    
    void pack_homepos(int32_t latitude, int32_t longitude, int32_t altitude, float x, float y, float z, const float *q, float approach_x, float approach_y, float approach_z, uint64_t time_usec)
    {
        mhomepos.latitude = latitude;
        mhomepos.longitude = longitude;
        mhomepos.altitude = altitude;
        mhomepos.x = x;
        mhomepos.y = y;
        mhomepos.z = z;
        mhomepos.approach_x = approach_x;
        mhomepos.approach_y = approach_y;
        mhomepos.approach_z = approach_z;
        mhomepos.time_usec = time_usec;
        for (int i = 0; i < 4; i++)
            mhomepos.q[i] = q[i];
    }
    
    void pack_localpositionned(uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
    {
        mlpned.time_boot_ms = time_boot_ms;
        mlpned.x = x;
        mlpned.y = y;
        mlpned.z = z;
        mlpned.vx = vx;
        mlpned.vy = vy;
        mlpned.vz = vz;
    }
    
    void pack_altitude(uint64_t time_usec, float altitude_monotonic, float altitude_amsl, float altitude_local, float altitude_relative, float altitude_terrain, float bottom_clearance)
    {
        maltitude.time_usec = time_usec;
        maltitude.altitude_monotonic = altitude_monotonic;
        maltitude.altitude_amsl = altitude_amsl;
        maltitude.altitude_local = altitude_local;
        maltitude.altitude_relative = altitude_relative;
        maltitude.altitude_terrain = altitude_terrain;
        maltitude.bottom_clearance = bottom_clearance;
    }
    
    void pack_attitude(uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
    {
		mattitude.time_boot_ms = time_boot_ms;
		mattitude.roll = roll;
		mattitude.pitch = pitch;
		mattitude.yaw = yaw;
		mattitude.rollspeed = rollspeed;
		mattitude.pitchspeed = pitchspeed;
		mattitude.yawspeed = yawspeed;
    }
    void pack_battery(uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, 
        int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining = 0, 
        uint8_t charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED, const uint16_t *voltages_ext = NULL, uint8_t mode = 0, uint32_t fault_bitmask = 0)
	{
		mbattery.id = id;
		mbattery.battery_function = battery_function;
		mbattery.type = type;
		mbattery.temperature = temperature;
		mbattery.current_battery = current_battery;
		mbattery.current_consumed = current_consumed;
		mbattery.energy_consumed = energy_consumed;
		mbattery.battery_remaining = battery_remaining;
		for (int i = 0; i < 10; i++)
			mbattery.voltages[i] = voltages[i];
		
		mbattery.time_remaining = time_remaining;
		mbattery.charge_state = charge_state;
		if (voltages_ext != NULL) {
			for (int i = 0; i < 4; i++) mbattery.voltages_ext[i] = voltages_ext[i];         
		}
		mbattery.mode = mode;
		mbattery.fault_bitmask = fault_bitmask;
	}
    void pack_gpsraw(uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, 
        uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, 
        uint32_t vel_acc, uint32_t hdg_acc, uint16_t yaw)
	{
		mgpsraw.time_usec = time_usec;
		mgpsraw.fix_type = fix_type;
		mgpsraw.lat = lat;
		mgpsraw.lon = lon;
		mgpsraw.alt = alt;
		mgpsraw.eph = eph;
		mgpsraw.epv = epv;
		mgpsraw.vel = vel;
		mgpsraw.cog = cog;
		mgpsraw.satellites_visible = satellites_visible;
		mgpsraw.alt_ellipsoid = alt_ellipsoid;
		mgpsraw.h_acc = h_acc;
		mgpsraw.v_acc = v_acc;
		mgpsraw.vel_acc = vel_acc;
		mgpsraw.hdg_acc = hdg_acc;
		mgpsraw.yaw = yaw;
	}
    void pack_globalposition(uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, 
        int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	{
		mglobalposition.time_boot_ms = time_boot_ms;
		mglobalposition.lat = lat;
		mglobalposition.lon = lon;
		mglobalposition.alt = alt;
		mglobalposition.relative_alt = relative_alt;
		mglobalposition.vx = vx;
		mglobalposition.vy = vy;
		mglobalposition.vz = vz;
		mglobalposition.hdg = hdg;    
	}
    
    void pack_highimu(uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, 
        float xmag, float ymag, float zmag, float abs_pressure, float diff_pressure, float pressure_alt, 
        float temperature, uint16_t fields_updated, uint8_t id)
    {
        mhighimu.time_usec = time_usec;
        mhighimu.xacc = xacc;
        mhighimu.yacc = yacc;
        mhighimu.zacc = zacc;
        mhighimu.xgyro = xgyro;
        mhighimu.ygyro = ygyro;
        mhighimu.zgyro = zgyro;
        mhighimu.xmag = xmag;
        mhighimu.ymag = ymag;
        mhighimu.zmag = zmag;
        mhighimu.abs_pressure = abs_pressure;
        mhighimu.diff_pressure = diff_pressure;
        mhighimu.pressure_alt = pressure_alt;
        mhighimu.temperature = temperature;
        mhighimu.fields_updated = fields_updated;
        mhighimu.id = id;
    }
    
    void pack_rawimu(uint64_t time_usec, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, 
        int16_t xmag, int16_t ymag, int16_t zmag, uint8_t id, int16_t temperature)
	{
		mrawimu.time_usec = time_usec;
		mrawimu.xacc = xacc;
		mrawimu.yacc = yacc;
		mrawimu.zacc = zacc;
		mrawimu.xgyro = xgyro;
		mrawimu.ygyro = ygyro;
		mrawimu.zgyro = zgyro;
		mrawimu.xmag = xmag;
		mrawimu.ymag = ymag;
		mrawimu.zmag = zmag;
		mrawimu.id = id;
		mrawimu.temperature = temperature;      
	}
    void pack_rcchannel(uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, 
        uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, 
        uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, 
        uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi)
	{
		mrcchannels.time_boot_ms = time_boot_ms;
		mrcchannels.chancount = chancount;
		mrcchannels.chan1_raw = chan1_raw;
		mrcchannels.chan2_raw = chan2_raw;
		mrcchannels.chan3_raw = chan3_raw;
		mrcchannels.chan4_raw = chan4_raw;
		mrcchannels.chan5_raw = chan5_raw;
		mrcchannels.chan6_raw = chan6_raw;
		mrcchannels.chan7_raw = chan7_raw;
		mrcchannels.chan8_raw = chan8_raw;
		mrcchannels.chan9_raw = chan9_raw;
		mrcchannels.chan10_raw = chan10_raw;
		mrcchannels.chan11_raw = chan11_raw;
		mrcchannels.chan12_raw = chan12_raw;
		mrcchannels.chan13_raw = chan13_raw;
		mrcchannels.chan14_raw = chan14_raw;
		mrcchannels.chan15_raw = chan15_raw;
		mrcchannels.chan16_raw = chan16_raw;  
		mrcchannels.chan17_raw = chan17_raw;
		mrcchannels.chan18_raw = chan18_raw;  
		mrcchannels.rssi = rssi;     
	}
    void pack_scaledimu2(uint32_t time_boot_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, 
        int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature)
	{
		mscaledimu2.time_boot_ms = time_boot_ms;
		mscaledimu2.xacc = xacc;
		mscaledimu2.yacc = yacc;
		mscaledimu2.zacc = zacc;
		mscaledimu2.xgyro = xgyro;
		mscaledimu2.ygyro = ygyro;
		mscaledimu2.zgyro = zgyro;
		mscaledimu2.xmag = xmag;
		mscaledimu2.ymag = ymag;
		mscaledimu2.zmag = zmag;
		mscaledimu2.temperature = temperature;        
	}
    void pack_scaledpressure(uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff)
    {
		mscaledpressure.temperature = temperature;
		mscaledpressure.time_boot_ms = time_boot_ms;
		mscaledpressure.press_abs = press_abs;
		mscaledpressure.press_diff = press_diff;
		mscaledpressure.temperature_press_diff = temperature_press_diff;
    }
    void pack_distancesensor(distance_sensor_idx idx, uint32_t time_boot_ms, uint16_t min_distance, uint16_t max_distance, uint16_t current_distance, 
            uint8_t type, uint8_t id, uint8_t orientation, uint8_t covariance = 255, float horizontal_fov = 0, 
            float vertical_fov = 0, const float *quaternion = NULL, uint8_t signal_quality = 100)
	{
		mdistancesensor[idx].time_boot_ms = time_boot_ms;
		mdistancesensor[idx].min_distance = min_distance;
		mdistancesensor[idx].max_distance = max_distance;
		mdistancesensor[idx].current_distance = current_distance;
		mdistancesensor[idx].type = type;
		mdistancesensor[idx].id = id;
		mdistancesensor[idx].orientation = orientation;
		
		mdistancesensor[idx].covariance = covariance;
		mdistancesensor[idx].horizontal_fov = horizontal_fov;
		mdistancesensor[idx].vertical_fov = vertical_fov;
		if (quaternion != NULL) {
			mdistancesensor[idx].quaternion[0] = quaternion[0];
			mdistancesensor[idx].quaternion[1] = quaternion[1];
			mdistancesensor[idx].quaternion[2] = quaternion[2];
			mdistancesensor[idx].quaternion[3] = quaternion[3];
		}
		mdistancesensor[idx].signal_quality = signal_quality;   
	}
    
    void pack_meminfo(uint16_t brkval, uint16_t freemem, uint32_t freemem32)
    {
        mmeminfo.brkval = brkval;
        mmeminfo.freemem = freemem;
        mmeminfo.freemem32 = freemem32;
    }
    
    bool                                _rq_homepos{false};
protected:
    uint8_t distance_sensor_step;

    mavlink_ping_t                      mping;
    mavlink_autopilot_version_t         mauversion;
    mavlink_statustext_t                mstatustext;
    
    // broadcast msg struct
    mavlink_heartbeat_t                 mheartbeat;
    mavlink_timesync_t                  mtimesync;
    mavlink_system_time_t               msystemtime;
    mavlink_sys_status_t                msystemstatus;

    mavlink_local_position_ned_t        mlpned;
    mavlink_altitude_t                  maltitude;
    mavlink_attitude_t                  mattitude;
    mavlink_battery_status_t            mbattery;
    mavlink_gps_raw_int_t               mgpsraw;
    mavlink_global_position_int_t       mglobalposition;
    mavlink_raw_imu_t                   mrawimu;
    mavlink_highres_imu_t               mhighimu;
    mavlink_rc_channels_t               mrcchannels;
    mavlink_scaled_imu2_t               mscaledimu2;
    mavlink_scaled_pressure_t           mscaledpressure;
    mavlink_distance_sensor_t           mdistancesensor[DISTANCE_NUM];

    mavlink_power_status_t              mpowerstatus;
    mavlink_ekf_status_report_t         mekfstatus;
    mavlink_ahrs_t                      mahrs;
    mavlink_ahrs2_t                     mahrs2;
    mavlink_meminfo_t                   mmeminfo;
    mavlink_mission_current_t           mmissioncurrent;
    mavlink_nav_controller_output_t     mnavcontroloutput;
    mavlink_servo_output_raw_t          mservooutputraw;
    mavlink_terrain_report_t            mterrainreport;
    mavlink_vfr_hud_t                   mvfrhud;
    mavlink_vibration_t                 mvibration;
    
    mavlink_home_position_t             mhomepos;
    
    // order
    mavlink_command_ack_t               cmd_ack;
    mavlink_command_long_t              cmd_long;
    mavlink_mission_ack_t               mission_ack;
    mavlink_mission_request_int_t       mission_request;
};

#endif
