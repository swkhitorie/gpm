#include "nmea0813.h"
#include "utility/log.hpp"

nmea0813::nmea0813(uint32_t _id, ESAF::ebus_uart *com) : device_id(_id), _com(com) {
    timestamp_start_block = 0;
//    com->write_through(ESAF::nmea0813_parser::cmd_turnon_gga, sizeof(ESAF::nmea0813_parser::cmd_turnon_gga), ESAF::EDev::BLOCK);
//    com->write_through(ESAF::nmea0813_parser::cmd_turnon_gsa, sizeof(ESAF::nmea0813_parser::cmd_turnon_gsa), ESAF::EDev::BLOCK);
//    com->write_through(ESAF::nmea0813_parser::cmd_turnon_rmc, sizeof(ESAF::nmea0813_parser::cmd_turnon_rmc), ESAF::EDev::BLOCK);
//    com->write_through(ESAF::nmea0813_parser::cmd_turnon_vtg, sizeof(ESAF::nmea0813_parser::cmd_turnon_vtg), ESAF::EDev::BLOCK);
//    com->write_through(ESAF::nmea0813_parser::cmd_turnon_gst, sizeof(ESAF::nmea0813_parser::cmd_turnon_gst), ESAF::EDev::BLOCK);
}
nmea0813::~nmea0813() {}
    
uint8_t nmea0813::parser_process(float dt)
{
	int res = ESAF::EDev::ENONE;
    uint16_t preframe_size = 0;
	uint16_t index = 0;
	uint8_t tmp = 0;
    uint16_t retrieve_size = 0;
    
    uint8_t frame_head[7] = {0};
    uint8_t sum_check_stringhex[2] = {0};
    uint16_t head_index = 0, precheck_index = 0, cr_index = 0, lf_index = 0;
    uint8_t frame_mask_check = 0x00;
    bool wrong_sequence = false;
    uint8_t sum_check = 0;
    int sum_check_handle = 0x00;
    uint8_t frame_len;

    if (_com->esize(ESAF::ebus_uart::STREAMIN) < 30)
        return 0x01;
    
	do {
		res = _com->query(index, &tmp, 1);
        switch (tmp) {
        case '$' :
            head_index = index;
            if (frame_mask_check == 0x00)
                frame_mask_check |= 0x01;
            else 
                wrong_sequence = true;
            break;
        case '*' :
            precheck_index = index;
            if (frame_mask_check == 0x01)
                frame_mask_check |= 0x02;
            else 
                wrong_sequence = true;
            break;
        case '\r' :
            cr_index = index;
            if (frame_mask_check == 0x03)
                frame_mask_check |= 0x04;
            else 
                wrong_sequence = true;
            break;
        case '\n' :
            lf_index = index;
            if (frame_mask_check == 0x07)
                frame_mask_check |= 0x08;
            else 
                wrong_sequence = true;
            break;
        }
        if (frame_mask_check == 0x0F)
            break;
        else
			index++;
	} while (res == ESAF::EDev::EOK);

	preframe_size = head_index;

	if (frame_mask_check != 0x0F || 
		((lf_index - cr_index) != 1) || 
		((cr_index - precheck_index) != 3)) {
        if (wrong_sequence) {
            devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
            /* wrong sequence */
            return 0x02;
		}else if (preframe_size != 0) {
            devbuf_retrieve(preframe_size);
		}
        devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
		/* no enough data */
		return 0x03;
	}else {
        
        frame_len = lf_index - head_index + 1;

        _com->query(precheck_index + 1, &sum_check_stringhex[0], 2);
        _com->query(head_index, &parse_window[0], frame_len);

        sum_check = ESAF::nmea0813_parser::stringhex_to_num(&sum_check_stringhex[0], 2);
        sum_check_handle = ESAF::nmea0813_parser::nmea_sum_check((const uint8_t *)&parse_window[0]);

        if (sum_check != sum_check_handle) {
        	devbuf_retrieve(preframe_size + frame_len);
            /* crc check failed */
            checkfault_cnt++;
            return 0x04;
        }
       	_com->query(head_index, &frame_head[0], 6);
        
        if (!strcmp((const char *)&frame_head[3], "RMC") || 
            !strcmp((const char *)&frame_head[3], "VTG") ||
            !strcmp((const char *)&frame_head[3], "GGA") ||
            !strcmp((const char *)&frame_head[3], "GSA") ||
            !strcmp((const char *)&frame_head[3], "GST")) {
                
            // for (int i = 0; i < frame_len; i++) {
            //     EDEBUG("%c", parse_window[i]);
            // }
            
            ESAF::nmea0813_parser::handleMessage(&parse_window[0], frame_len);
            
        } else {
            devbuf_retrieve(preframe_size + frame_len);
            /* no user wanted nmea frame */
            return 0x05;
        }
        devbuf_retrieve(preframe_size + frame_len);
	}

    #ifdef uOSB_PUBLISH_NMEA_GPS
        if (timestamp_start_block == 0) {
            timestamp_start_block = hrt_absolute_time();
        }
        _pub_sensor_gps.device_id = device_id;
        
        _pub_sensor_gps.latitude_deg = ESAF::nmea0813_parser::_gps_position.latitude_deg;
        _pub_sensor_gps.longitude_deg = ESAF::nmea0813_parser::_gps_position.longitude_deg;
        _pub_sensor_gps.altitude_msl_m = ESAF::nmea0813_parser::_gps_position.altitude_msl_m;
        _pub_sensor_gps.altitude_ellipsoid_m = ESAF::nmea0813_parser::_gps_position.altitude_ellipsoid_m;
        _pub_sensor_gps.lat = _pub_sensor_gps.latitude_deg * 1e+7;
        _pub_sensor_gps.lon = _pub_sensor_gps.longitude_deg * 1e+7;
        _pub_sensor_gps.alt = _pub_sensor_gps.altitude_msl_m;
        _pub_sensor_gps.alt_ellipsoid = _pub_sensor_gps.altitude_ellipsoid_m;
        _pub_sensor_gps.s_variance_m_s = ESAF::nmea0813_parser::_gps_position.s_variance_m_s;
        _pub_sensor_gps.c_variance_rad = ESAF::nmea0813_parser::_gps_position.c_variance_rad;
        _pub_sensor_gps.eph = ESAF::nmea0813_parser::_gps_position.eph;
        _pub_sensor_gps.epv = ESAF::nmea0813_parser::_gps_position.epv;
        _pub_sensor_gps.hdop = ESAF::nmea0813_parser::_gps_position.hdop;
        _pub_sensor_gps.vdop = ESAF::nmea0813_parser::_gps_position.vdop;
        _pub_sensor_gps.noise_per_ms = ESAF::nmea0813_parser::_gps_position.noise_per_ms;
        _pub_sensor_gps.jamming_indicator = ESAF::nmea0813_parser::_gps_position.jamming_indicator;
        _pub_sensor_gps.vel_m_s = ESAF::nmea0813_parser::_gps_position.vel_m_s;
        _pub_sensor_gps.vel_n_m_s = ESAF::nmea0813_parser::_gps_position.vel_n_m_s;
        _pub_sensor_gps.vel_e_m_s = ESAF::nmea0813_parser::_gps_position.vel_e_m_s;
        _pub_sensor_gps.vel_d_m_s = ESAF::nmea0813_parser::_gps_position.vel_d_m_s;
        _pub_sensor_gps.cog_rad = ESAF::nmea0813_parser::_gps_position.cog_rad;
        _pub_sensor_gps.timestamp_time_relative = ESAF::nmea0813_parser::_gps_position.timestamp_time_relative;
        _pub_sensor_gps.heading = ESAF::nmea0813_parser::_gps_position.heading;
        _pub_sensor_gps.heading_offset = ESAF::nmea0813_parser::_gps_position.heading_offset;
        _pub_sensor_gps.fix_type = ESAF::nmea0813_parser::_gps_position.fix_type;
        _pub_sensor_gps.jamming_state = ESAF::nmea0813_parser::_gps_position.jamming_state;
        _pub_sensor_gps.vel_ned_valid = ESAF::nmea0813_parser::_gps_position.vel_ned_valid;
        _pub_sensor_gps.satellites_used = ESAF::nmea0813_parser::_gps_position.satellites_used;
        
        _pub_sensor_gps.timestamp = hrt_absolute_time();
        _pub_sensor_gps.timestamp_time_relative = hrt_absolute_time() - timestamp_start_block;
        uorb_publish_topics<uORB::TOPIC_SENSOR_GPS>(&_pub_sensor_gps);
    #endif
        
    checkpass_cnt++;
    pass_rate = (float)checkpass_cnt / (float)(checkpass_cnt + checkfault_cnt);
    return 0x00;
}

void nmea0813::devbuf_retrieve(uint16_t len)
{
    while (len > 0) {
        if (len > 50) {
            _com->read(&retrieve[0], 50);
        }else if (len <= 50){
            _com->read(&retrieve[0], len);
            return;
        }
        len -= 50;
    }
}

/**
    sensor_gps_s _gps_position;
    _gps_position->timestamp = hrt_absolute_time();

    GPZDA
        _gps_position->time_utc_usec

    xxGGA/GPGGA/GNGGA

        _gps_position->longitude_deg
        _gps_position->latitude_deg
        _gps_position->hdop
        _gps_position->altitude_msl_m
        _gps_position->altitude_ellipsoid_m
        _gps_position->fix_type

    xxGNS/GPGNS/GNGNS

        _gps_position->longitude_deg
        _gps_position->latitude_deg
        _gps_position->hdop
        _gps_position->altitude_msl_m

    xxRMC/GPRMC/GNRMC
		_gps_position->latitude_deg
		_gps_position->longitude_deg
		_gps_position->vel_m_s
		_gps_position->vel_n_m_s
		_gps_position->vel_e_m_s
		_gps_position->cog_rad
		_gps_position->vel_ned_valid
		_gps_position->c_variance_rad
		_gps_position->s_variance_m_s
        _gps_position->time_utc_usec

    GPGST/GNGST
    
        _gps_position->eph
        _gps_position->epv
        
    GPGSA/GNGSA
    
        _gps_position->fix_type
        _gps_position->hdop
        _gps_position->vdop
    
    GNVTG
		_gps_position->vel_m_s
		_gps_position->vel_n_m_s
		_gps_position->vel_e_m_s
		_gps_position->cog_rad
		_gps_position->vel_ned_valid
		_gps_position->c_variance_rad
		_gps_position->s_variance_m_s
        _gps_position->satellites_used
        
        recommend: RMC + VTG + GSA + GGA + GST
*/
