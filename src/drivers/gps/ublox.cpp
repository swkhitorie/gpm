#include "ublox.h"

ublox::ublox(uint32_t _id, ESAF::ebus_uart *com) : device_id(_id), _com(com) 
{
    timestamp_start_block = 0;
}

void ublox::devbuf_retrieve(uint16_t len)
{
    while(len > 0) {
        if (len > 50) {
            _com->read(&_retrieve[0], 50);
        }else if (len <= 50){
            _com->read(&_retrieve[0], len);
            return;
        }
        len -= 50;
    }
}

uint8_t ublox::update()
{
	int res = ESAF::EDev::ENONE;
    uint16_t preframe_size = 0;
    uint16_t start_index = 0;
    uint16_t check_index = 0;
	uint16_t index = 0;
	uint8_t tmp = 0;
    
    uint8_t sync_char1, sync_char2;
    uint8_t class_id, msg_id;
    uint16_t payload_len;
    uint8_t ck_a, ck_a_cal, ck_b, ck_b_cal;
    
    if (_com->esize(ESAF::ebus_uart::STREAMIN) < UBLOX_PREPAYLOAD_LEN)
        return 0x01;
    
	do {
		res = _com->query(index, &tmp, 1);
		if (tmp == UBLOX_SYNC_CHAR1)
			break;
        else 
            index++;
	} while (res == ESAF::EDev::EOK);
	
	preframe_size = index;
    start_index = index;
	index++;
	if (res == ESAF::EDev::ENONE) {
        devbuf_retrieve(preframe_size);
		return 0x02;
	}

    _com->query(index++, &sync_char2, 1);
	if (sync_char2 != UBLOX_SYNC_CHAR2) {
        devbuf_retrieve(preframe_size + UBLOX_PREPAYLOAD_LEN);
		return 0x03;
	}
    
	if (_com->esize(ESAF::ebus_uart::STREAMIN) - preframe_size < UBLOX_PREPAYLOAD_LEN + ubloxframe::NAV_LEN_MIN) {
        devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
		return 0x04;
	}
    
    check_index = index;
    _com->query(index++, &class_id, 1);
    _com->query(index++, &msg_id, 1);
    _com->query(index, &payload_len, 2);
    index += 2;
    
    switch (class_id) {
    case UBLOX_CLASS_ID_NAV:
        if (msg_id == ubloxframe::NAV_ID_PVT && payload_len == ubloxframe::NAV_LEN_PVT) {
            _com->query(index + ubloxframe::NAV_LEN_PVT, &ck_a, 1);
            _com->query(index + ubloxframe::NAV_LEN_PVT + 1, &ck_b, 1);  
            
            _com->query(check_index, &_check_data[0], ubloxframe::NAV_LEN_PVT + 4);
            ubloxframe::calchecksum(&_check_data[0], ubloxframe::NAV_LEN_PVT + 4, ck_a_cal, ck_b_cal);

            if (ck_a != ck_a_cal || ck_b != ck_b_cal) {
                devbuf_retrieve(preframe_size + ubloxframe::NAV_LEN_PVT + UBLOX_NONPAYLOAD_LEN);
                return 0x05;                
            } else {
                _com->query(index, reinterpret_cast<uint8_t *>(&_msg_pvt), ubloxframe::NAV_LEN_PVT);   
                devbuf_retrieve(preframe_size + ubloxframe::NAV_LEN_PVT + UBLOX_NONPAYLOAD_LEN);
            }
        } else if (msg_id == ubloxframe::NAV_ID_DOP && payload_len == ubloxframe::NAV_LEN_DOP) {
            _com->query(index + ubloxframe::NAV_LEN_DOP, &ck_a, 1);
            _com->query(index + ubloxframe::NAV_LEN_DOP + 1, &ck_b, 1);  
            
            _com->query(check_index, &_check_data[0], ubloxframe::NAV_LEN_DOP + 4);
            ubloxframe::calchecksum(&_check_data[0], ubloxframe::NAV_LEN_DOP + 4, ck_a_cal, ck_b_cal);

            if (ck_a != ck_a_cal || ck_b != ck_b_cal) {
                devbuf_retrieve(preframe_size + ubloxframe::NAV_LEN_DOP + UBLOX_NONPAYLOAD_LEN);
                return 0x06;                
            } else {
                _com->query(index, reinterpret_cast<uint8_t *>(&_msg_dop), ubloxframe::NAV_LEN_DOP);
                devbuf_retrieve(preframe_size + ubloxframe::NAV_LEN_DOP + UBLOX_NONPAYLOAD_LEN);
            }
        }
        break;       
    default : return 0x07;
    }

    #ifdef uOSB_PUBLISH_UBLOX_GPS
        if (timestamp_start_block == 0) {
            timestamp_start_block = hrt_absolute_time();
        }
        _pub_sensor_gps.device_id = device_id;
        _pub_sensor_gps.time_utc_usec = _msg_pvt.iTOW * 1000;
        
        _pub_sensor_gps.fix_type = _msg_pvt.fixType;
        if (_msg_pvt.flags & 0x02) {
            _pub_sensor_gps.fix_type = 4;
        }
        uint8_t carr_soln = _msg_pvt.flags >> 6;
        if (carr_soln == 1) {
            _pub_sensor_gps.fix_type = 5; //Float RTK

        } else if (carr_soln == 2) {
            _pub_sensor_gps.fix_type = 6; //Fixed RTK
        }
        _pub_sensor_gps.vel_ned_valid = true;
        _pub_sensor_gps.satellites_used = _msg_pvt.numSV;
        
        {
            if (_pub_sensor_gps.fix_type < 6) {
                // When RTK is active and solid (fix=6), these values will be filled by HPPOSLLH
                _pub_sensor_gps.latitude_deg = _msg_pvt.lat * 1e-7;
                _pub_sensor_gps.longitude_deg = _msg_pvt.lon * 1e-7;
                _pub_sensor_gps.altitude_msl_m = _msg_pvt.hMSL * 1e-3;
                _pub_sensor_gps.altitude_ellipsoid_m = _msg_pvt.height * 1e-3;
                
                _pub_sensor_gps.lat = _msg_pvt.lat;
                _pub_sensor_gps.lon = _msg_pvt.lon;
                _pub_sensor_gps.alt = _msg_pvt.hMSL;
                _pub_sensor_gps.alt_ellipsoid = _msg_pvt.height;
                
                _pub_sensor_gps.eph = static_cast<float>(_msg_pvt.hAcc) * 1e-3f;
                _pub_sensor_gps.epv =static_cast<float>(_msg_pvt.vAcc) * 1e-3f;
            }
            
            _pub_sensor_gps.s_variance_m_s = static_cast<float>(_msg_pvt.sAcc) * 1e-3f;
            
            _pub_sensor_gps.vel_m_s = static_cast<float>(_msg_pvt.gSpeed) * 1e-3f;
            
            _pub_sensor_gps.vel_n_m_s = static_cast<float>(_msg_pvt.velN) * 1e-3f;
            _pub_sensor_gps.vel_e_m_s = static_cast<float>(_msg_pvt.velE) * 1e-3f;
            _pub_sensor_gps.vel_d_m_s = static_cast<float>(_msg_pvt.velD) * 1e-3f;
            
            _pub_sensor_gps.cog_rad = static_cast<float>(_msg_pvt.headMot) * M_DEG_TO_RAD_F * 1e-5f;
            _pub_sensor_gps.c_variance_rad = static_cast<float>(_msg_pvt.headAcc) * M_DEG_TO_RAD_F * 1e-5f;
        }
        
        _pub_sensor_gps.hdop = _msg_dop.hDOP * 0.01f;
        _pub_sensor_gps.vdop = _msg_dop.vDOP * 0.01f;
        
        _pub_sensor_gps.heading = 0;
        _pub_sensor_gps.heading_offset = 0;
        _pub_sensor_gps.heading_accuracy = 0;
        _pub_sensor_gps.automatic_gain_control = 0;
        _pub_sensor_gps.noise_per_ms = 0;
        _pub_sensor_gps.jamming_indicator = 0;
        _pub_sensor_gps.jamming_state = 0;
        
        _pub_sensor_gps.timestamp = hrt_absolute_time();
        _pub_sensor_gps.timestamp_time_relative = hrt_absolute_time() - timestamp_start_block;
        uorb_publish_topics<uORB::TOPIC_SENSOR_GPS>(&_pub_sensor_gps);
    #endif

    return 0x00;
}
