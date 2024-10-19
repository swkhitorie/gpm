#include "mti3.h"
#include "log.hpp"

Mti3::Mti3(ESAF::ebus_uart *com) : 
	raw_angle(Vector3f(0.0f, 0.0f, 0.0f)),
	raw_accel(Vector3f(0.0f, 0.0f, 0.0f)),
	raw_gyro(Vector3f(0.0f, 0.0f, 0.0f)),
	raw_free_accel(Vector3f(0.0f, 0.0f, 0.0f)),
	raw_hr_accel(Vector3f(0.0f, 0.0f, 0.0f)),
	raw_hr_gyro(Vector3f(0.0f, 0.0f, 0.0f)),
	min_packlen(0),
	updated(false),
	online(false),
	lostcnt(1),
	_com(com)
{
	if (DATALEN_100HZ_PACK == 0)
		min_packlen = sizeof(raw_highspeed_pack);
	if (DATALEN_1000HZ_PACK == 0)
		min_packlen = sizeof(raw_lowspeed_pack);
	if (min_packlen <= sizeof(raw_highspeed_pack))
		min_packlen = sizeof(raw_lowspeed_pack);
	else
		min_packlen = sizeof(raw_highspeed_pack);
}
Mti3::~Mti3() {}

uint8_t Mti3::device_update()
{
	int res = ESAF::EDev::ENONE;
    uint16_t preframe_size = 0;
	uint16_t index = 0;
	uint16_t start_index = 0;
	uint8_t tmp = 0;
    uint16_t retrieve_size = 0;

    uint8_t bid = 0;
    uint8_t mid = 0;
    uint8_t len = 0;
    uint16_t packid = 0;
    uint8_t *p = nullptr;
    uint8_t tmpcheck = 0;

	lostcnt++;
	if (lostcnt > 20) {
		lostcnt = 30;
		online = false;
	}
	updated = false;
	
    if (_com->esize(ESAF::ebus_uart::STREAMIN) < min_packlen)
        return 0x01;

	do {
		res = _com->query(index, &tmp, 1);
		if (tmp == MTI3_PACK_HEAD)
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

	if (_com->esize(ESAF::ebus_uart::STREAMIN) - preframe_size < min_packlen) {
        devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
		return 0x03;
	}

	_com->query(index++, &bid, 1);
	_com->query(index++, &mid, 1);
	_com->query(index++, &len, 1);
	
	if (bid != MTI3_PACK_BID || mid != MTI3_PACK_MID) {
        devbuf_retrieve(preframe_size + min_packlen);
		return 0x04;
	}
	
	if ((len != (sizeof(raw_highspeed_pack) - 5)) && 
		(len != (sizeof(raw_lowspeed_pack) - 5))) {
		devbuf_retrieve(preframe_size + min_packlen);
		return 0x05;
	}
	
    _com->query(index, &packid, 4);
    index += 4;
    packid = ESAF::Serialize::bytes_to_u16((uint8_t *)&packid, true);

    switch (packid) {
    case MTI3_ORIENTATION_EULER_ANGLE_ID:
    case MTI3_INERTIAL_ACCELERATION_ID:
    case MTI3_INERTIAL_RATEOFTURN_ID:
    case MTI3_INERTIAL_FREE_ACCELERATION_ID:
    	{
    		_com->query(start_index, (uint8_t *)&raw_lowspeed_pack, sizeof(raw_lowspeed_pack));
    		p = (uint8_t *)&raw_lowspeed_pack;
			for (uint8_t i = 1; i < sizeof(raw_lowspeed_pack); i++) {
				tmpcheck += p[i];
			}
			#if	MTI3_ORIENTATION_EULER_ANGLE
				raw_angle.x = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_euler_1, true);
				raw_angle.y = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_euler_2, true);
				raw_angle.z = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_euler_3, true);
			#endif
			#if	MTI3_INERTIAL_ACCELERATION
				raw_accel.x = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_accel_1, true);
				raw_accel.y = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_accel_2, true);
				raw_accel.z = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_accel_3, true);
			#endif
			#if	MTI3_INERTIAL_RATEOFTURN
				raw_gyro.x = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_rate_turn_1, true) * 57.3f;
				raw_gyro.y = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_rate_turn_2, true) * 57.3f;
				raw_gyro.z = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_rate_turn_3, true) * 57.3f;
			#endif
			#if	MTI3_INERTIAL_FREE_ACCELERATION
				raw_free_accel.x = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_free_accel_1, true);
				raw_free_accel.y = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_free_accel_2, true);
				raw_free_accel.z = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_lowspeed_pack.data_free_accel_3, true);		
			#endif
    		break;
    	}
    case MTI3_HIGHRATE_ACCELERATION_ID:
    case MTI3_HIGHRATE_RATEOFTURN_ID:
    	{
    		_com->query(start_index, (uint8_t *)&raw_highspeed_pack, sizeof(raw_highspeed_pack));
       		p = (uint8_t *)&raw_highspeed_pack;
			for (uint8_t i = 1; i < sizeof(raw_highspeed_pack); i++)
				tmpcheck += p[i];
    		break;
			#ifdef MTI3_HIGHRATE_ENABLE
				#if	MTI3_INERTIAL_RATEOFTURN
					raw_hr_accel_x = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_highspeed_pack.data_hr_accel_1, true);
					raw_hr_accel_y = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_highspeed_pack.data_hr_accel_2, true);
					raw_hr_accel_z = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_highspeed_pack.data_hr_accel_3, true);
				#endif
				#if	MTI3_INERTIAL_FREE_ACCELERATION
					raw_free_accel_x = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_highspeed_pack.data_hr_rate_turn_1, true) * 57.3f;
					raw_free_accel_y = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_highspeed_pack.data_hr_rate_turn_2, true) * 57.3f;
					raw_free_accel_z = ESAF::Serialize::bytes_to_float((uint8_t *)&raw_highspeed_pack.data_hr_rate_turn_3, true) * 57.3f;		
				#endif			
			#endif
    	}
    default :  { devbuf_retrieve(preframe_size + len + 5); } return 0x06;
    }

	updated = true;
	online = true;
	lostcnt = 1;
	devbuf_retrieve(preframe_size + len + 5);

	if (_com->esize(ESAF::ebus_uart::STREAMIN) >= min_packlen)
		device_update();
	return 0x00;
}

void Mti3::copy_device_data()
{
	angle(raw_angle.x, raw_angle.y, raw_angle.z);
	angle_vel(raw_gyro.x, raw_gyro.y, raw_gyro.z);
	accel(raw_accel.x, raw_accel.y, raw_accel.z);
	free_accel(raw_free_accel.x, raw_free_accel.y, raw_free_accel.z);
}

void Mti3::devbuf_retrieve(uint16_t len)
{
    while(1) {
        if (len > 50) {
            _com->read(&retrieve[0], 50);
        }else if (len <= 50){
            _com->read(&retrieve[0], len);
            return;
        }
        len -= 50;
    }
}

bool Mti3::is_online()
{
	return online;
}

