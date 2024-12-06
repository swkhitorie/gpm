#include "lsm6dsr.h"

int lsm6dsr_init(lsm6dsr_t *obj)
{
	int res = 0;
	uint8_t r_value[5] = {0};
	uint8_t w_value[5] = {0x44, 0x6E, 0x6C, (0<<7), (0<<4)};
	uint8_t i;
	uint8_t id;
	
	uint8_t offset[3];
	obj->platform_write(0x73, &offset[0], 1);
	obj->platform_write(0x74, &offset[1], 1);
	obj->platform_write(0x75, &offset[2], 1);

	res = obj->platform_write(0x12, &w_value[0], 1); //0x12H enable BDU， enable auto increse
	res = obj->platform_write(0x10, &w_value[1], 1); //0x10H 416Hz, ±8g, lpf enable
	res = obj->platform_write(0x11, &w_value[2], 1); //0x11H 416Hz, 2000dps
	res = obj->platform_write(0x16, &w_value[3], 1); //0x16H enable G_HM_MODE
	res = obj->platform_write(0x15, &w_value[4], 1); //0x15H enable XL_HM_MODE

	res = obj->platform_read(0x12, &r_value[0], 1);
	res = obj->platform_read(0x10, &r_value[1], 1);
	res = obj->platform_read(0x11, &r_value[2], 1);
	res = obj->platform_read(0x16, &r_value[3], 1);
	res = obj->platform_read(0x15, &r_value[4], 1);

	if (res != 0) {
		return res;
	}

	for (i = 0; i < 5; i++) {
		if (r_value[i] != w_value[i]) {
			break;
		}
	}

	if (i != 5) {
		return -(2 + i);
	}

	obj->accel_scale = (float)((2 * 8) / 65536.0);
	obj->gyro_scale = (float)((2 * 2000.0) / 65536.0);

	obj->platform_read(0x0f, &id, 1);

	if (id != 0x6A && id != 0x6B) {
		return -1;
	}

	obj->chipid = id;

	return 0;
}


int lsm6dsr_read(lsm6dsr_t *obj, float *acc, float *gyro, float *temper)
{
	int res = 0;
	uint8_t dbuf[12];
	uint8_t tbuf[2];
	int16_t tmper;
	int16_t gy[3], ac[3];
	uint8_t addr_temp = 0x20;
	uint8_t data_addr = 0x22; //OUTX_L_G

	res = obj->platform_read(addr_temp, &tbuf[0], 2);

	res = obj->platform_read(data_addr, &dbuf[0], 12);

	tmper = (tbuf[0] | tbuf[1] << 8);
	gy[0] = (dbuf[0] | dbuf[1] << 8);
	gy[1] = (dbuf[2] | dbuf[3] << 8);
	gy[2] = (dbuf[4] | dbuf[5] << 8);
	ac[0] = (dbuf[6] | dbuf[7] << 8);
	ac[1] = (dbuf[8] | dbuf[9] << 8);
	ac[2] = (dbuf[10] | dbuf[11] << 8);

	if ( tmper != 0x00 ) {
		*temper = (float)tmper / 256.0f + 25.0f;
	}

	if ( !(ac[0] == 0x00 && ac[1] == 0x00 && ac[2] == 0x00) ) {
		acc[0] = ac[0] * obj->accel_scale * 9.79135f;
		acc[1] = ac[1] * obj->accel_scale * 9.79135f;
		acc[2] = ac[2] * obj->accel_scale * 9.79135f;
	}

	if ( !(gy[0] == 0x00 && gy[1] == 0x00 && gy[2] == 0x00) ) {
		gyro[0] = gy[0] * obj->gyro_scale;
		gyro[1] = gy[1] * obj->gyro_scale;
		gyro[2] = gy[2] * obj->gyro_scale;
	}

	return res;
}
