#include "rtcm3.h"

void rtcm_st_pvt_decode(rtcm_st_pvt_t *f, uint8_t *p, uint16_t framelen)
{
	uint64_t data_mask_38bit = 0;
	for (uint8_t i = 0; i < (64 - 38); i++) {
		data_mask_38bit |= 1 << (63 - i);
	}
	f->msg_number = (uint16_t)(p[0] << 4) | ((p[1] & 0xF0) >> 4);
	f->subtype_id = ((p[1] & 0x0F) << 4) | ((p[2] & 0xF0) >> 4);
	f->refer_station_id = ((p[2] & 0x0F) << 8) | p[3];
	f->itrf_realization_year = (p[4] & 0xFC) >> 2;
	f->gps_quality = ((p[4] & 0x03) << 2) | ((p[5] & 0xC0) >> 6);
	f->satellites_use_num = ((p[5] & 0x3F) << 2) | ((p[6] & 0xC0) >> 6);
	f->satellites_view_num = ((p[6] & 0x3F) << 2) | ((p[7] & 0xC0) >> 6);
	f->hdop = ((p[7] & 0x3F) << 2) | ((p[8] & 0xC0) >> 6);
	f->vdop = ((p[8] & 0x3F) << 2) | ((p[9] & 0xC0) >> 6);
	f->pdop = ((p[9] & 0x3F) << 2) | ((p[10] & 0xC0) >> 6);
	f->geoidal_separation_meters = ((p[10] & 0x3F) << 9) |
			(p[11] << 1) | ((p[12] & 0x80) >> 7);
	f->age_of_differentials = ((p[12] & 0x7F) << 17) | (p[13] << 9) |
			(p[14] << 1) | ((p[15] & 0x80) >> 7);
	f->differential_refer_station_id = ((p[15] & 0x7F) << 5) | ((p[16] & 0xF8) >> 3);
	f->time_id = ((p[16] & 0x07) << 1) | ((p[17] & 0x80) >> 7);
	f->gnss_epoch_time = ((p[17] & 0x7F) << 23) |
			(p[18] << 15) | (p[19] << 7) | ((p[20] & 0xFE) >> 1);
	f->extend_week_num = ((p[20] & 0x01) << 15) | (p[21] << 7) | ((p[22] & 0xFE) >> 1);
	f->leap_sec_utc = ((p[22] & 0x01) << 7) | ((p[23] & 0xFE) >> 1);

//	f->antenna_position_ecef_x = ((int64_t)(p[23] & 0x01) << 37) | (p[24] << 29) |
//			(p[25] << 21) | (p[26] << 13) | (p[27] << 5) | ((p[28] & 0xF8) >> 3);
//	f->antenna_position_ecef_y = ((int64_t)(p[28] & 0x07) << 35) | (p[29] << 27) |
//			(p[30] << 19) | (p[31] << 11) | (p[32] << 3) | ((p[33] & 0xE0) >> 5);
//	f->antenna_position_ecef_z = ((int64_t)(p[33] & 0x1F) << 33) | (p[34] << 25) |
//			(p[35] << 17) | (p[36] << 9) | (p[37] << 1) | ((p[38] & 0x80) >> 7);

	f->antenna_position_ecef_x = getbitls(p, 191, 38);
	f->antenna_position_ecef_y = getbitls(p, 229, 38);
	f->antenna_position_ecef_z = getbitls(p, 267, 38);
	f->antenna_velocity_ecef_x = getbits(p, 305, 32);
	f->antenna_velocity_ecef_y = getbits(p, 337, 32);
	f->antenna_velocity_ecef_z = getbits(p, 369, 32);
//	f->antenna_velocity_ecef_x = ((p[38] & 0x7F) << 25) |
//			(p[39] << 17) | (p[40] << 9) | (p[41] << 1) | ((p[42] & 0x80) >> 7);
//	f->antenna_velocity_ecef_y = ((p[42] & 0x7F) << 25) |
//			(p[43] << 17) | (p[44] << 9) | (p[45] << 1) | ((p[46] & 0x80) >> 7);
//	f->antenna_velocity_ecef_z = ((p[46] & 0x7F) << 25) |
//			(p[47] << 17) | (p[48] << 9) | (p[49] << 1) | ((p[50] & 0x80) >> 7);

//	if (((f->antenna_position_ecef_x & ((uint64_t)0x01 << 37)) >> 37) == 0x01) {
//		f->antenna_position_ecef_x |= data_mask_38bit;
//	}
//
//	if (((f->antenna_position_ecef_y & ((uint64_t)0x01 << 37)) >> 37) == 0x01) {
//		f->antenna_position_ecef_y |= data_mask_38bit;
//	}
//
//	if (((f->antenna_position_ecef_z & ((uint64_t)0x01 << 37)) >> 37) == 0x01) {
//		f->antenna_position_ecef_z |= data_mask_38bit;
//	}
}

void rtcm_st_epvt_decode(rtcm_st_epvt_t *f, uint8_t *p, uint16_t framelen)
{
	f->msg_number = (uint16_t)(p[0] << 4) | ((p[1] & 0xF0) >> 4);
	f->subtype_id = ((p[1] & 0x0F) << 4) | ((p[2] & 0xF0) >> 4);
	f->refer_station_id = ((p[2] & 0x0F) << 8) | p[3];
	f->itrf_realization_year = (p[4] & 0xFC) >> 2;
	f->gps_quality = ((p[4] & 0x03) << 2) | ((p[5] & 0xC0) >> 6);
	f->data_status = (p[5] & 0x20) >> 5;
	f->fix_freq_mode = (p[5] & 0x10) >> 4;
	f->fix_integrity = (p[5] & 0x80) >> 3;
	f->rfu = (p[5] & 0x40) >> 2;
	f->satellites_num_use = ((p[5] & 0x03) << 6) | ((p[6] & 0xFC) >> 2);
	f->satellites_num_view = ((p[6] & 0x03) << 6) | ((p[7] & 0xFC) >> 2);
	f->hdop = ((p[7] & 0x03) << 6) | ((p[8] & 0xFC) >> 2);
	f->vdop = ((p[8] & 0x03) << 6) | ((p[9] & 0xFC) >> 2);
	f->pdop = ((p[9] & 0x03) << 6) | ((p[10] & 0xFC) >> 2);
	f->geoidal_separation_meters = ((p[10] & 0x03) << 13) |
			(p[11] << 5) | ((p[12] & 0xF8) >> 3);
	f->age_of_differentials = ((p[12] & 0x07) << 21) | (p[13] << 13) |
			(p[14] << 5) | ((p[15] & 0xF8) >> 3);
	f->differ_refer_station_id = ((p[15] & 0x07) << 9) |
			(p[16] << 1) | ((p[17] & 0x80) >> 7);
	f->time_id = (p[17] & 0x78) >> 3;
	f->time_validity =  ((p[17] & 0x07) << 1) | ((p[18] & 0x80) >> 7);
	f->gnss_epoch_time = ((p[18] & 0x7F) << 23) | (p[19] << 15) |
			(p[20] << 7) | ((p[21] & 0xFE) >> 1);
	f->extend_week_num = ((p[21] & 0x01) << 15) | (p[22] << 7) |
			((p[23] & 0xFE) >> 1);
	f->leap_sec_utc = ((p[23] & 0x01) << 7) | ((p[24] & 0xFE) >> 1);
	f->lat = getbitu(p, 199, 32);
	f->lon = getbitu(p, 231, 32);
	f->height = getbits(p, 263, 20);

	f->height = ((p[32] & 0x01) << 19) | (p[33] << 11) |
			(p[34] << 3) | ((p[35] & 0xE0) >> 5);
	f->vel_horizontal = ((p[35] & 0x1F) << 15) | (p[36] << 7) |
			((p[37] & 0xFE) >> 1);
	f->vel_vertical = ((p[37] & 0x01) << 19) | (p[38] << 11) |
			(p[39] << 3) | ((p[40] & 0xE0) >> 5);
	f->course_angle = ((p[40] & 0x1F) << 11) | (p[41] << 3) |
			((p[42] & 0xE0) >> 5);
	f->protection_level_horizontal = ((p[42] & 0x1F) << 11) | (p[43] << 3) |
			((p[44] & 0xE0) >> 5);
	f->protection_level_vertical = ((p[44] & 0x1F) << 11) | (p[45] << 3) |
			((p[46] & 0xE0) >> 5);
	f->protection_level_angle = ((p[46] & 0x1F) << 11) | (p[47] << 3) |
			((p[48] & 0xE0) >> 5);
	f->receiver_clock_bias = ((p[48] & 0x1F) << 27) | (p[49] << 19) |
			(p[50] << 11) | (p[51] << 3) | ((p[52] & 0xE0) >> 5);
	f->receiver_clock_drift = ((p[52] & 0x1F) << 27) | (p[53] << 19) |
			(p[54] << 11) | (p[55] << 3) | ((p[56] & 0xE0) >> 5);
}

void rtcm_st_sens_decode(rtcm_st_sens_t *f, uint8_t *p, uint16_t framelen)
{
	f->msg_number = (uint16_t)(p[0] << 4) | ((p[1] & 0xF0) >> 4);
	f->subtype_id = ((p[1] & 0x0F) << 4) | ((p[2] & 0xF0) >> 4);
	f->num_frame_entries = ((p[2] & 0x0F) << 4) | ((p[3] & 0xF0) >> 4);

	if (f->num_frame_entries != 4) return;

	f->sensor_type = ((p[3] & 0x0F) << 4) | ((p[4] & 0xF0) >> 4);
	f->cpu_timestamp = ((p[4] & 0x0F) << 28) | (p[5] << 20) | (p[6] << 12) | (p[7] << 4) | ((p[8] & 0xF0) >> 4);

	if (f->sensor_type == 0x1E) {
		f->accel_raw_x = ((p[8] & 0x0F) << 12) | (p[9] << 4) | ((p[10] & 0xF0) >> 4);
		f->accel_raw_y = ((p[10] & 0x0F) << 12) | (p[11] << 4) | ((p[12] & 0xF0) >> 4);
		f->accel_raw_z = ((p[12] & 0x0F) << 12) | (p[13] << 4) | ((p[14] & 0xF0) >> 4);
	}

	f->sensor_type = ((p[14] & 0x0F) << 4) | ((p[15] & 0xF0) >> 4);
	f->cpu_timestamp = ((p[15] & 0x0F) << 28) | (p[16] << 20) | (p[17] << 12) | (p[18] << 4) | ((p[19] & 0xF0) >> 4);
	if (f->sensor_type == 0x1F) {
		f->gyro_raw_x = ((p[19] & 0x0F) << 12) | (p[20] << 4) | ((p[21] & 0xF0) >> 4);
		f->gyro_raw_y = ((p[21] & 0x0F) << 12) | (p[22] << 4) | ((p[23] & 0xF0) >> 4);
		f->gyro_raw_z = ((p[23] & 0x0F) << 12) | (p[24] << 4) | ((p[25] & 0xF0) >> 4);
	}

	f->sensor_type = ((p[25] & 0x0F) << 4) | ((p[26] & 0xF0) >> 4);
	f->cpu_timestamp = ((p[26] & 0x0F) << 28) | (p[27] << 20) | (p[28] << 12) | (p[29] << 4) | ((p[30] & 0xF0) >> 4);
	if (f->sensor_type == 0x03) {
		f->odometer = ((p[30] & 0x0F) << 28) | (p[31] << 20) | (p[32] << 12) | (p[33] << 4) | ((p[34] & 0xF0) >> 4);
		f->odometer_reverse = (p[34] & 0x08) >> 3;
	}

	f->sensor_type = ((p[34] & 0x07) << 5) | ((p[35] & 0xF8) >> 3);
	f->cpu_timestamp = ((p[26] & 0x07) << 29) | (p[27] << 21) | (p[28] << 13) | (p[29] << 5) | ((p[39] & 0xF8) >> 2);
	if (f->sensor_type == 0x00) {
		f->sensor_type_len = ((p[39] & 0x07) << 3) | ((p[40] & 0xE0) >> 5);
		f->sensor_common_ver = (p[40] & 0x1F) >> 3;
		f->gps_epoch_time = ((p[40] & 0x07) << 27) | (p[41] << 19) | (p[42] << 11) | (p[43] << 3) | ((p[44] & 0xE0) >> 5);
	}
}

void rtcm_st_utc_pva_decode(rtcm_st_utc_pva *f, uint8_t *p, uint16_t framelen)
{
	uint64_t data_mask_24bit = 0;
	uint64_t data_mask_18bit = 0;
	for (uint8_t i = 0; i <= (32 - 24); i++) {
		data_mask_24bit |= 1 << (31 - i);
	}
	for (uint8_t i = 0; i <= (32 - 18); i++) {
		data_mask_18bit |= 1 << (31 - i);
	}

	f->msg_number = (uint16_t)(p[0] << 4) | ((p[1] & 0xF0) >> 4);
	f->subtype_id = ((p[1] & 0x0F) << 4) | ((p[2] & 0xF0) >> 4);
	f->gps_epoch_time = ((p[2] & 0x0F) << 26) | (p[3] << 18) | (p[4] << 10) | (p[5] << 2) | ((p[6] & 0xC0) >> 6);
	f->fix_quality_indicator = (p[6] & 0x3C) >> 2;
	f->rcv_ecef_position_x = ((p[6] & 0x03) << 30) | (p[7] << 22) | (p[8] << 14)
			| (p[9] << 6) | ((p[10] & 0xFC) >> 2);
	f->rcv_ecef_position_y = ((p[10] & 0x03) << 30) | (p[11] << 22) | (p[12] << 14)
					| (p[13] << 6) | ((p[14] & 0xFC) >> 2);
	f->rcv_ecef_position_z = ((p[14] & 0x03) << 30) | (p[15] << 22) | (p[16] << 14)
							| (p[17] << 6) | ((p[18] & 0xFC) >> 2);
	f->rcv_ecef_velocity_x = ((p[18] & 0x03) << 22) | (p[19] << 14)
									| (p[20] << 6) | ((p[21] & 0xFC) >> 2);
	f->rcv_ecef_velocity_y = ((p[21] & 0x03) << 22) | (p[22] << 14)
									| (p[23] << 6) | ((p[24] & 0xFC) >> 2);
	f->rcv_ecef_velocity_z = ((p[24] & 0x03) << 22) | (p[25] << 14)
									| (p[26] << 6) | ((p[27] & 0xFC) >> 2);
	f->rcv_ecef_acceleration_x = (p[27] & 0x03) << 16| (p[28] << 8) | p[29];
	f->rcv_ecef_acceleration_y = (p[30] << 10) | (p[31] << 2) | ((p[32] & 0xC0) >> 6);
	f->rcv_ecef_acceleration_z = ((p[32] & 0x3F) << 12) | (p[33] << 4) | ((p[34] & 0xF0) >> 4);
	f->rcv_pva_accuracy_indexes = ((p[34] & 0x0F) << 4) | ((p[35] & 0xF0) >> 4);
	f->rcv_clock_bias = ((p[35] & 0x0F) << 28) | (p[36] << 20) | (p[37] << 12) | (p[38] << 4) | ((p[39] & 0xF0) >> 4);
	f->rcv_clock_drift = ((p[39] & 0x0F) << 28) | (p[40] << 20) | (p[41] << 12) | (p[42] << 4) | ((p[43] & 0xF0) >> 4);

	if (((f->rcv_ecef_velocity_x & (0x01 << 23)) >> 23) == 0x01) {
		f->rcv_ecef_velocity_x |= data_mask_24bit;
	}
	if (((f->rcv_ecef_velocity_y & (0x01 << 23)) >> 23) == 0x01) {
		f->rcv_ecef_velocity_y |= data_mask_24bit;
	}
	if (((f->rcv_ecef_velocity_z & (0x01 << 23)) >> 23) == 0x01) {
		f->rcv_ecef_velocity_z |= data_mask_24bit;
	}
	if (((f->rcv_ecef_acceleration_x & (0x01 << 17)) >> 17) == 0x01) {
		f->rcv_ecef_acceleration_x |= data_mask_18bit;
	}
	if (((f->rcv_ecef_acceleration_y & (0x01 << 17)) >> 17) == 0x01) {
		f->rcv_ecef_acceleration_y |= data_mask_18bit;
	}
	if (((f->rcv_ecef_acceleration_z & (0x01 << 17)) >> 17) == 0x01) {
		f->rcv_ecef_acceleration_z |= data_mask_18bit;
	}
}

void rtcm_st_utc_pva_packsend(rtcm_st_utc_pva *f, uint8_t *p, uint16_t *len)
{
	uint16_t msg_len = 44; //4 bits: 348
	uint16_t msg_start_idx = 3;
	uint32_t msg_crc = 0;

	p[0] = 0xD3; //RTCM3_PREAMBLE;
	p[1] = 0x00; //0x00 | ((msg_len & 0x0300) >> 8);
	p[2] = 0x2C; //msg_len & 0x00FF;
	p[0 + msg_start_idx] = (f->msg_number & 0x0FF0) >> 4;
	p[1 + msg_start_idx] = ((f->msg_number & 0x000F) << 4) | ((f->subtype_id & 0xF0) >> 4);
	p[2 + msg_start_idx] = ((f->subtype_id & 0x0F) << 4) | ((f->gps_epoch_time & 0x3C000000) >> 26);
	p[3 + msg_start_idx] = (f->gps_epoch_time & 0x03FC0000) >> 18;
	p[4 + msg_start_idx] = (f->gps_epoch_time & 0x0003FC00) >> 10;
	p[5 + msg_start_idx] = (f->gps_epoch_time & 0x000003FC) >> 2;
	p[6 + msg_start_idx] = ((f->gps_epoch_time & 0x00000003) << 6) | ((f->fix_quality_indicator & 0x0F) << 2) |
			((f->rcv_ecef_position_x & 0xC0000000) >> 30);
	p[7 + msg_start_idx] = (f->rcv_ecef_position_x & 0x3FC00000) >> 22;
	p[8 + msg_start_idx] = (f->rcv_ecef_position_x & 0x003FC000) >> 14;
	p[9 + msg_start_idx] = (f->rcv_ecef_position_x & 0x00003FC0) >> 6;

	p[10 + msg_start_idx] = ((f->rcv_ecef_position_x & 0x0000003F) << 2) | ((f->rcv_ecef_position_y & 0xC0000000) >> 30);
	p[11 + msg_start_idx] = (f->rcv_ecef_position_y & 0x3FC00000) >> 22;
	p[12 + msg_start_idx] = (f->rcv_ecef_position_y & 0x003FC000) >> 14;
	p[13 + msg_start_idx] = (f->rcv_ecef_position_y & 0x00003FC0) >> 6;

	p[14 + msg_start_idx] = ((f->rcv_ecef_position_y & 0x0000003F) << 2) | ((f->rcv_ecef_position_z & 0xC0000000) >> 30);
	p[15 + msg_start_idx] = (f->rcv_ecef_position_z & 0x3FC00000) >> 22;
	p[16 + msg_start_idx] = (f->rcv_ecef_position_z & 0x003FC000) >> 14;
	p[17 + msg_start_idx] = (f->rcv_ecef_position_z & 0x00003FC0) >> 6;

	p[18 + msg_start_idx] = ((f->rcv_ecef_position_z & 0x0000003F) << 2) | ((f->rcv_ecef_velocity_x & 0x00C00000) >> 22);
	p[19 + msg_start_idx] = (f->rcv_ecef_velocity_x & 0x003FC000) >> 14;
	p[20 + msg_start_idx] = (f->rcv_ecef_velocity_x & 0x00003FC0) >> 6;

	p[21 + msg_start_idx] = ((f->rcv_ecef_velocity_x & 0x0000003F) << 2) | ((f->rcv_ecef_velocity_y & 0x00C00000) >> 22);
	p[22 + msg_start_idx] = (f->rcv_ecef_velocity_y & 0x003FC000) >> 14;
	p[23 + msg_start_idx] = (f->rcv_ecef_velocity_y & 0x00003FC0) >> 6;

	p[24 + msg_start_idx] = ((f->rcv_ecef_velocity_y & 0x0000003F) << 2) | ((f->rcv_ecef_velocity_z & 0x00C00000) >> 22);
	p[25 + msg_start_idx] = (f->rcv_ecef_velocity_z & 0x003FC000) >> 14;
	p[26 + msg_start_idx] = (f->rcv_ecef_velocity_z & 0x00003FC0) >> 6;

	p[27 + msg_start_idx] = ((f->rcv_ecef_velocity_z & 0x0000003F) << 2) | ((f->rcv_ecef_acceleration_x & 0x00030000) >> 16);
	p[28 + msg_start_idx] = (f->rcv_ecef_acceleration_x & 0x0000FF00) >> 8;
	p[29 + msg_start_idx] = f->rcv_ecef_acceleration_x & 0x000000FF;

	p[30 + msg_start_idx] = (f->rcv_ecef_acceleration_y & 0x0003FC00) >> 10;
	p[31 + msg_start_idx] = (f->rcv_ecef_acceleration_y & 0x000003FC) >> 2;
	p[32 + msg_start_idx] = ((f->rcv_ecef_acceleration_y & 0x00000003) << 6) | ((f->rcv_ecef_acceleration_z & 0x0003F000) >> 12);
	p[33 + msg_start_idx] = (f->rcv_ecef_acceleration_z & 0x00000FF0) >> 4;
	p[34 + msg_start_idx] = ((f->rcv_ecef_acceleration_z & 0x0000000F) << 4) | ((f->rcv_pva_accuracy_indexes & 0xF0) >> 4);
	p[35 + msg_start_idx] = ((f->rcv_pva_accuracy_indexes & 0x0F) << 4) | ((f->rcv_clock_bias & 0xF00000000) >> 28);
	p[36 + msg_start_idx] = (f->rcv_clock_bias & 0x0FF00000) >> 20;
	p[37 + msg_start_idx] = (f->rcv_clock_bias & 0x000FF000) >> 12;
	p[38 + msg_start_idx] = (f->rcv_clock_bias & 0x00000FF0) >> 4;
	p[39 + msg_start_idx] = ((f->rcv_clock_bias & 0x0000000F) << 4) | ((f->rcv_clock_drift & 0xF00000000) >> 28);
	p[40 + msg_start_idx] = (f->rcv_clock_drift & 0x0FF00000) >> 20;
	p[41 + msg_start_idx] = (f->rcv_clock_drift & 0x000FF000) >> 12;
	p[42 + msg_start_idx] = (f->rcv_clock_drift & 0x00000FF0) >> 4;
	p[43 + msg_start_idx] = ((f->rcv_clock_drift & 0x0000000F) << 4) & 0xF0;

	msg_crc = rtk_crc24q(&p[0], p[2] + 3);
	//msg_crc = rtcm_crc_calc(&p[0], msg_len + 3);
	p[44 + msg_start_idx] = (msg_crc & 0xFF0000) >> 16;
	p[45 + msg_start_idx] = (msg_crc & 0xFF00) >> 8;
	p[46 + msg_start_idx] = (msg_crc & 0xFF);

	*len = 6 + msg_len;
}



