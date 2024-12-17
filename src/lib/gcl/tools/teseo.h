#ifndef TESEO_H_
#define TESEO_H_

#include <stdint.h>
#include "rtkcmn.h"

typedef struct __st_sensor {
	uint16_t msg_number;
	uint8_t subtype_id;
	uint8_t num_frame_entries;
	uint8_t sensor_type;
	uint32_t cpu_timestamp;
	uint8_t sensor_type_len;
	uint8_t sensor_common_ver;
	uint32_t gps_epoch_time;

	uint32_t odometer;
	uint8_t odometer_reverse;

	int16_t accel_raw_x;
	int16_t accel_raw_y;
	int16_t accel_raw_z;

	int16_t gyro_raw_x;
	int16_t gyro_raw_y;
	int16_t gyro_raw_z;

} rtcm_st_sens_t;

typedef struct __st_pvt {
	uint16_t msg_number;
	uint8_t subtype_id;
	uint16_t refer_station_id;
	uint8_t itrf_realization_year;
	uint8_t gps_quality;
	uint8_t satellites_use_num;
	uint8_t satellites_view_num;
	uint8_t hdop;
	uint8_t vdop;
	uint8_t pdop;
	uint16_t geoidal_separation_meters;
	uint32_t age_of_differentials;
	uint16_t differential_refer_station_id;
	uint8_t time_id;
	uint32_t gnss_epoch_time;
	uint16_t extend_week_num;
	uint8_t leap_sec_utc;
	int64_t antenna_position_ecef_x;
	int64_t antenna_position_ecef_y;
	int64_t antenna_position_ecef_z;
	int32_t antenna_velocity_ecef_x;
	int32_t antenna_velocity_ecef_y;
	int32_t antenna_velocity_ecef_z;
} rtcm_st_pvt_t;

typedef struct __st_epvt {
	uint16_t msg_number;
	uint8_t subtype_id;
	uint16_t refer_station_id;
	uint8_t itrf_realization_year;
	uint8_t gps_quality;
	uint8_t data_status;
	uint8_t fix_freq_mode;
	uint8_t fix_integrity;
	uint8_t rfu;
	uint8_t satellites_num_use;
	uint8_t satellites_num_view;
	uint8_t hdop;
	uint8_t vdop;
	uint8_t pdop;
	uint16_t geoidal_separation_meters;
	uint32_t age_of_differentials;
	uint16_t differ_refer_station_id;
	uint8_t time_id;
	uint8_t time_validity;
	uint32_t gnss_epoch_time;
	uint16_t extend_week_num;
	uint8_t leap_sec_utc;
	uint32_t lat;
	uint32_t lon;
	int32_t height;
	uint32_t vel_horizontal;
	uint32_t vel_vertical;
	uint16_t course_angle;
	uint16_t protection_level_horizontal;
	uint16_t protection_level_vertical;
	uint16_t protection_level_angle;
	uint32_t receiver_clock_bias;
	uint32_t receiver_clock_drift;
} rtcm_st_epvt_t;

typedef struct __st_utc_pva {
	uint16_t msg_number;
	uint8_t subtype_id;
	uint32_t gps_epoch_time;
	uint8_t fix_quality_indicator;
	int32_t rcv_ecef_position_x;
	int32_t rcv_ecef_position_y;
	int32_t rcv_ecef_position_z;
	int32_t rcv_ecef_velocity_x;
	int32_t rcv_ecef_velocity_y;
	int32_t rcv_ecef_velocity_z;
	int32_t rcv_ecef_acceleration_x;
	int32_t rcv_ecef_acceleration_y;
	int32_t rcv_ecef_acceleration_z;
	uint8_t rcv_pva_accuracy_indexes;
	uint32_t rcv_clock_bias;
	uint32_t rcv_clock_drift;
} rtcm_st_utc_pva;

void rtcm_st_pvt_decode(rtcm_st_pvt_t *f, uint8_t *p, uint16_t framelen);
void rtcm_st_epvt_decode(rtcm_st_epvt_t *f, uint8_t *p, uint16_t framelen);
void rtcm_st_sens_decode(rtcm_st_sens_t *f, uint8_t *p, uint16_t framelen);

void rtcm_st_utc_pva_decode(rtcm_st_utc_pva *f, uint8_t *p, uint16_t framelen);
void rtcm_st_utc_pva_packsend(rtcm_st_utc_pva *f, uint8_t *p, uint16_t *len);

#endif
