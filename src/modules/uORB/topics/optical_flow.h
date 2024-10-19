#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define OPTICAL_FLOW_MODE_UNKNOWN 0
#define OPTICAL_FLOW_MODE_BRIGHT 1
#define OPTICAL_FLOW_MODE_LOWLIGHT 2
#define OPTICAL_FLOW_MODE_SUPER_LOWLIGHT 3

#endif


#ifdef __cplusplus
struct __EXPORT optical_flow_s {
#else
struct optical_flow_s {
#endif
	uint64_t timestamp;
	float pixel_flow_x_integral;
	float pixel_flow_y_integral;
	float gyro_x_rate_integral;
	float gyro_y_rate_integral;
	float gyro_z_rate_integral;
	float ground_distance_m;
	uint32_t integration_timespan;
	uint32_t time_since_last_sonar_update;
	float max_flow_rate;
	float min_ground_distance;
	float max_ground_distance;
	uint16_t frame_count_since_last_readout;
	int16_t gyro_temperature;
	uint8_t sensor_id;
	uint8_t quality;
	uint8_t mode;
	uint8_t _padding0[5]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t MODE_UNKNOWN = 0;
	static constexpr uint8_t MODE_BRIGHT = 1;
	static constexpr uint8_t MODE_LOWLIGHT = 2;
	static constexpr uint8_t MODE_SUPER_LOWLIGHT = 3;

#endif
};

