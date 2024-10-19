#pragma once

#include <uORB/uORB.h>

#ifndef __cplusplus

#endif

#ifdef __cplusplus
struct __EXPORT vehicle_optical_flow_vel_s {
#else
struct vehicle_optical_flow_vel_s {
#endif

	uint64_t timestamp;                       // time since system start (microseconds)
	uint64_t timestamp_sample;                // the timestamp of the raw data (microseconds)

	float vel_body[2];                    // velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)
	float vel_ne[2];                      // same as vel_body but in local frame (m/s)

	float flow_rate_uncompensated[2];     // integrated optical flow measurement (rad/s)
	float flow_rate_compensated[2];       // integrated optical flow measurement compensated for angular motion (rad/s)

	float gyro_rate[3];                   // gyro measurement synchronized with flow measurements (rad/s)

	float gyro_bias[3];
	float ref_gyro[3];

	// TOPICS estimator_optical_flow_vel vehicle_optical_flow_vel

#ifdef __cplusplus

#endif
};
