#pragma once


#include <uORB/uORB.h>

#ifndef __cplusplus

#endif

#ifdef __cplusplus
struct __EXPORT ekf_gps_drift_s {
#else
struct ekf_gps_drift_s {
#endif

	uint64_t timestamp;		// # time since system start (microseconds)
	float hpos_drift_rate;		// # Horizontal position rate magnitude checked using EKF2_REQ_HDRIFT (m/s)
	float vpos_drift_rate;		// # Vertical position rate magnitude checked using EKF2_REQ_VDRIFT (m/s)
	float hspd;			// # Filtered horizontal velocity magnitude checked using EKF2_REQ_HDRIFT (m/s)
	bool blocked;			// # true when drift calculation is blocked due to IMU movement check controlled by EKF2_MOVE_TEST

#ifdef __cplusplus

#endif
};
