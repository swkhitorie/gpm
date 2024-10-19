#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT vehicle_local_position_setpoint_s {
#else
struct vehicle_local_position_setpoint_s {
#endif
	uint64_t timestamp;
	float x;
	float y;
	float z;
	float yaw;
	float yawspeed;
	float vx;
	float vy;
	float vz;
	float acceleration[3];
	float jerk[3];
	float thrust[3];
	uint8_t _padding0[4]; // required for logger


#ifdef __cplusplus

#endif
};

