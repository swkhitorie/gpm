/* Auto-generated by genmsg_cpp from file /home/cwkj/PX4-Autopilot/msg/sensors_status.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT sensors_status_s {
#else
struct sensors_status_s {
#endif
	uint64_t timestamp;
	uint32_t device_id_primary;
	uint32_t device_ids[4];
	float inconsistency[4];
	bool healthy[4];
	uint8_t priority[4];
	bool enabled[4];
	bool external[4];
	uint8_t _padding0[4]; // required for logger


#ifdef __cplusplus

#endif
};
