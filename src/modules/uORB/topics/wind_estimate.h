#pragma once

#include <uORB/uORB.h>

#ifndef __cplusplus

#endif

#ifdef __cplusplus
struct __EXPORT wind_estimate_s {
#else
struct wind_s {
#endif
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float windspeed_north;
	float windspeed_east;
	float variance_north;
	float variance_east;
	float tas_innov;
	float tas_innov_var;
	float tas_scale;
	float beta_innov;
	float beta_innov_var;

#ifdef __cplusplus

#endif
};

