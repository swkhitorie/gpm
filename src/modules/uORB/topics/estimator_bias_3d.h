#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT estimator_bias_3d_s {
#else
struct estimator_bias_3d_s {
#endif

	uint64_t timestamp;                // time since system start (microseconds)
	uint64_t timestamp_sample;         // the timestamp of the raw data (microseconds)

	uint32_t device_id;                // unique device ID for the sensor that does not change between power cycles

	float bias[3];                 // estimated barometric altitude bias (m)
	float bias_var[3];             // estimated barometric altitude bias variance (m^2)

	float innov[3];                // innovation of the last measurement fusion (m)
	float innov_var[3];            // innovation variance of the last measurement fusion (m^2)
	float innov_test_ratio[3];     // normalized innovation squared test ratio

#ifdef __cplusplus

#endif
};
