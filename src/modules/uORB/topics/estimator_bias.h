
#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT estimator_bias_s {
#else
struct estimator_bias_s {
#endif

	uint64_t timestamp;                // time since system start (microseconds)
	uint64_t timestamp_sample;         // the timestamp of the raw data (microseconds)

	uint32_t device_id;		// unique device ID for the sensor that does not change between power cycles
	float bias;			// estimated barometric altitude bias (m)
	float bias_var;		// estimated barometric altitude bias variance (m^2)

	float innov;			// innovation of the last measurement fusion (m)
	float innov_var;		// innovation variance of the last measurement fusion (m^2)
	float innov_test_ratio;	// normalized innovation squared test ratio


#ifdef __cplusplus

#endif
};

