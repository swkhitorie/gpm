#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT sensor_selections_s {
#else
struct sensor_selections_s {
#endif

	// Sensor ID's for the voted sensors output on the sensor_combined topic.
	// Will be updated on startup of the sensor module and when sensor selection changes

	uint64_t timestamp;		// time since system start (microseconds)
	uint32_t accel_device_id;		// unique device ID for the selected accelerometers
	uint32_t gyro_device_id;		// unique device ID for the selected rate gyros

#ifdef __cplusplus

#endif
};

