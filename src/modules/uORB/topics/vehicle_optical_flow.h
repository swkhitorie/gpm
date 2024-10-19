#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT vehicle_optical_flow_s {
#else
struct vehicle_optical_flow_s {
#endif

	// Optical flow in XYZ body frame in SI units.

	uint64_t timestamp;               // time since system start (microseconds)
	uint64_t timestamp_sample;

	uint32_t device_id;               // unique device ID for the sensor that does not change between power cycles

	float pixel_flow[2];          // (radians) accumulated optical flow in radians where a positive value is produced by a RH rotation about the body axis

	float delta_angle[3];         // (radians) accumulated gyro radians where a positive value is produced by a RH rotation about the body axis. (NAN if unavailable)

	float distance_m;             // (meters) Distance to the center of the flow field (NAN if unavailable)

	uint32_t integration_timespan_us; // (microseconds) accumulation timespan in microseconds

	uint8_t quality;                  // Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

	float max_flow_rate;          // (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably

	float min_ground_distance;    // (meters) Minimum distance from ground at which the optical flow sensor operates reliably
	float max_ground_distance;    // (meters) Maximum distance from ground at which the optical flow sensor operates reliably


#ifdef __cplusplus

#endif
};

