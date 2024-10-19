#pragma once


#include <uORB/uORB.h>

#ifndef __cplusplus

#endif

#ifdef __cplusplus
struct __EXPORT ekf2_timestamps_s {
#else
struct ekf2_timestamps_s {
#endif

	// this message contains the (relative) timestamps of the sensor inputs used by EKF2.
	// It can be used for reproducible replay.

	// the timestamp field is the ekf2 reference time and matches the timestamp of
	// the sensor_combined topic.

	uint64_t timestamp;			 // time since system start (microseconds)

	static constexpr int16_t RELATIVE_TIMESTAMP_INVALID = 32767; // (0x7fff) If one of the relative timestamps
											 // is set to this value, it means the associated sensor values did not update

	// timestamps are relative to the main timestamp and are in 0.1 ms (timestamp +
	// *_timestamp_rel = absolute timestamp). For int16, this allows a maximum
	// difference of +-3.2s to the sensor_combined topic.

	int16_t airspeed_timestamp_rel;
	int16_t distance_sensor_timestamp_rel;
	int16_t optical_flow_timestamp_rel;
	int16_t vehicle_air_data_timestamp_rel;
	int16_t vehicle_magnetometer_timestamp_rel;
	int16_t visual_odometry_timestamp_rel;

	// Note: this is a high-rate logged topic, so it needs to be as small as possible

#ifdef __cplusplus

#endif
};

