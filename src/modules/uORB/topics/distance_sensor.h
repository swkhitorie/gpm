#pragma once


#include <uORB/uORB.h>

#ifndef __cplusplus

#endif

#ifdef __cplusplus
struct __EXPORT distance_sensor_s {
#else
struct distance_sensor_s {
#endif

	uint64_t timestamp;		// time since system start (microseconds)

	uint32_t device_id;		// unique device ID for the sensor that does not change between power cycles

	float min_distance;		// Minimum distance the sensor can measure (in m)
	float max_distance;		// Maximum distance the sensor can measure (in m)
	float current_distance;	// Current distance reading (in m)
	float variance;		// Measurement variance (in m^2), 0 for unknown / invalid readings
	int8_t signal_quality;		// Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.

	uint8_t type;			// Type from MAV_DISTANCE_SENSOR enum
	static constexpr uint8_t MAV_DISTANCE_SENSOR_LASER = 0;
	static constexpr uint8_t MAV_DISTANCE_SENSOR_ULTRASOUND = 1;
	static constexpr uint8_t MAV_DISTANCE_SENSOR_INFRARED = 2;
	static constexpr uint8_t MAV_DISTANCE_SENSOR_RADAR = 3;

	float h_fov; // Sensor horizontal field of view (rad)
	float v_fov; // Sensor vertical field of view (rad)
	float q[4]; // Quaterion sensor orientation with respect to the vehicle body frame to specify the orientation ROTATION_CUSTOM

	uint8_t orientation;		// Direction the sensor faces from MAV_SENSOR_ORIENTATION enum

	static constexpr uint8_t ROTATION_YAW_0		= 0; // MAV_SENSOR_ROTATION_NONE
	static constexpr uint8_t ROTATION_YAW_45		= 1; // MAV_SENSOR_ROTATION_YAW_45
	static constexpr uint8_t ROTATION_YAW_90		= 2; // MAV_SENSOR_ROTATION_YAW_90
	static constexpr uint8_t ROTATION_YAW_135		= 3; // MAV_SENSOR_ROTATION_YAW_135
	static constexpr uint8_t ROTATION_YAW_180		= 4; // MAV_SENSOR_ROTATION_YAW_180
	static constexpr uint8_t ROTATION_YAW_225		= 5; // MAV_SENSOR_ROTATION_YAW_225
	static constexpr uint8_t ROTATION_YAW_270		= 6; // MAV_SENSOR_ROTATION_YAW_270
	static constexpr uint8_t ROTATION_YAW_315		= 7; // MAV_SENSOR_ROTATION_YAW_315

	static constexpr uint8_t ROTATION_FORWARD_FACING	= 0; // MAV_SENSOR_ROTATION_NONE
	static constexpr uint8_t ROTATION_RIGHT_FACING	= 2; // MAV_SENSOR_ROTATION_YAW_90
	static constexpr uint8_t ROTATION_BACKWARD_FACING	= 4; // MAV_SENSOR_ROTATION_YAW_180
	static constexpr uint8_t ROTATION_LEFT_FACING	= 6; // MAV_SENSOR_ROTATION_YAW_270

	static constexpr uint8_t ROTATION_UPWARD_FACING   = 24; // MAV_SENSOR_ROTATION_PITCH_90
	static constexpr uint8_t ROTATION_DOWNWARD_FACING = 25; // MAV_SENSOR_ROTATION_PITCH_270

	static constexpr uint8_t ROTATION_CUSTOM          = 100; // MAV_SENSOR_ROTATION_CUSTOM

#ifdef __cplusplus

#endif
};


