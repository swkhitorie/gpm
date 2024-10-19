#pragma once


#include <uORB/uORB.h>

#ifndef __cplusplus

#endif

#ifdef __cplusplus
struct __EXPORT ekf_gps_position_s {
#else
struct ekf_gps_position_s {
#endif

	// # EKF blended position in WGS84 coordinates.
	uint64_t timestamp;		// # time since system start (microseconds)
	int32_t lat;			// # Latitude in 1E-7 degrees
	int32_t lon;			// # Longitude in 1E-7 degrees
	int32_t alt;			// # Altitude in 1E-3 meters above MSL, (millimetres)
	int32_t alt_ellipsoid; 		// # Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
	float s_variance_m_s;		// # GPS speed accuracy estimate, (metres/sec)
	uint8_t fix_type; // # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	float eph;			// # GPS horizontal position accuracy (metres)
	float epv;			// # GPS vertical position accuracy (metres)
	float vel_m_s;			// # GPS ground speed, (metres/sec)
	float vel_n_m_s;	// # GPS North velocity, (metres/sec)
	float vel_e_m_s;		// # GPS East velocity, (metres/sec)
	float vel_d_m_s;		// # GPS Down velocity, (metres/sec)
	bool vel_ned_valid;		// # True if NED velocity is valid
	uint8_t satellites_used;		// # Number of satellites used
	float heading;			// # heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
	float heading_offset;		// # heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
	uint8_t selected;			// # GPS selection: 0: GPS1, 1: GPS2. 2: GPS1+GPS2 blend

#ifdef __cplusplus

#endif
};


