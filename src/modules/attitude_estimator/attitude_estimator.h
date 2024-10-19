#ifndef ATTITUDE_ESTIMATOR_H_
#define ATTITUDE_ESTIMATOR_H_

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <geo/geo.h>
#include <world_magnetic_model/geo_mag_declination.h>

#include "param/parameters.hpp"
#include "platform_defines.h"
#include "uORB_topic_define.hpp"


class AttitudeEstimator
{
public:
	AttitudeEstimator();
	~AttitudeEstimator() = default;

	void init();

    void Run();

    void update_parameters();

private:

    void update_sensors();

	void update_magnetometer();

	void update();

	const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;
    
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_COMBINED>::type *_sub_sensor_combined;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_MAGNETOMETER>::type *_sub_vehicle_magnetometer;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_GPS_POSITION>::type *_sub_vgps;

    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ATTITUDE>::type _pub_vehicle_attitude;

	matrix::Vector3f    _accel{};
	matrix::Vector3f    _gyro{};
	matrix::Vector3f    _mag{};
    float               _mag_decl{0.0f};
        
    matrix::Quatf       _q{};  
	float               _yaw_integral{0.0f};
    float               _yaw_observe{0.0f};
        
	hrt_abstime _imu_timestamp{};
	hrt_abstime _imu_prev_timestamp{};

	bool        _initialized{false};
    
    float       _param_step_beta;
};

#endif
