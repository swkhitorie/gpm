#pragma once

/*
 * @file attitude_estimator_q_main.h
 *
 * Attitude estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */
 
#include <float.h>

#include <geo/geo.h>
#include <world_magnetic_model/geo_mag_declination.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include "param/parameters.hpp"
#include "platform_defines.h"
#include "uORB_topic_define.hpp"

class AttitudeEstimatorQ
{
public:

	AttitudeEstimatorQ();
	~AttitudeEstimatorQ() = default;

	void init();

    void Run();

private:

	void update_gps_position();

	void update_magnetometer();

	void update_sensors();

	void update_vehicle_attitude();

	void update_vehicle_local_position();

	void update_parameters();

	bool update(float dt);

	// Update magnetic declination (in rads) immediately changing yaw rotation
	void update_mag_declination(float new_declination);

    bool init_attitude_q();

	const float _eo_max_std_dev = 100.0f;           /**< Maximum permissible standard deviation for estimated orientation */
	const float _dt_min = 0.00001f;
	const float _dt_max = 0.02f;
    
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_COMBINED>::type *_sub_sensor_combined;
    
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ATTITUDE>::type *_sub_vehicle_attitude;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_GPS_POSITION>::type *_sub_vehicle_gps_position;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_LOCAL_POSITION>::type *_sub_vehicle_local_position;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_MAGNETOMETER>::type *_sub_vehicle_magnetometer;

    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ATTITUDE>::type _pub_vehicle_attitude;
    
	matrix::Vector3f    _accel{};
	matrix::Vector3f    _gyro{};
	matrix::Vector3f    _gyro_bias{};
	matrix::Vector3f    _rates{};

	matrix::Vector3f    _mag{};

	matrix::Vector3f    _pos_acc{};
	matrix::Vector3f    _vel_prev{};

    matrix::Quatf       _q{};

	hrt_abstime _imu_timestamp{};
	hrt_abstime _imu_prev_timestamp{};
	hrt_abstime _vel_prev_timestamp{};

	float       _bias_max{};
	float       _mag_decl{};

	bool        _data_good{false};
	bool        _ext_hdg_good{false};
	bool        _initialized{false};
    
    float       _param_att_w_acc;           //ATT_W_ACC
    float       _param_att_w_mag;           //ATT_W_MAG
    float       _param_att_w_ext_hdg;           //ATT_W_EXT_HDG
    float       _param_att_w_gyro_bias;           //ATT_W_GYRO_BIAS
    float       _param_att_mag_decl;           //ATT_MAG_DECL
    int         _param_att_mag_decl_a;           //ATT_MAG_DECL_A
    int         _param_att_ext_hdg_m;           //ATT_EXT_HDG_M
    int         _param_att_acc_comp;           //ATT_ACC_COMP
    float       _param_att_bias_mas;           //ATT_BIAS_MAX
    int         _param_sys_has_mag;           //SYS_HAS_MAG
};

