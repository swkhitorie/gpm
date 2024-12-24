#include "attitude_estimator.h"
#include "utility/log.hpp"
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;
using matrix::wrap_2pi;
using namespace UParams;

AttitudeEstimator::AttitudeEstimator() { init(); }
void AttitudeEstimator::update_parameters() 
{ 
    _param_step_beta = 0.01f; 
}
void AttitudeEstimator::init()
{
    uorb_subscribe_topics<uORB::TOPIC_SENSOR_COMBINED>(&_sub_sensor_combined);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_MAGNETOMETER>(&_sub_vehicle_magnetometer);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_GPS_POSITION>(&_sub_vgps);
    update_parameters();    
    _initialized = true;
}

void AttitudeEstimator::Run()
{
    update_sensors();
    update_magnetometer();
    update();
}

void AttitudeEstimator::update_sensors()
{
    if (!_initialized) return;

    // update validator with recent sensor data
    if (_sub_sensor_combined->timestamp > 0) {
        
        _imu_timestamp = _sub_sensor_combined->timestamp;
        _gyro(0) = _sub_sensor_combined->gyro_rad[0];
        _gyro(1) = _sub_sensor_combined->gyro_rad[1];
        _gyro(2) = _sub_sensor_combined->gyro_rad[2];
        
        _accel(0) = _sub_sensor_combined->accelerometer_m_s2[0];
        _accel(1) = _sub_sensor_combined->accelerometer_m_s2[1];
        _accel(2) = _sub_sensor_combined->accelerometer_m_s2[2];
        
        if (_accel.length() < 0.01f) {
            // degenerate accel!
            return;
        }        
    }
}

void AttitudeEstimator::update_magnetometer()
{
    if (!_initialized) return;
    _mag(0) = _sub_vehicle_magnetometer->magnetometer_ga[0];
    _mag(1) = _sub_vehicle_magnetometer->magnetometer_ga[1];
    _mag(2) = _sub_vehicle_magnetometer->magnetometer_ga[2];

    if (_mag.length() < 0.01f) {
        // degenerate mag!
        return;
    }
}

void AttitudeEstimator::update()
{
    if (!_initialized) return;
    
    matrix::Quatf qdot, q_acc_corr;
    float _beta_corr = 0.0f;
    
	// time from previous iteration
	hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _imu_prev_timestamp) / 1e6f, _dt_min, _dt_max);
	_imu_prev_timestamp = now;

    float spinRate = _gyro.length();
    
    float accel_len = _accel.length() - GRAVITY_MSS;

    qdot = _q.derivative1(_gyro); 
    
	if(_accel.isAllFinite()) {
        _accel.normalize();
        
        {
            // Gradient decent algorithm corrective step
            matrix::Vector3f vector_acc_corr(
                2 * (_q(1)*_q(3) - _q(0)*_q(2)) - _accel(0), 
                2 * (_q(2)*_q(3) + _q(0)*_q(1)) - _accel(1), 
                1 - 2 * (_q(1)*_q(1) + _q(2)*_q(2)) - _accel(2)
            );
            
            float init_acc_corr_jacobian[3][4] = {
                {-2*_q(2),  2*_q(3),    -2*_q(0),   2*_q(1)},
                {2*_q(1),   2*_q(0),    2*_q(3),    2*_q(2)},
                {0,         -4*_q(1),   -4*_q(2),   0      },
            };
            matrix::Matrix<float, 3, 4> vecor_acc_corr_q_jacobian(init_acc_corr_jacobian);
            q_acc_corr = vecor_acc_corr_q_jacobian.transpose() * vector_acc_corr;
            q_acc_corr.normalize();
        }
        
		_beta_corr  = _param_step_beta + 0.01f * dt * math::constrain(spinRate, 0.0f, 500.0f);
		_beta_corr -= 0.0005f * (accel_len / GRAVITY_MSS);
		_beta_corr  = math::constrain(_beta_corr, _param_step_beta, 0.06f);
        
		// Apply feedback step
        qdot -= (q_acc_corr) * _beta_corr;
	}
    
    _q += qdot * dt;
    _q.normalize();

    matrix::Eulerf _att(_q);  
    matrix::Eulerf _att_no_yaw(_att.phi(), _att.theta(), 0.0f); 
    matrix::Quatf _q_noyaw(_att_no_yaw);
    
    matrix::Vector3f gyro_ground = _q_noyaw.rotateVectorInverse(_gyro);
    _yaw_integral += -gyro_ground(2) * dt;
    
    matrix::Vector3f mag_earth = _q_noyaw.rotateVector(_mag);
    if (_sub_vgps->fix_type > 2) {
        _mag_decl = get_mag_declination_radians((float)_sub_vgps->lat * 1e-7f, (float)_sub_vgps->lon * 1e-7f);
    }
    // wrap_2pi / pi
    _yaw_observe = wrap_2pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
    _yaw_integral = _yaw_integral * (1 - 0.025f) + _yaw_observe * 0.025f;
    _yaw_integral = wrap_2pi(_yaw_integral);
    
    matrix::Quatf _q_fusion(matrix::Eulerf(_att.phi(), -_att.theta(), _yaw_integral));
    
    _pub_vehicle_attitude.timestamp = hrt_absolute_time();
    _pub_vehicle_attitude.timestamp_sample = now;
    _pub_vehicle_attitude.quat_reset_counter = 0;
    _q_fusion.copyTo(_pub_vehicle_attitude.q);
    // uorb_publish_topics<uORB::TOPIC_VEHICLE_ATTITUDE>(&_pub_vehicle_attitude);  
    
    if (!_q.isAllFinite()) {
        _q(0) = 1.0f;
        _q(1) = _q(2) = _q(3) = 0.0f;
    }

//    EDEBUG("%.3f, %.3f, (%.3f | %.3f)\n", 
//        math::degrees(_att.phi()), 
//        math::degrees(_att.theta()),
//        math::degrees(_att.psi()),
//        math::degrees(wrap_2pi(atan2f(_mag(0), _mag(1)))));
}
