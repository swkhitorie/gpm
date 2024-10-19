#include "attitude_estimator_q.h"

/**
* imu axes(right hand) x to front / y to left / z to up
* this algorithm axes(right hand) x to front / y to right / z to down
    更改 传感器朝向
*/
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::wrap_pi;

using namespace UParams;

AttitudeEstimatorQ::AttitudeEstimatorQ() { init(); }
void AttitudeEstimatorQ::update_parameters() {}
void AttitudeEstimatorQ::init()
{
    _initialized = true;
    _data_good = true;
    
    _param_att_w_acc = 0.2f;           //ATT_W_ACC
    _param_att_w_mag = 0.1f;           //ATT_W_MAG
    _param_att_w_ext_hdg = 0.1f;           //ATT_W_EXT_HDG
    _param_att_w_gyro_bias = 0.1f;           //ATT_W_GYRO_BIAS
    _param_att_mag_decl = 0.0f;           //ATT_MAG_DECL
    _param_att_mag_decl_a = 0;           //ATT_MAG_DECL_A
    _param_att_ext_hdg_m = 0;           //ATT_EXT_HDG_M
    _param_att_acc_comp = 0;           //ATT_ACC_COMP
    _param_att_bias_mas = 0.05f;           //ATT_BIAS_MAX
    _param_sys_has_mag = 1;           //SYS_HAS_MAG
    
    uorb_subscribe_topics<uORB::TOPIC_SENSOR_COMBINED>(&_sub_sensor_combined);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_ATTITUDE>(&_sub_vehicle_attitude);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_GPS_POSITION>(&_sub_vehicle_gps_position);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_LOCAL_POSITION>(&_sub_vehicle_local_position);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_MAGNETOMETER>(&_sub_vehicle_magnetometer);  
    
    init_attitude_q();
}

void AttitudeEstimatorQ::Run()
{
    update_parameters();
    update_sensors();
	update_magnetometer();
    update_gps_position();
	update_vehicle_local_position();
    update_vehicle_attitude();
}

void AttitudeEstimatorQ::update_gps_position()
{
    if (_sub_vehicle_gps_position->eph < 20.0f) {
        // set magnetic declination automatically
        update_mag_declination(get_mag_declination_radians((float)_sub_vehicle_gps_position->lat * 1e-7f,
                       (float)_sub_vehicle_gps_position->lon * 1e-7f));        
    }
}

void AttitudeEstimatorQ::update_magnetometer()
{
    _mag(0) = _sub_vehicle_magnetometer->magnetometer_ga[0];
    _mag(1) = -_sub_vehicle_magnetometer->magnetometer_ga[1];
    _mag(2) = -_sub_vehicle_magnetometer->magnetometer_ga[2];

    if (_mag.length() < 0.01f) {
        // PX4_ERR("degenerate mag!");
        return;
    } 
}

void AttitudeEstimatorQ::update_sensors()
{
    // update validator with recent sensor data
    if (_sub_sensor_combined->timestamp > 0) {
        _imu_timestamp = _sub_sensor_combined->timestamp;
        _gyro(0) = _sub_sensor_combined->gyro_rad[0];
        _gyro(1) = -_sub_sensor_combined->gyro_rad[1];
        _gyro(2) = -_sub_sensor_combined->gyro_rad[2];
        
        _accel(0) = _sub_sensor_combined->accelerometer_m_s2[0];
        _accel(1) = -_sub_sensor_combined->accelerometer_m_s2[1];
        _accel(2) = -_sub_sensor_combined->accelerometer_m_s2[2];
        
        if (_accel.length() < 0.01f) {
            // PX4_ERR("degenerate accel!");
            return;
        }        
    }
}

void AttitudeEstimatorQ::update_vehicle_attitude()
{
	// time from previous iteration
	hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _imu_prev_timestamp) / 1e6f, _dt_min, _dt_max);
	_imu_prev_timestamp = now;

	if (update(dt)) {
		_pub_vehicle_attitude.timestamp_sample = _imu_timestamp;
		_q.copyTo(_pub_vehicle_attitude.q);

		/* the instance count is not used here */
		_pub_vehicle_attitude.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_VEHICLE_ATTITUDE>(&_pub_vehicle_attitude);
	}
}

void AttitudeEstimatorQ::update_vehicle_local_position()
{
    if ((hrt_elapsed_time(&_sub_vehicle_local_position->timestamp) < 20 * 1000) && (_sub_vehicle_local_position->eph < 5.0f)
        && _sub_vehicle_local_position->v_xy_valid && _sub_vehicle_local_position->v_z_valid && _initialized) {

        /* position data is actual */
        const Vector3f vel(_sub_vehicle_local_position->vx, _sub_vehicle_local_position->vy, _sub_vehicle_local_position->vz);

        /* velocity updated */
        if (_vel_prev_timestamp != 0 && _sub_vehicle_local_position->timestamp != _vel_prev_timestamp) {
            float vel_dt = (_sub_vehicle_local_position->timestamp - _vel_prev_timestamp) / 1e6f;
            /* calculate acceleration in body frame */
            _pos_acc = _q.rotateVectorInverse((vel - _vel_prev) / vel_dt);
        }

        _vel_prev_timestamp = _sub_vehicle_local_position->timestamp;
        _vel_prev = vel;

    } else {
        /* position data is outdated, reset acceleration */
        _pos_acc.zero();
        _vel_prev.zero();
        _vel_prev_timestamp = 0;
    }
}


bool AttitudeEstimatorQ::update(float dt)
{
	if (!_initialized) {
		if (!_data_good) {
			return false;
		}
		return init_attitude_q();
	}

	Quatf q_last = _q;

	// Angular rate of correction
	Vector3f corr;
	float spinRate = _gyro.length();

	if (!_ext_hdg_good) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
		Vector3f mag_earth = _q.rotateVector(_mag);
		float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;

		if (spinRate > fifty_dps) {
			gainMult = math::min(spinRate / fifty_dps, 10.0f);
		}

		// Project magnetometer correction to body frame
		corr += _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, -mag_err)) * _param_att_w_mag * gainMult;
	}

	_q.normalize();

	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector3f k = _q.rotateVectorInverse(Vector3f(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros
	Vector3f k(
		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3))
	);

	// If we are not using acceleration compensation based on GPS velocity,
	// fuse accel data only if its norm is close to 1 g (reduces drift).
	const float accel_norm_sq = _accel.norm_squared();
	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;

	if (((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
					  (accel_norm_sq < upper_accel_limit * upper_accel_limit))) {
		corr += (k % (_accel - _pos_acc).normalized()) * _param_att_w_acc;
	}

	// Gyro bias estimation
	if (spinRate < 0.175f) {
		_gyro_bias += corr * (_param_att_w_gyro_bias * dt);

		for (int i = 0; i < 3; i++) {
			_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
		}

	}

	_rates = _gyro + _gyro_bias;

	// Feed forward gyro
	corr += _rates;

	// Apply correction to state
	_q += _q.derivative1(corr) * dt;

	// Normalize quaternion
	_q.normalize();
    
    matrix::Eulerf _att(_q);
    
//    EDEBUG("%.3f, %.3f, %.3f\n", 
//        math::degrees(_att.phi()), 
//        math::degrees(_att.theta()),
//        math::degrees(_att.psi()));
    
	if (!_q.isAllFinite()) {
		// Reset quaternion to last good state
		_q = q_last;
		_rates.zero();
		_gyro_bias.zero();
		return false;
	}

	return true;  
}

bool AttitudeEstimatorQ::init_attitude_q()
{
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector3f k = -_accel;
	k.normalize();

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	Vector3f i = (_mag - k * (_mag * k));
	i.normalize();

	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector3f j = k % i;

	// Fill rotation matrix
	Dcmf R;
	R.row(0) = i;
	R.row(1) = j;
	R.row(2) = k;

	// Convert to quaternion
	_q = R;

	// Compensate for magnetic declination
	Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
	_q = _q * decl_rotation;

	_q.normalize();

	if (_q.isAllFinite() && _q.length() > 0.95f && _q.length() < 1.05f) {
		_initialized = true;

	} else {
		_initialized = false;
	}
    
	return _initialized;
}

void AttitudeEstimatorQ::update_mag_declination(float new_declination)
{
	// Apply initial declination or trivial rotations without changing estimation
	if (!_initialized || fabsf(new_declination - _mag_decl) < 0.0001f) {
		_mag_decl = new_declination;

	} else {
		// Immediately rotate current estimation to avoid gyro bias growth
		Quatf decl_rotation = Eulerf(0.0f, 0.0f, new_declination - _mag_decl);
		_q = _q * decl_rotation;
		_mag_decl = new_declination;
	} 
}

