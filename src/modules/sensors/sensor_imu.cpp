#include "sensor_imu.hpp"
#include <float.h>
using namespace matrix;

using math::constrain;

namespace sensors
{
bool SensorIMU::param_update()
{
    bool result = _param_imu_accel_filter_cutoff.update() && _param_imu_gyro_filter_cutoff.update();
    
    // _calibration.ParametersUpdate();
    
    if (result) {
        
        if ((fabsf(_lp_accel_filter.get_cutoff_freq() - _param_imu_accel_filter_cutoff.get()) > 0.1f)) {
            _lp_accel_filter.set_cutoff_frequency(_imu_integ_rate, _param_imu_accel_filter_cutoff.get());
            _lp_accel_filter.reset(_acceleration_prev);
        }
        
        if ((fabsf(_lp_gyro_filter.get_cutoff_freq() - _param_imu_gyro_filter_cutoff.get()) > 0.1f)) {
            _lp_gyro_filter.set_cutoff_frequency(_imu_integ_rate, _param_imu_gyro_filter_cutoff.get());
            _lp_gyro_filter.reset(_angular_velocity_prev);
        }
    }
    
    return result;
}

bool SensorIMU::start()
{
    _imu_integ_rate = 1000;

    const uint32_t imu_integration_rate_hz = math::constrain(_imu_integ_rate, (uint32_t)50, (uint32_t)1000);
    
    _imu_integration_interval_us = 1e6f / imu_integration_rate_hz;

    _param_imu_accel_filter_cutoff.set(30.0f);
    _param_imu_gyro_filter_cutoff.set(50.0f);
    
    _lp_accel_filter.set_cutoff_frequency(1000, _param_imu_accel_filter_cutoff.get());
    _lp_gyro_filter.set_cutoff_frequency(1000, _param_imu_gyro_filter_cutoff.get());
    
	_accel_integrator.set_reset_interval(_imu_integration_interval_us);
	_gyro_integrator.set_reset_interval(_imu_integration_interval_us);

    uorb_subscribe_topics<uORB::TOPIC_ESTIMATOR_BIAS>(&_sub_estimator_sensor_bias);
    uorb_subscribe_topics<uORB::TOPIC_SENSOR_ACCEL>(&_sub_sensor_accel);
    uorb_subscribe_topics<uORB::TOPIC_SENSOR_GYRO>(&_sub_sensor_gyro);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_CONTROL_MODE>(&_sub_vehicle_controlmode);
    uorb_subscribe_topics<uORB::TOPIC_ESTIMATOR_STATES>(&_sub_estimator_state);

    _accel_calibration.clear();
    _gyro_calibration.clear();
    
	return param_update();
}

bool SensorIMU::run()
{
	const hrt_abstime now_us = hrt_absolute_time();

    bool updated = false;

    if (update_gyro()) {
        updated = true;
    }
    
    if (update_accel()) {
        updated = true;
    }
    
    Vector3f accel_raw{_sub_sensor_accel->x, _sub_sensor_accel->y, _sub_sensor_accel->z};
    Vector3f gyro_raw{_sub_sensor_gyro->x, _sub_sensor_gyro->y, _sub_sensor_gyro->z};
    
    _accel_calibration.run(accel_raw, gyro_raw.length());
    _gyro_calibration.run(gyro_raw);
    
    _accel_interval_us = _accel_mean_interval_us.mean();
    _gyro_interval_us = _gyro_mean_interval_us.mean();
    
    publish();
    
    return updated;
}

bool SensorIMU::update_accel()
{
	bool updated = false;
	// integrate queued accel
    if (_sub_sensor_accel->timestamp_sample > _accel_timestamp_sample_last) {
        if (_accel_timestamp_sample_last != 0) {
            const float interval_us = _sub_sensor_accel->timestamp_sample - _accel_timestamp_sample_last;

            _accel_mean_interval_us.update(interval_us);

            // check measured interval periodically
            if (_accel_mean_interval_us.valid() && (_accel_mean_interval_us.count() % 10 == 0)) {
                
                const float interval_mean = _accel_mean_interval_us.mean();
                
                // update sample rate if previously invalid or changed by more than 1 standard deviation
                const bool diff_exceeds_stddev = sq(interval_mean - _accel_interval_us) > _accel_mean_interval_us.variance();
                
                if (!PX4_ISFINITE(_accel_interval_us) || diff_exceeds_stddev) {
                    // update integrator configuration if interval has changed by more than 10%
                    _update_integrator_config = true;
                }
            } 
        }
    }
    
    const float dt = (_sub_sensor_accel->timestamp_sample - _accel_timestamp_sample_last) * 1e-6f;
    _accel_timestamp_sample_last = _sub_sensor_accel->timestamp_sample;

    const Vector3f accel_raw{_sub_sensor_accel->x, _sub_sensor_accel->y, _sub_sensor_accel->z};
    _raw_accel_mean.update(accel_raw);
    _accel_integrator.put(accel_raw, dt);

    updated = true;
	return updated;
}

bool SensorIMU::update_gyro()
{
	bool updated = false;

    if (_sub_sensor_gyro->timestamp_sample > _gyro_timestamp_sample_last) {
        if (_gyro_timestamp_sample_last != 0) {
            const float interval_us = _sub_sensor_gyro->timestamp_sample - _gyro_timestamp_sample_last;

            _gyro_mean_interval_us.update(interval_us);
  
            // check measured interval periodically
            if (_gyro_mean_interval_us.valid() && (_gyro_mean_interval_us.count() % 10 == 0)) {
                const float interval_mean = _gyro_mean_interval_us.mean();

                // update sample rate if previously invalid or changed by more than 1 standard deviation
                const bool diff_exceeds_stddev = sq(interval_mean - _gyro_interval_us) > _gyro_mean_interval_us.variance();

                if (!PX4_ISFINITE(_gyro_interval_us) || diff_exceeds_stddev) {
                    // update integrator configuration if interval has changed by more than 10%
                    _update_integrator_config = true;
                }
            }
        }
    }
    
    const float dt = (_sub_sensor_gyro->timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;

    _gyro_timestamp_sample_last = _sub_sensor_gyro->timestamp_sample;
    _gyro_timestamp_last = _sub_sensor_gyro->timestamp;
    
    const Vector3f gyro_raw{_sub_sensor_gyro->x, _sub_sensor_gyro->y, _sub_sensor_gyro->z};
    _raw_gyro_mean.update(gyro_raw);
    _gyro_integrator.put(gyro_raw, dt);
    
    updated = true;
	return updated;
}

bool SensorIMU::publish()
{
	bool updated = false;

    Vector3f accel_raw{_sub_sensor_accel->x, _sub_sensor_accel->y, _sub_sensor_accel->z};
    Vector3f gyro_raw{_sub_sensor_gyro->x, _sub_sensor_gyro->y, _sub_sensor_gyro->z};
    
    if (!accel_raw.isAllFinite() || !gyro_raw.isAllFinite()) return updated;
    
    // vehicle_imu_status
    // publish before vehicle_imu so that error counts are available synchronously if needed
    const bool status_publish_interval_exceeded = (hrt_elapsed_time(&_pub_vehicle_imu_status.timestamp) >= kIMUStatusPublishingInterval);

    if (_raw_accel_mean.valid() && _raw_gyro_mean.valid()
        && _accel_mean_interval_us.valid() && _gyro_mean_interval_us.valid()
        && status_publish_interval_exceeded) {
            
        {
            _pub_vehicle_imu_status.accel_device_id = _sub_sensor_gyro->device_id;
            _pub_vehicle_imu_status.accel_rate_hz = 1e6f / _accel_mean_interval_us.mean();

            // accel mean and variance
            const Dcmf &R{};// = _accel_calibration.rotation();
            Vector3f(R * _raw_accel_mean.mean()).copyTo(_pub_vehicle_imu_status.mean_accel);
                
            // variance from R * COV * R^T
            const Matrix3f cov = R * _raw_accel_mean.covariance() * R.transpose();
            cov.diag().copyTo(_pub_vehicle_imu_status.var_accel);
        }
        
        {
            _pub_vehicle_imu_status.gyro_device_id = _sub_sensor_gyro->device_id;
            _pub_vehicle_imu_status.gyro_rate_hz = 1e6f / _gyro_mean_interval_us.mean();

            // gyro mean and variance
            const Dcmf &R{}; //_gyro_calibration.rotation();
            Vector3f(R * _raw_gyro_mean.mean()).copyTo(_pub_vehicle_imu_status.mean_gyro);

            // variance from R * COV * R^T
            const Matrix3f cov = R * _raw_gyro_mean.covariance() * R.transpose();
            cov.diag().copyTo(_pub_vehicle_imu_status.var_gyro);
        }
        _pub_vehicle_imu_status.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_VEHICLE_IMU_STATUS>(&_pub_vehicle_imu_status);
    }
     
    Vector3f accel_correct{};
    Vector3f gyro_correct{};
            
    _gyro_bias.setZero();
    
    accel_correct = _accel_calibration.correct(accel_raw) - _accel_bias;
    gyro_correct = _gyro_calibration.correct(gyro_raw) - _gyro_bias;   

    accel_correct = _lp_accel_filter.apply(accel_correct);
    gyro_correct = _lp_gyro_filter.apply(gyro_correct);

    _pub_sensor_combined.timestamp = _gyro_timestamp_sample_last;
    _pub_sensor_combined.accelerometer_timestamp_relative = _accel_timestamp_sample_last; 
    _pub_sensor_combined.accelerometer_integral_dt = _accel_interval_us;
    _pub_sensor_combined.gyro_integral_dt = _gyro_interval_us;
    accel_correct.copyTo(_pub_sensor_combined.accelerometer_m_s2);
    gyro_correct.copyTo(_pub_sensor_combined.gyro_rad);
    _pub_sensor_combined.accel_calibration_count = 0;
    _pub_sensor_combined.gyro_calibration_count = 0;
    uorb_publish_topics<uORB::TOPIC_SENSOR_COMBINED>(&_pub_sensor_combined);

    _gyro_publish_latency_mean_us.update(_pub_sensor_combined.timestamp - _gyro_timestamp_last);
    _gyro_update_latency_mean_us.update(_pub_sensor_combined.timestamp - _gyro_timestamp_sample_last);
    
    update_accel_vibrationmetrics(accel_raw);
    update_gyro_vibrationmetrics(gyro_raw);
    updated = true;                
        
	return updated;
}

void SensorIMU::update_accel_vibrationmetrics(const Vector3f &acceleration)
{
	// Accel high frequency vibe = filtered length of (acceleration - acceleration_prev)
	_pub_vehicle_imu_status.accel_vibration_metric = 0.99f * _pub_vehicle_imu_status.accel_vibration_metric
					 + 0.01f * Vector3f(acceleration - _acceleration_prev).norm();
	_acceleration_prev = acceleration;
}

void SensorIMU::update_gyro_vibrationmetrics(const Vector3f &angular_velocity)
{
	// Gyro high frequency vibe = filtered length of (angular_velocity - angular_velocity_prev)
	_pub_vehicle_imu_status.gyro_vibration_metric = 0.99f * _pub_vehicle_imu_status.gyro_vibration_metric
					+ 0.01f * Vector3f(angular_velocity - _angular_velocity_prev).norm();
	_angular_velocity_prev = angular_velocity;
}
void SensorIMU::update_integrator_configuration() {}
void SensorIMU::sensor_calibration_update() {}
void SensorIMU::sensor_calibration_save_accel() {}
void SensorIMU::sensor_calibration_save_gyro() {}
} // namespace sensors
