#include "sensor_compass.hpp"

namespace sensors
{

using namespace matrix;

static constexpr uint32_t SENSOR_TIMEOUT{300}; //300ms

bool SensorCompass::param_update()
{
    bool res = _param_compass_filter_cutoff.update();
    
    if (res) {
        if ((fabsf(_lp_compass_filter.get_cutoff_freq() - _param_compass_filter_cutoff.get()) > 0.1f)) {
            _lp_compass_filter.set_cutoff_frequency(_compass_rate, _param_compass_filter_cutoff.get());
            _lp_compass_filter.reset(_compass_prev);
        }
    }
	return res;
}

bool SensorCompass::start()
{
    uorb_subscribe_topics<uORB::TOPIC_SENSOR_MAG>(&_sub_sensor_mag);
    uorb_subscribe_topics<uORB::TOPIC_MAGNETOMETER_BIAS_ESTIMATE>(&_sub_magnetometer_bias_estimate);
    uorb_subscribe_topics<uORB::TOPIC_ESTIMATOR_SENSOR_BIAS>(&_sub_estimator_sensor_bias);
    
    _param_compass_filter_cutoff.set(80.0f);
    _compass_timestamp_sample_last = 0;
    
	return param_update();
}

void SensorCompass::run()
{
	const hrt_abstime time_now_us = hrt_absolute_time();

    // _calibration[i].SensorCorrectionsUpdate();
	update_powercompensation();
	update_magbiasestimate();

    if (_sub_sensor_mag->timestamp_sample > _compass_timestamp_sample_last) {
        _compass_timestamp_sample_last = _sub_sensor_mag->timestamp_sample;
        
        Vector3f mag_raw{_sub_sensor_mag->x, _sub_sensor_mag->y, _sub_sensor_mag->z};
        
        if (_calibration.running())
            _calibration.new_sample(mag_raw);

        Vector3f mag_correct = _calibration.correct(mag_raw);
        
        mag_correct = _lp_compass_filter.apply(mag_correct);

        _pub_vehicle_mag.device_id = _sub_sensor_mag->device_id;
        _pub_vehicle_mag.timestamp = hrt_absolute_time();
        _pub_vehicle_mag.timestamp_sample = _sub_sensor_mag->timestamp_sample;
        _pub_vehicle_mag.magnetometer_ga[0] = mag_correct(0);
        _pub_vehicle_mag.magnetometer_ga[1] = mag_correct(1);
        _pub_vehicle_mag.magnetometer_ga[2] = mag_correct(2);
        uorb_publish_topics<uORB::TOPIC_VEHICLE_MAGNETOMETER>(&_pub_vehicle_mag);
        
        check_failover(time_now_us);
        calc_maginconsistency();
        update_magcalibration();
        update_status();
        _compass_prev = mag_correct;        
    }
}

void SensorCompass::calc_maginconsistency() {}
void SensorCompass::update_status() {}
void SensorCompass::check_failover(const hrt_abstime &time_now_us) {}
void SensorCompass::update_magbiasestimate() {}
void SensorCompass::update_magcalibration() {}
void SensorCompass::update_powercompensation() {}
}; // namespace sensors
