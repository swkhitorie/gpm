#include "sensor_opticalflow.hpp"

namespace sensors
{

using namespace matrix;

static constexpr uint32_t SENSOR_TIMEOUT{300};

VehicleOpticalFlow::VehicleOpticalFlow()
{
	uorb_subscribe_topics<uORB::TOPIC_DISTANCE_SENSOR>(&_sub_distance_sensor);
	uorb_subscribe_topics<uORB::TOPIC_VEHICLE_ATTITUDE>(&_sub_vehicle_attitude);
	uorb_subscribe_topics<uORB::TOPIC_SENSOR_OPTICALFLOW>(&_sub_sensor_opticalflow);
	uorb_subscribe_topics<uORB::TOPIC_SENSOR_GYRO>(&_sub_sensor_gyro);
	uorb_subscribe_topics<uORB::TOPIC_SENSOR_SELECTION>(&_sub_sensor_selection);
	_gyro_integrator.set_reset_samples(1);
}


bool VehicleOpticalFlow::start()
{
	param_update();
	return true;
}

void VehicleOpticalFlow::param_update()
{
	if (_param_sens_flow_rot.update() &&
		_param_sens_flow_minhgt.update() &&
		_param_sens_flow_maxhgt.update() &&
		_param_sens_flow_maxr.update() &&
		_param_sens_flow_rate.update()) {
		_flow_rotation = get_rot_matrix(static_cast<enum Rotation>(_param_sens_flow_rot.get()));	
	}
}

void VehicleOpticalFlow::run()
{
	//100hz
	update_distancesensor();

	if (!_delta_angle_available) {
		update_sensorgyro();
	}

    // clear data accumulation if there's a gap in data
    const uint64_t integration_gap_threshold_us = _sub_sensor_opticalflow->integration_timespan_us * 2;

    if ((_sub_sensor_opticalflow->timestamp_sample >= _flow_timestamp_sample_last + integration_gap_threshold_us)
        || (_accumulated_count > 0 && (_sub_sensor_opticalflow->quality > 0) && _quality_sum == 0)) {
        clear_accumulated();
    }

    const hrt_abstime timestamp_oldest = _sub_sensor_opticalflow->timestamp_sample - _sub_sensor_opticalflow->integration_timespan_us;
    const hrt_abstime timestamp_newest = _sub_sensor_opticalflow->timestamp;

    // delta angle
    //  - from _sub_sensor_opticalflow if available, otherwise use synchronized sensor_gyro if available
    if (_sub_sensor_opticalflow->delta_angle_available && Vector2f(_sub_sensor_opticalflow->delta_angle).isAllFinite()) {
        // passthrough integrated gyro if available
        Vector3f delta_angle(_sub_sensor_opticalflow->delta_angle);

        if (!PX4_ISFINITE(delta_angle(2))) {
            // Some sensors only provide X and Y angular rates, rotate them but place back the NAN on the Z axis
            delta_angle(2) = 0.f;
            _delta_angle += _flow_rotation * delta_angle;
            _delta_angle(2) = NAN;

        } else {
            _delta_angle += _flow_rotation * delta_angle;
        }

        _delta_angle_available = true;

    } else {
        _delta_angle_available = false;

        // integrate synchronized gyro
        gyroSample gyro_sample;

        while (_gyro_buffer.pop_oldest(timestamp_oldest, timestamp_newest, &gyro_sample)) {

            _gyro_integrator.put(gyro_sample.data, gyro_sample.dt);

            float min_interval_s = (_sub_sensor_opticalflow->integration_timespan_us * 1e-6f) * 0.99f;

            if (_gyro_integrator.integral_dt() > min_interval_s) {
                //PX4_INFO("integral dt: %.6f, min interval: %.6f", (double)_gyro_integrator.integral_dt(),(double) min_interval_s);
                break;
            }
        }

        Vector3f delta_angle{NAN, NAN, NAN};
        uint16_t delta_angle_dt;

        if (_gyro_integrator.reset(delta_angle, delta_angle_dt)) {
            _delta_angle += delta_angle;

        } else {
            // force integrator reset
            _gyro_integrator.reset();
        }
    }

    // distance
    //  - from _sub_sensor_opticalflow if available, otherwise use downward distance_sensor if available
    if (_sub_sensor_opticalflow->distance_available && PX4_ISFINITE(_sub_sensor_opticalflow->distance_m)) {
        if (!PX4_ISFINITE(_distance_sum)) {
            _distance_sum = _sub_sensor_opticalflow->distance_m;
            _distance_sum_count = 1;

        } else {
            _distance_sum += _sub_sensor_opticalflow->distance_m;
            _distance_sum_count += 1;
        }

    } else {
        // otherwise use buffered downward facing distance_sensor if available
        rangeSample range_sample;

        if (_range_buffer.peak_first_older_than(_sub_sensor_opticalflow->timestamp_sample, &range_sample)) {
            if (!PX4_ISFINITE(_distance_sum)) {
                _distance_sum = range_sample.data;
                _distance_sum_count = 1;

            } else {
                _distance_sum += range_sample.data;
                _distance_sum_count += 1;
            }
        }
    }

    _flow_timestamp_sample_last = _sub_sensor_opticalflow->timestamp_sample;
    _flow_integral(0) += _sub_sensor_opticalflow->pixel_flow[0];
    _flow_integral(1) += _sub_sensor_opticalflow->pixel_flow[1];

    _integration_timespan_us += _sub_sensor_opticalflow->integration_timespan_us;

    _quality_sum += _sub_sensor_opticalflow->quality;
    _accumulated_count++;

    bool publish = true;

    if (_param_sens_flow_rate.get() > 0) {
        const float interval_us = 1e6f / _param_sens_flow_rate.get();

        // don't allow publishing faster than SENS_FLOW_RATE
        if (_integration_timespan_us < interval_us) {
            publish = false;
        }
    }

    if (publish) {
        vehicle_optical_flow_s vehicle_optical_flow{};

        _pub_vehicle_optical_flow.timestamp_sample = _sub_sensor_opticalflow->timestamp_sample;
        _pub_vehicle_optical_flow.device_id = _sub_sensor_opticalflow->device_id;

        _flow_integral.copyTo(_pub_vehicle_optical_flow.pixel_flow);
        _delta_angle.copyTo(_pub_vehicle_optical_flow.delta_angle);

        _pub_vehicle_optical_flow.integration_timespan_us = _integration_timespan_us;

        _pub_vehicle_optical_flow.quality = _quality_sum / _accumulated_count;

        if (_distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
            _pub_vehicle_optical_flow.distance_m = _distance_sum / _distance_sum_count;

        } else {
            _pub_vehicle_optical_flow.distance_m = NAN;
        }

        // SENS_FLOW_MAXR
        if (PX4_ISFINITE(_sub_sensor_opticalflow->max_flow_rate)
            && (_sub_sensor_opticalflow->max_flow_rate <= _param_sens_flow_maxr.get())) {

            _pub_vehicle_optical_flow.max_flow_rate = _sub_sensor_opticalflow->max_flow_rate;

        } else {
            _pub_vehicle_optical_flow.max_flow_rate = _param_sens_flow_maxr.get();
        }

        // SENS_FLOW_MINHGT
        if (PX4_ISFINITE(_sub_sensor_opticalflow->min_ground_distance)
            && (_sub_sensor_opticalflow->min_ground_distance >= _param_sens_flow_minhgt.get())) {

            _pub_vehicle_optical_flow.min_ground_distance = _sub_sensor_opticalflow->min_ground_distance;

        } else {
            _pub_vehicle_optical_flow.min_ground_distance = _param_sens_flow_minhgt.get();
        }

        // SENS_FLOW_MAXHGT
        if (PX4_ISFINITE(_sub_sensor_opticalflow->max_ground_distance)
            && (_sub_sensor_opticalflow->max_ground_distance <= _param_sens_flow_maxhgt.get())) {

            _pub_vehicle_optical_flow.max_ground_distance = _sub_sensor_opticalflow->max_ground_distance;

        } else {
            _pub_vehicle_optical_flow.max_ground_distance = _param_sens_flow_maxhgt.get();
        }


        // rotate (SENS_FLOW_ROT)
        float zeroval = 0.f;
        rotate_3f((enum Rotation)_param_sens_flow_rot.get(), _pub_vehicle_optical_flow.pixel_flow[0],
              _pub_vehicle_optical_flow.pixel_flow[1], zeroval);

        _pub_vehicle_optical_flow.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_VEHICLE_OPTICAL_FLOW>(&_pub_vehicle_optical_flow);

        // vehicle_optical_flow_vel if distance is available (for logging)
        if (_distance_sum_count > 0 && PX4_ISFINITE(_distance_sum)) {
            const float range = _distance_sum / _distance_sum_count;

            vehicle_optical_flow_vel_s _pub_vehicle_optical_flow_vel{};

            _pub_vehicle_optical_flow_vel.timestamp_sample = _pub_vehicle_optical_flow.timestamp_sample;

            // NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
            // is produced by a RH rotation of the image about the sensor axis.
            const Vector2f flow_xy_rad{-_pub_vehicle_optical_flow.pixel_flow[0], -_pub_vehicle_optical_flow.pixel_flow[1]};
            const Vector3f gyro_rate_integral{-_pub_vehicle_optical_flow.delta_angle[0], -_pub_vehicle_optical_flow.delta_angle[1], -_pub_vehicle_optical_flow.delta_angle[2]};

            const float flow_dt = 1e-6f * _pub_vehicle_optical_flow.integration_timespan_us;

            // compensate for body motion to give a LOS rate
            const Vector2f flow_compensated_XY_rad = flow_xy_rad - gyro_rate_integral.xy();

            Vector3f vel_optflow_body;
            vel_optflow_body(0) = - range * flow_compensated_XY_rad(1) / flow_dt;
            vel_optflow_body(1) =   range * flow_compensated_XY_rad(0) / flow_dt;
            vel_optflow_body(2) = 0.f;

            // vel_body
            _pub_vehicle_optical_flow_vel.vel_body[0] = vel_optflow_body(0);
            _pub_vehicle_optical_flow_vel.vel_body[1] = vel_optflow_body(1);

            // vel_ne
            _pub_vehicle_optical_flow_vel.vel_ne[0] = NAN;
            _pub_vehicle_optical_flow_vel.vel_ne[1] = NAN;

            const matrix::Dcmf R_to_earth = matrix::Quatf(_sub_vehicle_attitude->q);
            const Vector3f flow_vel_ne = R_to_earth * vel_optflow_body;

            _pub_vehicle_optical_flow_vel.vel_ne[0] = flow_vel_ne(0);
            _pub_vehicle_optical_flow_vel.vel_ne[1] = flow_vel_ne(1);

            const Vector2f flow_rate(flow_xy_rad * (1.f / flow_dt));
            flow_rate.copyTo(_pub_vehicle_optical_flow_vel.flow_rate_uncompensated);

            const Vector2f flow_rate_compensated(flow_compensated_XY_rad * (1.f / flow_dt));
            flow_rate_compensated.copyTo(_pub_vehicle_optical_flow_vel.flow_rate_compensated);

            const Vector3f measured_body_rate(gyro_rate_integral * (1.f / flow_dt));

            // gyro_rate
            _pub_vehicle_optical_flow_vel.gyro_rate[0] = measured_body_rate(0);
            _pub_vehicle_optical_flow_vel.gyro_rate[1] = measured_body_rate(1);
            _pub_vehicle_optical_flow_vel.gyro_rate[2] = measured_body_rate(2);

            _pub_vehicle_optical_flow_vel.timestamp = hrt_absolute_time();

            uorb_publish_topics<uORB::TOPIC_VEHICLE_OPTICAL_FLOW_VEL>(&_pub_vehicle_optical_flow_vel);
        }

        clear_accumulated();
    }
}

void VehicleOpticalFlow::update_distancesensor()
{
	// update range finder buffer
	if ((hrt_elapsed_time(&_sub_distance_sensor->timestamp) < 100)
		&& (_sub_distance_sensor->orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING)) {
		_last_range_sensor_update = _sub_distance_sensor->timestamp;
		_distance_sensor_selected = 0;
	}
	
	// range sample
	if (_sub_distance_sensor->orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
		if ((_sub_distance_sensor->current_distance >= _sub_distance_sensor->min_distance)
		&& (_sub_distance_sensor->current_distance <= _sub_distance_sensor->max_distance)) {
			rangeSample sample;
			sample.time_us = _sub_distance_sensor->timestamp;
			sample.data = _sub_distance_sensor->current_distance;
			_range_buffer.push(sample);
			_last_range_sensor_update = _sub_distance_sensor->timestamp;
			return;
		}
	} else {
		_distance_sensor_selected = -1;
	}

	if (hrt_elapsed_time(&_last_range_sensor_update) > 1000) {
		_distance_sensor_selected = -1;
	}
}

void VehicleOpticalFlow::update_sensorgyro()
{
	// _gyro_calibration.SensorCorrectionsUpdate();

	const float dt_s = (_sub_sensor_gyro->timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
	_gyro_timestamp_sample_last = _sub_sensor_gyro->timestamp_sample;

	gyroSample gyro_sample;
	gyro_sample.time_us = _sub_sensor_gyro->timestamp_sample;
	gyro_sample.data = //_gyro_calibration.Correct(Vector3f{_sub_sensor_gyro->x, _sub_sensor_gyro->y, _sub_sensor_gyro->z});
    gyro_sample.data = Vector3f{_sub_sensor_gyro->x, _sub_sensor_gyro->y, _sub_sensor_gyro->z};
	gyro_sample.dt = dt_s;

	_gyro_buffer.push(gyro_sample);
}

void VehicleOpticalFlow::clear_accumulated()
{
	_gyro_integrator.reset();
	
	_flow_integral.zero();
	_delta_angle.zero();
	_integration_timespan_us = 0;
	_distance_sum = NAN;
	_distance_sum_count = 0;
	_quality_sum = 0;
	_accumulated_count = 0;
}

}; // namespace sensors
