#include "../local_position_estimator.hpp"
#include <matrix/matrix/math.hpp>

/*
    移植修改部分
        uorb sub 和 pub的对象修改
        mavlink log 注释掉
        增加OK宏
*/

// required number of samples for sensor to initialize.
// This is a vision based position measurement so we assume
// as soon as we get one measurement it is initialized.
static const uint32_t		REQ_VISION_INIT_COUNT = 1;

// We don't want to deinitialize it because
// this will throw away a correction before it starts using the data so we
// set the timeout to 0.5 seconds
static const uint32_t		VISION_TIMEOUT = 500000;	// 0.5 s

// set pose/velocity as invalid if standard deviation is bigger than EP_MAX_STD_DEV
// TODO: the user should be allowed to set these values by a parameter
static constexpr float 	EP_MAX_STD_DEV = 100.0f;

void BlockLocalPositionEstimator::visionInit()
{
	// measure
	Vector<float, n_y_vision> y;

	if (visionMeasure(y) != OK) {
		_visionStats.reset();
		return;
	}

	// increament sums for mean
	if (_visionStats.getCount() > REQ_VISION_INIT_COUNT) {
//		mavlink_log_info(&mavlink_log_pub, "[lpe] vision position init: "
//				 "%5.2f %5.2f %5.2f m std %5.2f %5.2f %5.2f m",
//				 double(_visionStats.getMean()(0)),
//				 double(_visionStats.getMean()(1)),
//				 double(_visionStats.getMean()(2)),
//				 double(_visionStats.getStdDev()(0)),
//				 double(_visionStats.getStdDev()(1)),
//				 double(_visionStats.getStdDev()(2)));
		_sensorTimeout &= ~SENSOR_VISION;
		_sensorFault &= ~SENSOR_VISION;

		// get reference for global position
		_ref_lat = _global_local_proj_ref.getProjectionReferenceLat();
		_ref_lon = _global_local_proj_ref.getProjectionReferenceLon();
		_ref_alt = _global_local_alt0;

		_is_global_cov_init = _global_local_proj_ref.isInitialized();

		if (!_map_ref.isInitialized() && _is_global_cov_init) {
			// initialize global origin using the visual estimator reference
//			mavlink_log_info(&mavlink_log_pub, "[lpe] global origin init (vision) : lat %6.2f lon %6.2f alt %5.1f m",
//					 double(_ref_lat), double(_ref_lon), double(_ref_alt));
			_map_ref.initReference(_ref_lat, _ref_lon, hrt_absolute_time());
			// set timestamp when origin was set to current time
			_time_origin = _timeStamp;
		}

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOriginGlobal = true;
			_altOrigin = _global_local_proj_ref.isInitialized() ? _ref_alt : 0.0f;
		}
	}
}

int BlockLocalPositionEstimator::visionMeasure(Vector<float, n_y_vision> &y)
{
	if (PX4_ISFINITE(_sub_visual_odom->position_variance[0])) {
		// check if the vision data is valid based on the covariances
		_vision_eph = sqrtf(fmaxf(_sub_visual_odom->position_variance[0], _sub_visual_odom->position_variance[1]));
		_vision_epv = sqrtf(_sub_visual_odom->position_variance[2]);
		_vision_xy_valid = _vision_eph <= EP_MAX_STD_DEV;
		_vision_z_valid = _vision_epv <= EP_MAX_STD_DEV;

	} else {
		// if we don't have covariances, assume every reading
		_vision_xy_valid = true;
		_vision_z_valid = true;
	}

	if (!_vision_xy_valid || !_vision_z_valid) {
		_time_last_vision_p = _sub_visual_odom->timestamp_sample;
		return -1;

	} else {
		_time_last_vision_p = _sub_visual_odom->timestamp_sample;

		if (PX4_ISFINITE(_sub_visual_odom->position[0])) {
			y.setZero();
			y(Y_vision_x) = _sub_visual_odom->position[0];
			y(Y_vision_y) = _sub_visual_odom->position[1];
			y(Y_vision_z) = _sub_visual_odom->position[2];
			_visionStats.update(y);

			return OK;

		} else {
			return -1;
		}
	}
}

void BlockLocalPositionEstimator::visionCorrect()
{
	// measure
	Vector<float, n_y_vision> y;

	if (visionMeasure(y) != OK) {
//		mavlink_log_info(&mavlink_log_pub, "[lpe] vision data invalid. eph: %f epv: %f", (double)_vision_eph,
//				 (double)_vision_epv);
		return;
	}

	// vision measurement matrix, measures position
	Matrix<float, n_y_vision, n_x> C;
	C.setZero();
	C(Y_vision_x, X_x) = 1;
	C(Y_vision_y, X_y) = 1;
	C(Y_vision_z, X_z) = 1;

	// noise matrix
	Matrix<float, n_y_vision, n_y_vision> R;
	R.setZero();

	// use std dev from vision data if available
	if (_vision_eph > _param_lpe_vis_xy.get()) {
		R(Y_vision_x, Y_vision_x) = _vision_eph * _vision_eph;
		R(Y_vision_y, Y_vision_y) = _vision_eph * _vision_eph;

	} else {
		R(Y_vision_x, Y_vision_x) = _param_lpe_vis_xy.get() * _param_lpe_vis_xy.get();
		R(Y_vision_y, Y_vision_y) = _param_lpe_vis_xy.get() * _param_lpe_vis_xy.get();
	}

	if (_vision_epv > _param_lpe_vis_z.get()) {
		R(Y_vision_z, Y_vision_z) = _vision_epv * _vision_epv;

	} else {
		R(Y_vision_z, Y_vision_z) = _param_lpe_vis_z.get() * _param_lpe_vis_z.get();
	}

	// vision delayed x
	uint8_t i_hist = 0;

	float vision_delay = (_timeStamp - _sub_visual_odom->timestamp_sample) * 1e-6f;	// measurement delay in seconds

	if (vision_delay < 0.0f) { vision_delay = 0.0f; }

	// use auto-calculated delay from measurement if parameter is set to zero
	if (getDelayPeriods(_param_lpe_vis_delay.get() > 0.0f ? _param_lpe_vis_delay.get() : vision_delay, &i_hist) < 0) { return; }

	Vector<float, n_x> x0 = _xDelay.get(i_hist);

	// residual
	Matrix<float, n_y_vision, 1> r = y - C * x0;
	// residual covariance
	Matrix<float, n_y_vision, n_y_vision> S = C * m_P * C.transpose() + R;

	// publish innovations
	_pub_innov.ev_hpos[0] = r(0, 0);
	_pub_innov.ev_hpos[1] = r(1, 0);
	_pub_innov.ev_vpos    = r(2, 0);
	_pub_innov.ev_hvel[0] = NAN;
	_pub_innov.ev_hvel[1] = NAN;
	_pub_innov.ev_vvel    = NAN;

	// publish innovation variances
	_pub_innov_var.ev_hpos[0] = S(0, 0);
	_pub_innov_var.ev_hpos[1] = S(1, 1);
	_pub_innov_var.ev_vpos    = S(2, 2);
	_pub_innov_var.ev_hvel[0] = NAN;
	_pub_innov_var.ev_hvel[1] = NAN;
	_pub_innov_var.ev_vvel    = NAN;

	// residual covariance, (inverse)
	Matrix<float, n_y_vision, n_y_vision> S_I = inv<float, n_y_vision>(S);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_vision]) {
		if (!(_sensorFault & SENSOR_VISION)) {
			// mavlink_log_info(&mavlink_log_pub, "[lpe] vision position fault, beta %5.2f", double(beta));
			_sensorFault |= SENSOR_VISION;
		}

	} else if (_sensorFault & SENSOR_VISION) {
		_sensorFault &= ~SENSOR_VISION;
		// mavlink_log_info(&mavlink_log_pub, "[lpe] vision position OK");
	}

	// kalman filter correction if no fault
	if (!(_sensorFault & SENSOR_VISION)) {
		Matrix<float, n_x, n_y_vision> K = m_P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		_x += dx;
		m_P -= K * C * m_P;
	}
}

void BlockLocalPositionEstimator::visionCheckTimeout()
{
	if (_timeStamp - _time_last_vision_p > VISION_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_VISION)) {
			_sensorTimeout |= SENSOR_VISION;
			_visionStats.reset();
			// mavlink_log_critical(&mavlink_log_pub, "[lpe] vision position timeout ");
		}
	}
}
