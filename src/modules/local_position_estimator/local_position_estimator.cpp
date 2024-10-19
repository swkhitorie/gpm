#include "local_position_estimator.hpp"
#include <matrix/matrix/math.hpp>
#include <cstdlib>

/*
    移植修改部分
        uorb sub 和 pub的对象修改
        mavlink log 注释掉
        适配已经移植的geo库
        增加OK宏
        修改blockparam相关的初始化, 新增成员_prev_timestamp手动计算适配getDt接口
*/

// required standard deviation of estimate for estimator to publish data
static const uint32_t		EST_STDDEV_XY_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_Z_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_TZ_VALID = 2.0;	// 2.0 m

static const float P_MAX = 1.0e6f;	// max allowed value in state covariance
static const float LAND_RATE = 10.0f;	// rate of land detector correction

static const char *msg_label = "[lpe] ";	// rate of land detector correction

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :

	// flow gyro
	_flow_gyro_x_high_pass(),
	_flow_gyro_y_high_pass(),

	// stats
	_baroStats(),
	_sonarStats(),
	_lidarStats(),
	_flowQStats(),
	_visionStats(),
	_mocapStats(),
	_gpsStats(),

	// low pass
	_xLowPass(),

	// use same lp constant for agl
	_aglLowPass(),

	// delay
	_xDelay(),
	_tDelay(),

	// misc
	_timeStamp(hrt_absolute_time()),
	_time_origin(0),
	_timeStampLastBaro(hrt_absolute_time()),
	_time_last_hist(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_init_sonar(0),
	_time_last_vision_p(0),
	_time_last_mocap(0),
	_time_last_land(0),
	_time_last_target(0),

	// reference altitudes
	_altOrigin(0),
	_altOriginInitialized(false),
	_altOriginGlobal(false),
	_baroAltOrigin(0),
	_gpsAltOrigin(0),

	// status
	_receivedGps(false),
	_lastArmedState(false),

	// masks
	_sensorTimeout(UINT16_MAX),
	_sensorFault(0),
	_estimatorInitialized(0),

	// sensor update flags
	_flowUpdated(false),
	_gpsUpdated(false),
	_visionUpdated(false),
	_mocapUpdated(false),
	_lidarUpdated(false),
	_sonarUpdated(false),
	_landUpdated(false),
	_baroUpdated(false),

	// sensor validation flags
	_vision_xy_valid(false),
	_vision_z_valid(false),
	_mocap_xy_valid(false),
	_mocap_z_valid(false),

	// sensor std deviations
	_vision_eph(0.0),
	_vision_epv(0.0),
	_mocap_eph(0.0),
	_mocap_epv(0.0),

	// local to global coversion related variables
	_is_global_cov_init(false),
	_ref_lat(0.0),
	_ref_lon(0.0),
	_ref_alt(0.0)
{
	// _sensors_sub.set_interval_ms(10); // main prediction loop, 100 hz (lockstep requires to run at full rate)
    _param_lpe_fusion.set(1);
    _param_lpe_vxy_pub.set(0.3);
    _param_lpe_z_pub.set(1.0);
    _param_lpe_snr_z.set(0.05);
    _param_lpe_snr_off_z.set(0.000);
    _param_lpe_ldr_z.set(0.3);
    _param_lpe_ldr_off_z.set(0.000);
    _param_lpe_acc_xy.set(0.12f);       // default: acc xy 0.012
    _param_lpe_acc_z.set(0.02f);         // default: acc z 0.02
    _param_lpe_bar_z.set(3.0);
    _param_lpe_gps_delay.set(0.20f);
    _param_lpe_gps_xy.set(1.0);             // default : 1.0f
    _param_lpe_gps_z.set(3.0);              // default : 3.0f
    _param_lpe_gps_vxy.set(0.25);           // default : 0.25f
    _param_lpe_gps_vz.set(0.25);            // default : 0.25f
    _param_lpe_eph_max.set(3.0);
    _param_lpe_epv_max.set(5.0);
    _param_lpe_vis_xy.set(0.1);
    _param_lpe_vis_z.set(0.5);
    _param_lpe_vis_delay.set(0.1);
    _param_lpe_vic_p.set(0.001);
    _param_lpe_flw_off_z.set(0.0);
    _param_lpe_flw_scale.set(1.3);
    _param_lpe_flw_qmin.set(150);
    _param_lpe_flw_r.set(7.0);
    _param_lpe_flw_rr.set(7.0);
    _param_lpe_land_z.set(0.03);
    _param_lpe_land_vxy.set(0.05);
    _param_lpe_pn_p.set(0.1);
    _param_lpe_pn_v.set(0.1);
    _param_lpe_pn_b.set(1e-3);
    _param_lpe_pn_t.set(0.001);
    _param_lpe_t_max_grade.set(1.0);
    _param_lpe_lt_cov.set(0.0001);
    _param_ltest_mode.set(0.0);
    _param_lpe_fake_origin.set(0);
    _param_lpe_lat.set(29.450276);
    _param_lpe_lon.set(106.52031);
    
    _flow_gyro_x_high_pass.setParamFCut(0.001f);
    _flow_gyro_y_high_pass.setParamFCut(0.001f);
    _xLowPass.setParamFCut(5.0f);
    _aglLowPass.setParamFCut(5.0f);
    
    uorb_subscribe_topics<uORB::TOPIC_SENSOR_COMBINED>(&_sub_sensors);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_COMMAND>(&_sub_vehicle_command);
    uorb_subscribe_topics<uORB::TOPIC_ACTUATOR_ARMED>(&_sub_armed);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_LAND_DETECTED>(&_sub_land);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_ATTITUDE>(&_sub_att);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_ANGULAR_VELOCITY>(&_sub_angular_velocity);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_OPTICAL_FLOW>(&_sub_flow);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_GPS_POSITION>(&_sub_gps);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_ODOMETRY>(&_sub_visual_odom);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_ODOMETRY>(&_sub_mocap_odom);
    uorb_subscribe_topics<uORB::TOPIC_LANDING_TARGET_POSE>(&_sub_landing_target_pose);
    uorb_subscribe_topics<uORB::TOPIC_VEHICLE_AIR_DATA>(&_sub_airdata);
    uorb_subscribe_topics<uORB::TOPIC_DISTANCE_SENSOR>(&_sub_dist0);
    uorb_subscribe_topics<uORB::TOPIC_DISTANCE_SENSOR>(&_sub_dist1);
    uorb_subscribe_topics<uORB::TOPIC_DISTANCE_SENSOR>(&_sub_dist2);
    uorb_subscribe_topics<uORB::TOPIC_DISTANCE_SENSOR>(&_sub_dist3);

    _sub_lidar = nullptr;
    _sub_sonar = nullptr;
    
	_dist_subs[0] = _sub_dist0;
	_dist_subs[1] = _sub_dist1;
	_dist_subs[2] = _sub_dist2;
	_dist_subs[3] = _sub_dist3;
    
	// initialize A, B,  P, x, u
	_x.setZero();
	_u.setZero();
	initSS();
}

bool
BlockLocalPositionEstimator::init()
{

	return true;
}

Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::dynamics(
	float t,
	const Vector<float, BlockLocalPositionEstimator::n_x> &x,
	const Vector<float, BlockLocalPositionEstimator::n_u> &u)
{
	return m_A * x + m_B * u;
}

void BlockLocalPositionEstimator::Run()
{
    if (_sub_vehicle_command->command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
        const double latitude = _sub_vehicle_command->param5;
        const double longitude = _sub_vehicle_command->param6;
        const float altitude = _sub_vehicle_command->param7;
        _global_local_proj_ref.initReference(latitude, longitude, _sub_vehicle_command->timestamp);
        _global_local_alt0 = altitude;
        
        GCS_SEND_TEXT(MAVLINK_COMM_1, MAV_SEVERITY_INFO, "New NED origin (LLA) setted\n");
    }

    sensor_combined_s imu = *_sub_sensors;
    imu.accelerometer_m_s2[1] *= -1;
    imu.accelerometer_m_s2[2] *= -1;
    imu.gyro_rad[1] *= -1;
    imu.gyro_rad[2] *= -1;

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;
    
	// auto-detect connected rangefinders while not armed
	bool armedState = _sub_armed->armed;


	// reset pos, vel, and terrain on arming

	// XXX this will be re-enabled for indoor use cases using a
	// selection param, but is really not helping outdoors
	// right now.

	// if (!_lastArmedState && armedState) {

	//      // we just armed, we are at origin on the ground
	//      _x(X_x) = 0;
	//      _x(X_y) = 0;
	//      // reset Z or not? _x(X_z) = 0;

	//      // we aren't moving, all velocities are zero
	//      _x(X_vx) = 0;
	//      _x(X_vy) = 0;
	//      _x(X_vz) = 0;

	//      // assume we are on the ground, so terrain alt is local alt
	//      _x(X_tz) = _x(X_z);

	//      // reset lowpass filter as well
	//      _xLowPass.setState(_x);
	//      _aglLowPass.setState(0);
	// }

	_lastArmedState = armedState;

	// see which updates are available
	// const bool paramsUpdated = _parameter_update_sub.updated();
	_baroUpdated = false;

	if ((_param_lpe_fusion.get() & FUSE_BARO)) {
		if (_sub_airdata->timestamp != _timeStampLastBaro) {
			_baroUpdated = true;
			_timeStampLastBaro = _sub_airdata->timestamp;
		}
	}

	_flowUpdated = (_param_lpe_fusion.get() & FUSE_FLOW);
	_gpsUpdated = (_param_lpe_fusion.get() & FUSE_GPS);
	_visionUpdated = (_param_lpe_fusion.get() & FUSE_VIS_POS);
	_mocapUpdated = false;
	_lidarUpdated = (_sub_lidar != nullptr);
	_sonarUpdated = (_sub_sonar != nullptr);
	_landUpdated = landed() && ((_timeStamp - _time_last_land) > 1.0e6f / LAND_RATE);// throttle rate
	bool targetPositionUpdated = false; //_sub_landing_target_pose.update();

	// is xy valid?
	bool vxy_stddev_ok = false;

	if (math::max(m_P(X_vx, X_vx), m_P(X_vy, X_vy)) < _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get()) {
		vxy_stddev_ok = true;
	}

	if (_estimatorInitialized & EST_XY) {
		// if valid and gps has timed out, set to not valid
		if (!vxy_stddev_ok && (_sensorTimeout & SENSOR_GPS)) {
			_estimatorInitialized &= ~EST_XY;
            mavlink_log_info(MAVLINK_COMM_1, "xy flag reset");
		}

	} else {
		if (vxy_stddev_ok) {
			if (!(_sensorTimeout & SENSOR_GPS)
			    || !(_sensorTimeout & SENSOR_FLOW)
			    || !(_sensorTimeout & SENSOR_VISION)
			    || !(_sensorTimeout & SENSOR_MOCAP)
			    || !(_sensorTimeout & SENSOR_LAND)
			    || !(_sensorTimeout & SENSOR_LAND_TARGET)
			   ) {
				_estimatorInitialized |= EST_XY;
                mavlink_log_info(MAVLINK_COMM_1, "xy flag set");
			}
		}
	}

	// is z valid?
	bool z_stddev_ok = sqrtf(m_P(X_z, X_z)) < _param_lpe_z_pub.get();

	if (_estimatorInitialized & EST_Z) {
		// if valid and baro has timed out, set to not valid
		if (!z_stddev_ok && (_sensorTimeout & SENSOR_BARO)) {
			_estimatorInitialized &= ~EST_Z;
            mavlink_log_info(MAVLINK_COMM_1, "z flag reset");
		}

	} else {
		if (z_stddev_ok) {
			_estimatorInitialized |= EST_Z;
            mavlink_log_info(MAVLINK_COMM_1, "z flag set");
		}
	}

	// is terrain valid?
	bool tz_stddev_ok = sqrtf(m_P(X_tz, X_tz)) < _param_lpe_z_pub.get();

	if (_estimatorInitialized & EST_TZ) {
		if (!tz_stddev_ok) {
			_estimatorInitialized &= ~EST_TZ;
            mavlink_log_info(MAVLINK_COMM_1, "tz flag reset");
		}

	} else {
		if (tz_stddev_ok) {
			_estimatorInitialized |= EST_TZ;
            mavlink_log_info(MAVLINK_COMM_1, "tz flag set");
		}
	}

	// check timeouts
	checkTimeouts();

	// if we have no lat, lon initialize projection to LPE_LAT, LPE_LON parameters
	if (!_map_ref.isInitialized() && (_estimatorInitialized & EST_XY) && _param_lpe_fake_origin.get()) {
		_map_ref.initReference(
			(double)_param_lpe_lat.get(),
			(double)_param_lpe_lon.get(), hrt_absolute_time());

		// set timestamp when origin was set to current time
		_time_origin = _timeStamp;
            
		mavlink_log_info(MAVLINK_COMM_1, "[lpe] global origin init (parameter) : lat %6.2f lon %6.2f alt %5.1f m",
				 double(_param_lpe_lat.get()), double(_param_lpe_lon.get()), double(_altOrigin));
	}

	// reinitialize x if necessary
	bool reinit_x = false;

	for (size_t i = 0; i < n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!PX4_ISFINITE(_x(i))) {
			reinit_x = true;
            mavlink_log_info(MAVLINK_COMM_1, "%sreinit x, x(%zu) not finite", msg_label, i);
			break;
		}
	}

	if (reinit_x) {
		for (size_t i = 0; i < n_x; i++) {
			_x(i) = 0;
		}
        
        mavlink_log_info(MAVLINK_COMM_1, "%sreinit x", msg_label);
	}

	// force P symmetry and reinitialize P if necessary
	bool reinit_P = false;

	for (size_t i = 0; i < n_x; i++) {
		for (size_t j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(m_P(i, j))) {
				mavlink_log_info(MAVLINK_COMM_1,
						 "%sreinit P (%zu, %zu) not finite", msg_label, i, j);
				reinit_P = true;
			}

			if (i == j) {
				// make sure diagonal elements are positive
				if (m_P(i, i) <= 0) {
					mavlink_log_info(MAVLINK_COMM_1,
							 "%sreinit P (%zu, %zu) negative", msg_label, i, j);
					reinit_P = true;
				}

			} else {
				// copy elememnt from upper triangle to force
				// symmetry
				m_P(j, i) = m_P(i, j);
			}

			if (reinit_P) { break; }
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		initP();
	}

	// do prediction
//    imu.accelerometer_m_s2[0] = imu.accelerometer_m_s2[0] -  _x(X_bx);
//    imu.accelerometer_m_s2[1] = imu.accelerometer_m_s2[1] -  _x(X_by);
//    imu.accelerometer_m_s2[2] = imu.accelerometer_m_s2[2] -  _x(X_bz);
	predict(imu);

	// sensor corrections/ initializations
	if (_gpsUpdated) {
		if (_sensorTimeout & SENSOR_GPS) {
			gpsInit();

		} else {
            //EDEBUG("%.3f, %.3f\n", _x(X_vx), _sub_gps->vel_n_m_s);
			gpsCorrect();
		}
	}

	if (_baroUpdated) {
		if (_sensorTimeout & SENSOR_BARO) {
			baroInit();

		} else {
			baroCorrect();
		}
	}

	if (_lidarUpdated) {
		if (_sensorTimeout & SENSOR_LIDAR) {
			lidarInit();

		} else {
			lidarCorrect();
		}
	}

	if (_sonarUpdated) {
		if (_sensorTimeout & SENSOR_SONAR) {
			sonarInit();

		} else {
			sonarCorrect();
		}
	}

	if (_flowUpdated) {
		if (_sensorTimeout & SENSOR_FLOW) {
			flowInit();

		} else {
			flowCorrect();
		}
	}

	if (_visionUpdated) {
		if (_sensorTimeout & SENSOR_VISION) {
			visionInit();

		} else {
			visionCorrect();
		}
	}

	if (_mocapUpdated) {
		if (_sensorTimeout & SENSOR_MOCAP) {
			mocapInit();

		} else {
			mocapCorrect();
		}
	}

	if (_landUpdated) {
		if (_sensorTimeout & SENSOR_LAND) {
			landInit();

		} else {
			landCorrect();
		}
	}

	if (targetPositionUpdated) {
		if (_sensorTimeout & SENSOR_LAND_TARGET) {
			landingTargetInit();

		} else {
			landingTargetCorrect();
		}
	}

	if (_altOriginInitialized) {
		// update all publications if possible
		publishLocalPos();
		publishOdom();
		publishEstimatorStatus();

		_pub_innov.timestamp_sample = _timeStamp;
		_pub_innov.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_ESTIMATOR_INNOVATIONS>(&_pub_innov);
        
		_pub_innov_var.timestamp_sample = _timeStamp;
		_pub_innov_var.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_ESTIMATOR_INNOVATIONS>(&_pub_innov_var);

		if ((_estimatorInitialized & EST_XY) && (_map_ref.isInitialized() || _param_lpe_fake_origin.get())) {
			publishGlobalPos();
		}
	}

	// propagate delayed state, no matter what
	// if state is frozen, delayed state still
	// needs to be propagated with frozen state
	float dt_hist = 1.0e-6f * (_timeStamp - _time_last_hist);

	if (_time_last_hist == 0 ||
	    (dt_hist >= HIST_STEP)) {
		_tDelay.update(Scalar<uint64_t>(_timeStamp));
		_xDelay.update(_x);
		_time_last_hist = _timeStamp;
	}
}

void BlockLocalPositionEstimator::checkTimeouts()
{
	baroCheckTimeout();
	gpsCheckTimeout();
	lidarCheckTimeout();
	flowCheckTimeout();
	sonarCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
	landCheckTimeout();
	landingTargetCheckTimeout();
}

bool BlockLocalPositionEstimator::landed()
{
	if (!(_param_lpe_fusion.get() & FUSE_LAND)) {
		return false;
	}

	// _sub_land->update();

	bool disarmed_not_falling = (!_sub_armed->armed) && (!_sub_land->freefall);

	return _sub_land->landed || disarmed_not_falling;
}

void BlockLocalPositionEstimator::publishLocalPos()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(m_P(X_vx, X_vx) + m_P(X_vy, X_vy));
	float evv = sqrtf(m_P(X_vz, X_vz));
	float eph = sqrtf(m_P(X_x, X_x) + m_P(X_y, X_y));
	float epv = sqrtf(m_P(X_z, X_z));

	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (evh < _param_lpe_vxy_pub.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	// publish local position
	if (Vector3f(_x(X_x), _x(X_y), _x(X_z)).isAllFinite() &&
	    Vector3f(_x(X_vx), _x(X_vy), _x(X_vz)).isAllFinite()) {
		_pub_lpos.timestamp_sample = _timeStamp;

		_pub_lpos.xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.z_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.v_xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.v_z_valid = _estimatorInitialized & EST_Z;

		_pub_lpos.x = xLP(X_x);	// north
		_pub_lpos.y = xLP(X_y);	// east

		if (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) {
			_pub_lpos.z = -_aglLowPass.getState();	// agl

		} else {
			_pub_lpos.z = xLP(X_z);	// down
		}

		const float heading = matrix::Eulerf(matrix::Quatf(_sub_att->q)).psi();
		_pub_lpos.heading = heading;
		_pub_lpos.heading_good_for_control = PX4_ISFINITE(heading);
		_pub_lpos.unaided_heading = NAN;

		_pub_lpos.vx = xLP(X_vx);		// north
		_pub_lpos.vy = xLP(X_vy);		// east
		_pub_lpos.vz = xLP(X_vz);		// down

		// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		_pub_lpos.z_deriv = xLP(X_vz);

		_pub_lpos.ax = _u(U_ax);		// north
		_pub_lpos.ay = _u(U_ay);		// east
		_pub_lpos.az = _u(U_az);		// down

		_pub_lpos.xy_global = _estimatorInitialized & EST_XY;
		_pub_lpos.z_global = !(_sensorTimeout & SENSOR_BARO) && _altOriginGlobal;
		_pub_lpos.ref_timestamp = _time_origin;
		_pub_lpos.ref_lat = _map_ref.getProjectionReferenceLat();
		_pub_lpos.ref_lon = _map_ref.getProjectionReferenceLon();
		_pub_lpos.ref_alt = _altOrigin;
		_pub_lpos.dist_bottom = _aglLowPass.getState();
		// we estimate agl even when we don't have terrain info
		// if you are in terrain following mode this is important
		// so that if terrain estimation fails there isn't a
		// sudden altitude jump
		_pub_lpos.dist_bottom_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.eph = eph;
		_pub_lpos.epv = epv;
		_pub_lpos.evh = evh;
		_pub_lpos.evv = evv;
		_pub_lpos.vxy_max = INFINITY;
		_pub_lpos.vz_max = INFINITY;
		_pub_lpos.hagl_min = INFINITY;
		_pub_lpos.hagl_max = INFINITY;
		_pub_lpos.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_VEHICLE_LOCAL_POSITION>(&_pub_lpos);
	}
}

void BlockLocalPositionEstimator::publishOdom()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// publish vehicle odometry
	if (Vector3f(_x(X_x), _x(X_y), _x(X_z)).isAllFinite() &&
	    Vector3f(_x(X_vx), _x(X_vy), _x(X_vz)).isAllFinite()) {
		_pub_odom.timestamp_sample = _timeStamp;
		_pub_odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;

		// position
		_pub_odom.position[0] = xLP(X_x);	// north
		_pub_odom.position[1] = xLP(X_y);	// east

		if (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) {
			_pub_odom.position[2] = -_aglLowPass.getState();	// agl

		} else {
			_pub_odom.position[2] = xLP(X_z);	// down
		}

		// orientation
		matrix::Quatf q = matrix::Quatf(_sub_att->q);
		q.copyTo(_pub_odom.q);

		// linear velocity
		_pub_odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
		_pub_odom.velocity[0] = xLP(X_vx);		// vel north
		_pub_odom.velocity[1] = xLP(X_vy);		// vel east
		_pub_odom.velocity[2] = xLP(X_vz);		// vel down

		// angular velocity
		_pub_odom.angular_velocity[0] = NAN;
		_pub_odom.angular_velocity[1] = NAN;
		_pub_odom.angular_velocity[2] = NAN;

		// get the covariance matrix size
		const size_t POS_URT_SIZE = sizeof(_pub_odom.position_variance) / sizeof(_pub_odom.position_variance[0]);
		const size_t VEL_URT_SIZE = sizeof(_pub_odom.velocity_variance) / sizeof(_pub_odom.velocity_variance[0]);

		// initially set pose covariances to 0
		for (size_t i = 0; i < POS_URT_SIZE; i++) {
			_pub_odom.position_variance[i] = NAN;
		}

		// set the position variances
		_pub_odom.position_variance[0] = m_P(X_vx, X_vx);
		_pub_odom.position_variance[1] = m_P(X_vy, X_vy);
		_pub_odom.position_variance[2] = m_P(X_vz, X_vz);

		// unknown orientation covariances
		// TODO: add orientation covariance to vehicle_attitude
		_pub_odom.orientation_variance[0] = NAN;
		_pub_odom.orientation_variance[1] = NAN;
		_pub_odom.orientation_variance[2] = NAN;

		// initially set velocity covariances to 0
		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			_pub_odom.velocity_variance[i] = NAN;
		}

		// set the linear velocity variances
		_pub_odom.velocity_variance[0] = m_P(X_vx, X_vx);
		_pub_odom.velocity_variance[1] = m_P(X_vy, X_vy);
		_pub_odom.velocity_variance[2] = m_P(X_vz, X_vz);

		_pub_odom.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_VEHICLE_ODOMETRY>(&_pub_odom);
	}
}

void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	_pub_est_states.timestamp_sample = _timeStamp;

	for (size_t i = 0; i < n_x; i++) {
		_pub_est_states.states[i] = _x(i);
	}

	// matching EKF2 covariances indexing
	// quaternion - not determined, as it is a position estimator
	_pub_est_states.covariances[0] = NAN;
	_pub_est_states.covariances[1] = NAN;
	_pub_est_states.covariances[2] = NAN;
	_pub_est_states.covariances[3] = NAN;
	// linear velocity
	_pub_est_states.covariances[4] = m_P(X_vx, X_vx);
	_pub_est_states.covariances[5] = m_P(X_vy, X_vy);
	_pub_est_states.covariances[6] = m_P(X_vz, X_vz);
	// position
	_pub_est_states.covariances[7] = m_P(X_x, X_x);
	_pub_est_states.covariances[8] = m_P(X_y, X_y);
	_pub_est_states.covariances[9] = m_P(X_z, X_z);
	// gyro bias - not determined
	_pub_est_states.covariances[10] = NAN;
	_pub_est_states.covariances[11] = NAN;
	_pub_est_states.covariances[12] = NAN;
	// accel bias
	_pub_est_states.covariances[13] = m_P(X_bx, X_bx);
	_pub_est_states.covariances[14] = m_P(X_by, X_by);
	_pub_est_states.covariances[15] = m_P(X_bz, X_bz);

	// mag - not determined
	for (size_t i = 16; i <= 21; i++) {
		_pub_est_states.covariances[i] = NAN;
	}

	// replacing the hor wind cov with terrain altitude covariance
	_pub_est_states.covariances[22] = m_P(X_tz, X_tz);

	_pub_est_states.n_states = n_x;
	_pub_est_states.timestamp = hrt_absolute_time();
    uorb_publish_topics<uORB::TOPIC_ESTIMATOR_STATES>(&_pub_est_states);
    
	// estimator_status
	_pub_est_status.timestamp_sample = _timeStamp;
	_pub_est_status.health_flags = _sensorFault;
	_pub_est_status.timeout_flags = _sensorTimeout;
	_pub_est_status.pos_horiz_accuracy = _pub_gpos.eph;
	_pub_est_status.pos_vert_accuracy = _pub_gpos.epv;

	_pub_est_status.timestamp = hrt_absolute_time();
	uorb_publish_topics<uORB::TOPIC_ESTIMATOR_STATUS>(&_pub_est_status);
}

void BlockLocalPositionEstimator::publishGlobalPos()
{
	// publish global position
	double lat = 0;
	double lon = 0;
	const Vector<float, n_x> &xLP = _xLowPass.getState();
	_map_ref.reproject(xLP(X_x), xLP(X_y), lat, lon);
	float alt = -xLP(X_z) + _altOrigin;
    
	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(m_P(X_vx, X_vx) + m_P(X_vy, X_vy));
	float eph = sqrtf(m_P(X_x, X_x) + m_P(X_y, X_y));
	float epv = sqrtf(m_P(X_z, X_z));

	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (evh < _param_lpe_vxy_pub.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) &&
	    Vector3f(xLP(X_vx), xLP(X_vy), xLP(X_vz)).isAllFinite()) {
		_pub_gpos.timestamp_sample = _timeStamp;
		_pub_gpos.lat = lat;
		_pub_gpos.lon = lon;
		_pub_gpos.alt = alt;
		_pub_gpos.eph = eph;
		_pub_gpos.epv = epv;
		_pub_gpos.terrain_alt = _altOrigin - xLP(X_tz);
		_pub_gpos.terrain_alt_valid = _estimatorInitialized & EST_TZ;
		_pub_gpos.dead_reckoning = !(_estimatorInitialized & EST_XY);
		_pub_gpos.timestamp = hrt_absolute_time();
        uorb_publish_topics<uORB::TOPIC_VEHICLE_GLOBAL_POSITION>(&_pub_gpos);
	}
}

void BlockLocalPositionEstimator::initP()
{
	m_P.setZero();
	// initialize to twice valid condition
	m_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	m_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	m_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	m_P(X_vx, X_vx) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	m_P(X_vy, X_vy) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	// use vxy thresh for vz init as well
	m_P(X_vz, X_vz) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	// initialize bias uncertainty to small values to keep them stable
	m_P(X_bx, X_bx) = 1e-6;
	m_P(X_by, X_by) = 1e-6;
	m_P(X_bz, X_bz) = 1e-6;
	m_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
}

void BlockLocalPositionEstimator::initSS()
{
	initP();

	// dynamics matrix
	m_A.setZero();
	// derivative of position is velocity
	m_A(X_x, X_vx) = 1;
	m_A(X_y, X_vy) = 1;
	m_A(X_z, X_vz) = 1;

	// input matrix
	m_B.setZero();
	m_B(X_vx, U_ax) = 1;
	m_B(X_vy, U_ay) = 1;
	m_B(X_vz, U_az) = 1;

	// update components that depend on current state
	updateSSStates();
	updateSSParams();
}

void BlockLocalPositionEstimator::updateSSStates()
{
	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	m_A(X_vx, X_bx) = -_R_att(0, 0);
	m_A(X_vx, X_by) = -_R_att(0, 1);
	m_A(X_vx, X_bz) = -_R_att(0, 2);

	m_A(X_vy, X_bx) = -_R_att(1, 0);
	m_A(X_vy, X_by) = -_R_att(1, 1);
	m_A(X_vy, X_bz) = -_R_att(1, 2);

	m_A(X_vz, X_bx) = -_R_att(2, 0);
	m_A(X_vz, X_by) = -_R_att(2, 1);
	m_A(X_vz, X_bz) = -_R_att(2, 2);
}

void BlockLocalPositionEstimator::updateSSParams()
{
	// input noise covariance matrix
	m_R.setZero();
	m_R(U_ax, U_ax) = _param_lpe_acc_xy.get() * _param_lpe_acc_xy.get();
	m_R(U_ay, U_ay) = _param_lpe_acc_xy.get() * _param_lpe_acc_xy.get();
	m_R(U_az, U_az) = _param_lpe_acc_z.get() * _param_lpe_acc_z.get();

	// process noise power matrix
	m_Q.setZero();
	float pn_p_sq = _param_lpe_pn_p.get() * _param_lpe_pn_p.get();
	float pn_v_sq = _param_lpe_pn_v.get() * _param_lpe_pn_v.get();
	m_Q(X_x, X_x) = pn_p_sq;
	m_Q(X_y, X_y) = pn_p_sq;
	m_Q(X_z, X_z) = pn_p_sq;
	m_Q(X_vx, X_vx) = pn_v_sq;
	m_Q(X_vy, X_vy) = pn_v_sq;
	m_Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	float pn_b_sq = _param_lpe_pn_b.get() * _param_lpe_pn_b.get();
	m_Q(X_bx, X_bx) = pn_b_sq;
	m_Q(X_by, X_by) = pn_b_sq;
	m_Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	float pn_t_noise_density =
		_param_lpe_pn_t.get() +
		(_param_lpe_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
	m_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
}

void BlockLocalPositionEstimator::predict(const sensor_combined_s &imu)
{
	// get acceleration
	_R_att = matrix::Dcm<float>(matrix::Quatf(_sub_att->q));
	Vector3f a(imu.accelerometer_m_s2);
	// note, bias is removed in dynamics function
	_u = _R_att * a;
	_u(U_az) += CONSTANTS_ONE_G;	// add g
    
	// update state space based on new states
	updateSSStates();

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	// float h = getDt();
    
	hrt_abstime now = hrt_absolute_time();
	const float h = (now - _prev_timestamp) / 1e6f;
	_prev_timestamp = now;

	Vector<float, n_x> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);
    // Vector<float, n_x> dx = dynamics(h, _x, _u) * h;
	// don't integrate position if no valid xy data
	if (!(_estimatorInitialized & EST_XY))  {
		dx(X_x) = 0;
		dx(X_vx) = 0;
		dx(X_y) = 0;
		dx(X_vy) = 0;
	}

	// don't integrate z if no valid z data
	if (!(_estimatorInitialized & EST_Z))  {
		dx(X_z) = 0;
	}

	// don't integrate tz if no valid tz data
	if (!(_estimatorInitialized & EST_TZ))  {
		dx(X_tz) = 0;
	}
    
	// saturate bias
	float bx = dx(X_bx) + _x(X_bx);
	float by = dx(X_by) + _x(X_by);
	float bz = dx(X_bz) + _x(X_bz);

	if (std::abs(bx) > BIAS_MAX) {
		bx = BIAS_MAX * bx / std::abs(bx);
		dx(X_bx) = bx - _x(X_bx);
	}

	if (std::abs(by) > BIAS_MAX) {
		by = BIAS_MAX * by / std::abs(by);
		dx(X_by) = by - _x(X_by);
	}

	if (std::abs(bz) > BIAS_MAX) {
		bz = BIAS_MAX * bz / std::abs(bz);
		dx(X_bz) = bz - _x(X_bz);
	}

	// propagate
	_x += dx;
	Matrix<float, n_x, n_x> dP = (m_A * m_P + m_P * m_A.transpose() +
				      m_B * m_R * m_B.transpose() + m_Q) * h;
	// covariance propagation logic
	for (size_t i = 0; i < n_x; i++) {
		if (m_P(i, i) > P_MAX) {
			// if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for (size_t j = 0; j < n_x; j++) {
				dP(i, j) = 0;
				dP(j, i) = 0;
			}
		}
	}

	m_P += dP;
    _xLowPass.setDt(h);
    _aglLowPass.setDt(h);
	_xLowPass.update(_x);
	_aglLowPass.update(agl());
}

int BlockLocalPositionEstimator::getDelayPeriods(float delay, uint8_t *periods)
{
	float t_delay = 0;
	uint8_t i_hist = 0;

	for (i_hist = 1; i_hist < HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > delay) {
			break;
		}
	}

	*periods = i_hist;

	if (t_delay > DELAY_MAX) {
		mavlink_log_info(MAVLINK_COMM_1, "%sdelayed data old: %8.4f", msg_label, double(t_delay));
		return -1;
	}

	return OK;
}
