#pragma once

#include <float.h>
#include "platform_defines.h"

#include <ecl/EKF/ekf.h>
#include <ecl/geo/geo_old.h>
#include <mathlib/mathlib.h>
#include "Utility/PreFlightChecker.hpp"

#include "param/parameters.hpp"
#include "uORB_topic_define.hpp"
//#include <uORB/topics/airspeed.h>
//#include <uORB/topics/distance_sensor.h>
//#include <uORB/topics/ekf2_timestamps.h>
//#include <uORB/topics/ekf_gps_drift.h>
//#include <uORB/topics/ekf_gps_position.h>
//#include <uORB/topics/estimator_innovations.h>
//#include <uORB/topics/estimator_sensor_bias.h>
//#include <uORB/topics/estimator_status.h>
//#include <uORB/topics/landing_target_pose.h>
//#include <uORB/topics/optical_flow.h>
//#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/sensor_selection.h>
//#include <uORB/topics/vehicle_air_data.h>
//#include <uORB/topics/vehicle_attitude.h>
//#include <uORB/topics/vehicle_global_position.h>
//#include <uORB/topics/vehicle_gps_position.h>
//#include <uORB/topics/vehicle_imu.h>
//#include <uORB/topics/vehicle_land_detected.h>
//#include <uORB/topics/vehicle_local_position.h>
//#include <uORB/topics/vehicle_magnetometer.h>
//#include <uORB/topics/vehicle_odometry.h>
//#include <uORB/topics/vehicle_status.h>
//#include <uORB/topics/wind_estimate.h>
//#include <uORB/topics/yaw_estimator_status.h>

// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
#define BLEND_MASK_USE_SPD_ACC      1
#define BLEND_MASK_USE_HPOS_ACC     2
#define BLEND_MASK_USE_VPOS_ACC     4

// define max number of GPS receivers supported and 0 base instance used to access virtual 'blended' GPS solution
#define GPS_MAX_RECEIVERS 2
#define GPS_BLENDED_INSTANCE 2


class Ekf2 final
{
public:
	explicit Ekf2(bool replay_mode = false);
	~Ekf2() = default;

	bool init();

    void Run();

private:

	int getRangeSubIndex(); ///< get subscription index of first downward-facing range sensor
	void fillGpsMsgWithVehicleGpsPosData(gps_message &msg, const vehicle_gps_position_s &data);

	PreFlightChecker _preflt_checker;
	void runPreFlightChecks(float dt, const filter_control_status_u &control_status,
				const vehicle_status_s &vehicle_status,
				const estimator_innovations_s &innov);
	void resetPreFlightChecks();

	template<typename Param>
	void update_mag_bias(Param &mag_bias_param, int axis_index);

	template<typename Param>
	bool update_mag_decl(Param &mag_decl_param);

	void publish_attitude(const hrt_abstime &timestamp);
	void publish_wind_estimate(const hrt_abstime &timestamp);
	void publish_yaw_estimator_status(const hrt_abstime &timestamp);

	/*
	 * Update the internal state estimate for a blended GPS solution that is a weighted average of the phsyical
	 * receiver solutions. This internal state cannot be used directly by estimators because if physical receivers
	 * have significant position differences, variation in receiver estimated accuracy will cause undesirable
	 * variation in the position solution.
	*/
	bool blend_gps_data();

	/*
	 * Calculate internal states used to blend GPS data from multiple receivers using weightings calculated
	 * by calc_blend_weights()
	 * States are written to _gps_state and _gps_blended_state class variables
	 */
	void update_gps_blend_states();

	/*
	 * The location in _gps_blended_state will move around as the relative accuracy changes.
	 * To mitigate this effect a low-pass filtered offset from each GPS location to the blended location is
	 * calculated.
	*/
	void update_gps_offsets();

	/*
	 * Apply the steady state physical receiver offsets calculated by update_gps_offsets().
	*/
	void apply_gps_offsets();

	/*
	 Calculate GPS output that is a blend of the offset corrected physical receiver data
	*/
	void calc_gps_blend_output();

	/*
	 * Calculate filtered WGS84 height from estimated AMSL height
	 */
	float filter_altitude_ellipsoid(float amsl_hgt);

	inline float sq(float x) { return x * x; };

	const bool 	_replay_mode;			///< true when we use replay data from a log

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)
    
	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint8_t _invalid_mag_id_count = 0;	///< number of times an invalid magnetomer device ID has been detected

	// Used to check, save and use learned magnetometer biases
	hrt_abstime _last_magcal_us = 0;	///< last time the EKF was operating a mode that estimates magnetomer biases (uSec)
	hrt_abstime _total_cal_time_us = 0;	///< accumulated calibration time since the last save

	float _last_valid_mag_cal[3] = {};	///< last valid XYZ magnetometer bias estimates (mGauss)
	bool _valid_cal_available[3] = {};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
	float _last_valid_variance[3] = {};	///< variances for the last valid magnetometer XYZ bias estimates (mGauss**2)

	// Used to control saving of mag declination to be used on next startup
	bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

	// set pose/velocity as invalid if standard deviation is bigger than max_std_dev
	// TODO: the user should be allowed to set these values by a parameter
	static constexpr float ep_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated position
	static constexpr float eo_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated orientation
	//static constexpr float ev_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated velocity

	// GPS blending and switching
	gps_message _gps_state[GPS_MAX_RECEIVERS] {}; ///< internal state data for the physical GPS
	gps_message _gps_blended_state{};		///< internal state data for the blended GPS
	gps_message _gps_output[GPS_MAX_RECEIVERS + 1] {}; ///< output state data for the physical and blended GPS
	Vector2f _NE_pos_offset_m[GPS_MAX_RECEIVERS] = {}; ///< Filtered North,East position offset from GPS instance to blended solution in _output_state.location (m)
	float _hgt_offset_mm[GPS_MAX_RECEIVERS] = {};	///< Filtered height offset from GPS instance relative to blended solution in _output_state.location (mm)
	Vector3f _blended_antenna_offset = {};		///< blended antenna offset
	float _blend_weights[GPS_MAX_RECEIVERS] = {};	///< blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
	uint64_t _time_prev_us[GPS_MAX_RECEIVERS] = {};	///< the previous value of time_us for that GPS instance - used to detect new data.
	uint8_t _gps_best_index = 0;			///< index of the physical receiver with the lowest reported error
	uint8_t _gps_select_index = 0;			///< 0 = GPS1, 1 = GPS2, 2 = blended
	uint8_t _gps_time_ref_index =
		0;		///< index of the receiver that is used as the timing reference for the blending update
	uint8_t _gps_oldest_index = 0;			///< index of the physical receiver with the oldest data
	uint8_t _gps_newest_index = 0;			///< index of the physical receiver with the newest data
	uint8_t _gps_slowest_index = 0;			///< index of the physical receiver with the slowest update rate
	float _gps_dt[GPS_MAX_RECEIVERS] = {};		///< average time step in seconds.
	bool  _gps_new_output_data = false;		///< true if there is new output data for the EKF
	bool _had_valid_terrain = false;		///< true if at any time there was a valid terrain estimate

	int32_t _gps_alttitude_ellipsoid[GPS_MAX_RECEIVERS] {};	///< altitude in 1E-3 meters (millimeters) above ellipsoid
	uint64_t _gps_alttitude_ellipsoid_previous_timestamp[GPS_MAX_RECEIVERS] {}; ///< storage for previous timestamp to compute dt
	float   _wgs84_hgt_offset = 0;  ///< height offset between AMSL and WGS84

	bool _imu_bias_reset_request{false};

	// republished aligned external visual odometry
	bool new_ev_data_received = false;
	vehicle_odometry_s _ev_odom{};

    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_AIR_DATA>::type *_airdata_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_AIRSPEED>::type *_airspeed_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ODOMETRY>::type *_ev_odom_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_LANDING_TARGET_POSE>::type *_landing_target_pose_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_MAGNETOMETER>::type *_magnetometer_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_OPTICAL_FLOW>::type *_optical_flow_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_PARAMETER_UPDATE>::type *_parameter_update_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_SELECTION>::type *_sensor_selection_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_STATUS>::type *_status_sub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_LAND_DETECTED>::type *_vehicle_land_detected_sub;
    
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_COMBINED>::type *_sensor_combined_sub;
    
    hrt_abstime     _sensor_combined_timestamp_sample_last{0};
    hrt_abstime     _vehicle_air_data_timestamp_sample_last{0};
    hrt_abstime     _airspeed_timestamp_sample_last{0};
    hrt_abstime     _vehicle_odometry_timestamp_sample_last{0};
    hrt_abstime     _landing_target_pose_timestamp_sample_last{0};
    hrt_abstime     _vehicle_magnetometer_timestamp_sample_last{0};
    hrt_abstime     _optical_flow_timestamp_sample_last{0};
    hrt_abstime     _vehicle_land_detected_timestamp_sample_last{0};
    hrt_abstime     _vehicle_gps_position_timestamp_sample_last{0};
    hrt_abstime     _distance_sensor_timestamp_sample_last{0};
    
	static constexpr int MAX_SENSOR_COUNT = 1;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_IMU>::type *_vehicle_imu_subs[MAX_SENSOR_COUNT];
	int _imu_sub_index{-1};
	bool _callback_registered{false};
	int _lockstep_component{-1};

	// because we can have several distance sensor instances with different orientations
	static constexpr int MAX_RNG_SENSOR_COUNT = 4;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_range_finder_subs[MAX_RNG_SENSOR_COUNT];
	int _range_finder_sub_index = -1; // index for downward-facing range finder subscription

	// because we can have multiple GPS instances
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_GPS_POSITION>::type *_gps_subs;
    
	sensor_selections_s		_sensor_selection{};
	vehicle_land_detected_s		_vehicle_land_detected{};
	vehicle_status_s		_vehicle_status{};
        
    uORB::uORBtopicsTypeMap<uORB::TOPIC_EKF2_TIMESTAMPS>::type _ekf2_timestamps_pub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_EKF_GPS_DRIFT>::type _ekf_gps_drift_pub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_EKF_GPS_POSITION>::type _blended_gps_pub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_INNOVATIONS>::type _estimator_innovation_test_ratios_pub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_INNOVATIONS>::type _estimator_innovation_variances_pub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_INNOVATIONS>::type _estimator_innovations_pub;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_SENSOR_BIAS>::type _estimator_sensor_bias_pub;   
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_STATUS>::type _estimator_status_pub;   
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ATTITUDE>::type _att_pub;   
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ODOMETRY>::type _vehicle_odometry_pub; 
    uORB::uORBtopicsTypeMap<uORB::TOPIC_YAW_ESTIMATOR_STATUS>::type _yaw_est_pub;  
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_GLOBAL_POSITION>::type _vehicle_global_position_pub;   
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_LOCAL_POSITION>::type _vehicle_local_position_pub; 
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ODOMETRY>::type _vehicle_visual_odometry_aligned_pub; 
    uORB::uORBtopicsTypeMap<uORB::TOPIC_WIND_ESTIMATE>::type _wind_pub; 
        
	Ekf _ekf;

	parameters *_params;	///< pointer to ekf parameter struct (located in _ekf class instance)
    
    ParamExtInt<UParams::EKF2_MIN_OBS_DT> _param_ekf2_min_obs_dt;	///< Maximum time delay of any sensor used to increase buffer length to handle large timing jitter mSec
    ParamExtFloat<UParams::EKF2_MAG_DELAY> _param_ekf2_mag_delay;	///< magnetometer measurement delay relative to the IMU mSec
    ParamExtFloat<UParams::EKF2_BARO_DELAY> _param_ekf2_baro_delay;	///< barometer height measurement delay relative to the IMU mSec
    ParamExtFloat<UParams::EKF2_GPS_DELAY> _param_ekf2_gps_delay;	///< GPS measurement delay relative to the IMU mSec
    ParamExtFloat<UParams::EKF2_OF_DELAY> _param_ekf2_of_delay;	///< optical flow measurement delay relative to the IMU mSec - this is to the middle of the optical flow integration interval
    ParamExtFloat<UParams::EKF2_RNG_DELAY> _param_ekf2_rng_delay;	///< range finder measurement delay relative to the IMU mSec
    ParamExtFloat<UParams::EKF2_ASP_DELAY> _param_ekf2_asp_delay;	///< airspeed measurement delay relative to the IMU mSec
    ParamExtFloat<UParams::EKF2_EV_DELAY> _param_ekf2_ev_delay;	///< off-board vision measurement delay relative to the IMU mSec
    ParamExtFloat<UParams::EKF2_AVEL_DELAY> _param_ekf2_avel_delay;	///< auxillary velocity measurement delay relative to the IMU mSec

    ParamExtFloat<UParams::EKF2_GYR_NOISE> _param_ekf2_gyr_noise;	///< IMU angular rate noise used for covariance prediction rad/sec
    ParamExtFloat<UParams::EKF2_ACC_NOISE> _param_ekf2_acc_noise;	///< IMU acceleration noise use for covariance prediction m/sec**2

    // process noise
    ParamExtFloat<UParams::EKF2_GYR_B_NOISE> _param_ekf2_gyr_b_noise;	///< process noise for IMU rate gyro bias prediction rad/sec**2
    ParamExtFloat<UParams::EKF2_ACC_B_NOISE> _param_ekf2_acc_b_noise;///< process noise for IMU accelerometer bias prediction m/sec**3
    ParamExtFloat<UParams::EKF2_MAG_E_NOISE> _param_ekf2_mag_e_noise;	///< process noise for earth magnetic field prediction Gauss/sec
    ParamExtFloat<UParams::EKF2_MAG_B_NOISE> _param_ekf2_mag_b_noise;	///< process noise for body magnetic field prediction Gauss/sec
    ParamExtFloat<UParams::EKF2_WIND_NOISE> _param_ekf2_wind_noise;	///< process noise for wind velocity prediction m/sec**2
    ParamExtFloat<UParams::EKF2_TERR_NOISE> _param_ekf2_terr_noise;	///< process noise for terrain offset m/sec
    ParamExtFloat<UParams::EKF2_TERR_GRAD> _param_ekf2_terr_grad;	///< gradient of terrain used to estimate process noise due to changing position m/m

    ParamExtFloat<UParams::EKF2_GPS_V_NOISE> _param_ekf2_gps_v_noise;	///< minimum allowed observation noise for gps velocity fusion m/sec
    ParamExtFloat<UParams::EKF2_GPS_P_NOISE> _param_ekf2_gps_p_noise;	///< minimum allowed observation noise for gps position fusion m
    ParamExtFloat<UParams::EKF2_NOAID_NOISE> _param_ekf2_noaid_noise;	///< observation noise for non-aiding position fusion m
    ParamExtFloat<UParams::EKF2_BARO_NOISE> _param_ekf2_baro_noise;	///< observation noise for barometric height fusion m
    ParamExtFloat<UParams::EKF2_BARO_GATE> _param_ekf2_baro_gate;	///< barometric height innovation consistency gate size STD
    ParamExtFloat<UParams::EKF2_GND_EFF_DZ> _param_ekf2_gnd_eff_dz;	///< barometric deadzone range for negative innovations m
    ParamExtFloat<UParams::EKF2_GND_MAX_HGT> _param_ekf2_gnd_max_hgt;	///< maximum height above the ground level for expected negative baro innovations m
    ParamExtFloat<UParams::EKF2_GPS_P_GATE> _param_ekf2_gps_p_gate;	///< GPS horizontal position innovation consistency gate size STD
    ParamExtFloat<UParams::EKF2_GPS_V_GATE> _param_ekf2_gps_v_gate;	///< GPS velocity innovation consistency gate size STD
    ParamExtFloat<UParams::EKF2_TAS_GATE> _param_ekf2_tas_gate;	///< True Airspeed innovation consistency gate size STD

    // control of magnetometer fusion
    ParamExtFloat<UParams::EKF2_HEAD_NOISE> _param_ekf2_head_noise;	///< measurement noise used for simple heading fusion rad
    ParamExtFloat<UParams::EKF2_MAG_NOISE> _param_ekf2_mag_noise;		///< measurement noise used for 3-axis magnetoemeter fusion Gauss
    ParamExtFloat<UParams::EKF2_EAS_NOISE> _param_ekf2_eas_noise;		///< measurement noise used for airspeed fusion m/sec
    ParamExtFloat<UParams::EKF2_BETA_GATE> _param_ekf2_beta_gate; ///< synthetic sideslip innovation consistency gate size STD
    ParamExtFloat<UParams::EKF2_BETA_NOISE> _param_ekf2_beta_noise;	///< synthetic sideslip noise rad
    ParamExtFloat<UParams::EKF2_MAG_DECL> _param_ekf2_mag_decl;///< magnetic declination degrees
    ParamExtFloat<UParams::EKF2_HDG_GATE> _param_ekf2_hdg_gate;///< heading fusion innovation consistency gate size STD
    ParamExtFloat<UParams::EKF2_MAG_GATE> _param_ekf2_mag_gate;	///< magnetometer fusion innovation consistency gate size STD
    ParamExtInt<UParams::EKF2_DECL_TYPE> _param_ekf2_decl_type;	///< bitmask used to control the handling of declination data
    ParamExtInt<UParams::EKF2_MAG_TYPE> _param_ekf2_mag_type;	///< integer used to specify the type of magnetometer fusion used
    ParamExtFloat<UParams::EKF2_MAG_ACCLIM> _param_ekf2_mag_acclim;	///< integer used to specify the type of magnetometer fusion used
    ParamExtFloat<UParams::EKF2_MAG_YAWLIM> _param_ekf2_mag_yawlim;	///< yaw rate threshold used by mode select logic rad/sec

    ParamExtInt<UParams::EKF2_GPS_CHECK> _param_ekf2_gps_check;	///< bitmask used to control which GPS quality checks are used
    ParamExtFloat<UParams::EKF2_REQ_EPH> _param_ekf2_req_eph;	///< maximum acceptable horiz position error m
    ParamExtFloat<UParams::EKF2_REQ_EPV> _param_ekf2_req_epv;	///< maximum acceptable vert position error m
    ParamExtFloat<UParams::EKF2_REQ_SACC> _param_ekf2_req_sacc;	///< maximum acceptable speed error m/s
    ParamExtInt<UParams::EKF2_REQ_NSATS> _param_ekf2_req_nsats;	///< minimum acceptable satellite count
    ParamExtFloat<UParams::EKF2_REQ_PDOP> _param_ekf2_req_pdop;	///< maximum acceptable position dilution of precision
    ParamExtFloat<UParams::EKF2_REQ_HDRIFT> _param_ekf2_req_hdrift;	///< maximum acceptable horizontal drift speed m/s
    ParamExtFloat<UParams::EKF2_REQ_VDRIFT> _param_ekf2_req_vdrift;	///< maximum acceptable vertical drift speed m/s

    // measurement source control
    ParamExtInt<UParams::EKF2_AID_MASK> _param_ekf2_aid_mask;		///< bitmasked integer that selects which of the GPS and optical flow aiding sources will be used
    ParamExtInt<UParams::EKF2_HGT_MODE> _param_ekf2_hgt_mode;	///< selects the primary source for height data
    ParamExtInt<UParams::EKF2_TERR_MASK> _param_ekf2_terr_mask; ///< bitmasked integer that selects which of range finder and optical flow aiding sources will be used for terrain estimation
    ParamExtInt<UParams::EKF2_NOAID_TOUT> _param_ekf2_noaid_tout;	///< maximum lapsed time from last fusion of measurements that constrain drift before the EKF will report the horizontal nav solution invalid uSec

    // range finder fusion
    ParamExtFloat<UParams::EKF2_RNG_NOISE> _param_ekf2_rng_noise;	///< observation noise for range finder measurements m
    ParamExtFloat<UParams::EKF2_RNG_SFE> _param_ekf2_rng_sfe; ///< scale factor from range to range noise m/m
    ParamExtFloat<UParams::EKF2_RNG_GATE> _param_ekf2_rng_gate;	///< range finder fusion innovation consistency gate size STD
    ParamExtFloat<UParams::EKF2_MIN_RNG> _param_ekf2_min_rng;	///< minimum valid value for range when on ground m
    ParamExtFloat<UParams::EKF2_RNG_PITCH> _param_ekf2_rng_pitch;	///< range sensor pitch offset rad
    ParamExtInt<UParams::EKF2_RNG_AID> _param_ekf2_rng_aid;		///< enables use of a range finder even if primary height source is not range finder
    ParamExtFloat<UParams::EKF2_RNG_A_VMAX> _param_ekf2_rng_a_vmax;	///< maximum allowed horizontal velocity for range aid m/s
    ParamExtFloat<UParams::EKF2_RNG_A_HMAX> _param_ekf2_rng_a_hmax;	///< maximum allowed absolute altitude AGL for range aid m
    ParamExtFloat<UParams::EKF2_RNG_A_IGATE> _param_ekf2_rng_a_igate;	///< gate size used for innovation consistency checks for range aid fusion STD

    // vision estimate fusion
    ParamInt<UParams::EKF2_EV_NOISE_MD>
    _param_ekf2_ev_noise_md;	///< determine source of vision observation noise
    ParamFloat<UParams::EKF2_EVP_NOISE>
    _param_ekf2_evp_noise;	///< default position observation noise for exernal vision measurements m
    ParamFloat<UParams::EKF2_EVV_NOISE>
    _param_ekf2_evv_noise;	///< default velocity observation noise for exernal vision measurements m/s
    ParamFloat<UParams::EKF2_EVA_NOISE>
    _param_ekf2_eva_noise;	///< default angular observation noise for exernal vision measurements rad
    ParamExtFloat<UParams::EKF2_EVV_GATE>
    _param_ekf2_evv_gate;	///< external vision velocity innovation consistency gate size STD
    ParamExtFloat<UParams::EKF2_EVP_GATE>
    _param_ekf2_evp_gate;	///< external vision position innovation consistency gate size STD

    // optical flow fusion
    ParamExtFloat<UParams::EKF2_OF_N_MIN>
    _param_ekf2_of_n_min;	///< best quality observation noise for optical flow LOS rate measurements rad/sec
    ParamExtFloat<UParams::EKF2_OF_N_MAX>
    _param_ekf2_of_n_max;	///< worst quality observation noise for optical flow LOS rate measurements rad/sec
    ParamExtInt<UParams::EKF2_OF_QMIN>
    _param_ekf2_of_qmin;	///< minimum acceptable quality integer from  the flow sensor
    ParamExtFloat<UParams::EKF2_OF_GATE>
    _param_ekf2_of_gate;	///< optical flow fusion innovation consistency gate size STD

    ParamInt<UParams::EKF2_IMU_ID> _param_ekf2_imu_id;

    // sensor positions in body frame
    ParamExtFloat<UParams::EKF2_IMU_POS_X> _param_ekf2_imu_pos_x;		///< X position of IMU in body frame m
    ParamExtFloat<UParams::EKF2_IMU_POS_Y> _param_ekf2_imu_pos_y;		///< Y position of IMU in body frame m
    ParamExtFloat<UParams::EKF2_IMU_POS_Z> _param_ekf2_imu_pos_z;		///< Z position of IMU in body frame m
    ParamExtFloat<UParams::EKF2_GPS_POS_X> _param_ekf2_gps_pos_x;		///< X position of GPS antenna in body frame m
    ParamExtFloat<UParams::EKF2_GPS_POS_Y> _param_ekf2_gps_pos_y;		///< Y position of GPS antenna in body frame m
    ParamExtFloat<UParams::EKF2_GPS_POS_Z> _param_ekf2_gps_pos_z;		///< Z position of GPS antenna in body frame m
    ParamExtFloat<UParams::EKF2_RNG_POS_X> _param_ekf2_rng_pos_x;		///< X position of range finder in body frame m
    ParamExtFloat<UParams::EKF2_RNG_POS_Y> _param_ekf2_rng_pos_y;		///< Y position of range finder in body frame m
    ParamExtFloat<UParams::EKF2_RNG_POS_Z> _param_ekf2_rng_pos_z;		///< Z position of range finder in body frame m
    ParamExtFloat<UParams::EKF2_OF_POS_X> _param_ekf2_of_pos_x;	///< X position of optical flow sensor focal point in body frame m
    ParamExtFloat<UParams::EKF2_OF_POS_Y> _param_ekf2_of_pos_y;	///< Y position of optical flow sensor focal point in body frame m
    ParamExtFloat<UParams::EKF2_OF_POS_Z> _param_ekf2_of_pos_z;	///< Z position of optical flow sensor focal point in body frame m
    ParamExtFloat<UParams::EKF2_EV_POS_X> _param_ekf2_ev_pos_x;		///< X position of VI sensor focal point in body frame m
    ParamExtFloat<UParams::EKF2_EV_POS_Y> _param_ekf2_ev_pos_y;		///< Y position of VI sensor focal point in body frame m
    ParamExtFloat<UParams::EKF2_EV_POS_Z> _param_ekf2_ev_pos_z;		///< Z position of VI sensor focal point in body frame m

    // control of airspeed and sideslip fusion
    ParamFloat<UParams::EKF2_ARSP_THR> _param_ekf2_arsp_thr; 	///< A value of zero will disabled airspeed fusion. Any positive value sets the minimum airspeed which will be used m/sec
    ParamInt<UParams::EKF2_FUSE_BETA> _param_ekf2_fuse_beta;		///< Controls synthetic sideslip fusion; 0 disables; 1 enables

    // output predictor filter time constants
    ParamExtFloat<UParams::EKF2_TAU_VEL> _param_ekf2_tau_vel;		///< time constant used by the output velocity complementary filter sec
    ParamExtFloat<UParams::EKF2_TAU_POS> _param_ekf2_tau_pos;		///< time constant used by the output position complementary filter sec

    // IMU switch on bias parameters
    ParamExtFloat<UParams::EKF2_GBIAS_INIT> _param_ekf2_gbias_init;	///< 1-sigma gyro bias uncertainty at switch on rad/sec
    ParamExtFloat<UParams::EKF2_ABIAS_INIT> _param_ekf2_abias_init;	///< 1-sigma accelerometer bias uncertainty at switch on m/sec**2
    ParamExtFloat<UParams::EKF2_ANGERR_INIT> _param_ekf2_angerr_init;	///< 1-sigma tilt error after initial alignment using gravity vector rad

    // EKF saved XYZ magnetometer bias values
    ParamFloat<UParams::EKF2_MAGBIAS_X> _param_ekf2_magbias_x;		///< X magnetometer bias mGauss
    ParamFloat<UParams::EKF2_MAGBIAS_Y> _param_ekf2_magbias_y;		///< Y magnetometer bias mGauss
    ParamFloat<UParams::EKF2_MAGBIAS_Z> _param_ekf2_magbias_z;		///< Z magnetometer bias mGauss
    ParamInt<UParams::EKF2_MAGBIAS_ID> _param_ekf2_magbias_id;		///< ID of the magnetometer sensor used to learn the bias values
    ParamFloat<UParams::EKF2_MAGB_VREF> _param_ekf2_magb_vref; ///< Assumed error variance of previously saved magnetometer bias estimates mGauss**2
    ParamFloat<UParams::EKF2_MAGB_K> _param_ekf2_magb_k;	///< maximum fraction of the learned magnetometer bias that is saved at each disarm

    // EKF accel bias learning control
    ParamExtFloat<UParams::EKF2_ABL_LIM> _param_ekf2_abl_lim;	///< Accelerometer bias learning limit m/s**2
    ParamExtFloat<UParams::EKF2_ABL_ACCLIM> _param_ekf2_abl_acclim;	///< Maximum IMU accel magnitude that allows IMU bias learning m/s**2
    ParamExtFloat<UParams::EKF2_ABL_GYRLIM> _param_ekf2_abl_gyrlim;	///< Maximum IMU gyro angular rate magnitude that allows IMU bias learning m/s**2
    ParamExtFloat<UParams::EKF2_ABL_TAU> _param_ekf2_abl_tau;	///< Time constant used to inhibit IMU delta velocity bias learning sec

    // Multi-rotor drag specific force fusion
    ParamExtFloat<UParams::EKF2_DRAG_NOISE> _param_ekf2_drag_noise;	///< observation noise variance for drag specific force measurements m/sec**2**2
    ParamExtFloat<UParams::EKF2_BCOEF_X> _param_ekf2_bcoef_x;		///< ballistic coefficient along the X-axis kg/m**2
    ParamExtFloat<UParams::EKF2_BCOEF_Y> _param_ekf2_bcoef_y;		///< ballistic coefficient along the Y-axis kg/m**2

    // Corrections for static pressure position error where Ps_error = Ps_meas - Ps_truth
    // Coef = Ps_error / Pdynamic; where Pdynamic = 1/2 * density * TAS**2
    ParamExtFloat<UParams::EKF2_ASPD_MAX> _param_ekf2_aspd_max;		///< upper limit on airspeed used for correction  m/s**2
    ParamExtFloat<UParams::EKF2_PCOEF_XP> _param_ekf2_pcoef_xp;	///< static pressure position error coefficient along the positive X body axis
    ParamExtFloat<UParams::EKF2_PCOEF_XN> _param_ekf2_pcoef_xn;	///< static pressure position error coefficient along the negative X body axis
    ParamExtFloat<UParams::EKF2_PCOEF_YP> _param_ekf2_pcoef_yp;	///< static pressure position error coefficient along the positive Y body axis
    ParamExtFloat<UParams::EKF2_PCOEF_YN> _param_ekf2_pcoef_yn;	///< static pressure position error coefficient along the negative Y body axis
    ParamExtFloat<UParams::EKF2_PCOEF_Z> _param_ekf2_pcoef_z;	///< static pressure position error coefficient along the Z body axis

    // GPS blending
    ParamInt<UParams::EKF2_GPS_MASK> _param_ekf2_gps_mask;	///< mask defining when GPS accuracy metrics are used to calculate the blend ratio
    ParamFloat<UParams::EKF2_GPS_TAU> _param_ekf2_gps_tau;		///< time constant controlling how rapidly the offset used to bring GPS solutions together is allowed to change sec

    // Test used to determine if the vehicle is static or moving
    ParamExtFloat<UParams::EKF2_MOVE_TEST> _param_ekf2_move_test;	///< scaling applied to IMU data thresholds used to determine if the vehicle is static or moving.

    ParamFloat<UParams::EKF2_REQ_GPS_H> _param_ekf2_req_gps_h; ///< Required GPS health time
    ParamExtInt<UParams::EKF2_MAG_CHECK> _param_ekf2_mag_check; ///< Mag field strength check

    // Used by EKF-GSF experimental yaw estimator
    ParamExtFloat<UParams::EKF2_GSF_TAS> _param_ekf2_gsf_tas_default;	///< default value of true airspeed assumed during fixed wing operation

};

