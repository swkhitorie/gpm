#pragma once

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <geo/geo.h>
#include "controllib/blocks.hpp"

#include "param/parameters.hpp"
#include "platform_defines.h"
#include "uORB_topic_define.hpp"
#include "mavlink_vehicle.h"

/*
    修改部分->
    适当的头文件引用修改和using namespace time删除
    修改pub 和 sub对象
    修改param对象
    调试: 
        手动给param赋值
        手动给blocks设置dt和参数
    px4 设计踩坑: blocklowpassVector 的 vec 初始化为0.0f/0.0f, 需要手动检查或者改初始化
*/

/*
original: 
static const float DELAY_MAX = 0.5f;	// seconds                  // 0.5s
static const float HIST_STEP = 0.05f;	// 20 hz
static const float BIAS_MAX = 1e-1f;
static const size_t HIST_LEN = 10;	// DELAY_MAX / HIST_STEP;       // 0.5s / 0.05f = 10
static const size_t N_DIST_SUBS = 4;

 (dt_hist >= HIST_STEP) 从 > 改为 >=
*/


using namespace matrix;
using namespace control;

static const float DELAY_MAX = 0.5f;	// seconds
static const float HIST_STEP = 0.05f;	// 50 hz
static const float BIAS_MAX = 1e-1f;
static const size_t HIST_LEN = 10;	// DELAY_MAX / HIST_STEP;
static const size_t N_DIST_SUBS = 4;

// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
static const float BETA_TABLE[7] = {0,
				    8.82050518214,
				    12.094592431,
				    13.9876612368,
				    16.0875642296,
				    17.8797700658,
				    19.6465647819,
				   };

class BlockLocalPositionEstimator
{
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
//      ax, ay, az (acceleration NED)
//
// states:
//      px, py, pz , ( position NED, m)
//      vx, vy, vz ( vel NED, m/s),
//      bx, by, bz ( accel bias, m/s^2)
//      tz (terrain altitude, ASL, m)
//
// measurements:
//
//      sonar: pz (measured d*cos(phi)*cos(theta))
//
//      baro: pz
//
//      flow: vx, vy (flow is in body x, y frame)
//
//      gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
//      lidar: pz (actual measured d*cos(phi)*cos(theta))
//
//      vision: px, py, pz, vx, vy, vz
//
//      mocap: px, py, pz
//
//      land (detects when landed)): pz (always measures agl = 0)
//
public:

	BlockLocalPositionEstimator();
	~BlockLocalPositionEstimator() = default;

	bool init();

    void Run();

public:

	// constants
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_vx = 0, Y_flow_vy, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {Y_land_vx = 0, Y_land_vy, Y_land_agl, n_y_land};
	enum {Y_target_x = 0, Y_target_y, n_y_target};
	enum {
		FUSE_GPS = 1 << 0,
		FUSE_FLOW = 1 << 1,
		FUSE_VIS_POS = 1 << 2,
		FUSE_LAND_TARGET = 1 << 3,
		FUSE_LAND = 1 << 4,
		FUSE_PUB_AGL_Z = 1 << 5,
		FUSE_FLOW_GYRO_COMP = 1 << 6,
		FUSE_BARO = 1 << 7
	};

	enum sensor_t {
		SENSOR_BARO = 1 << 0,
		SENSOR_GPS = 1 << 1,
		SENSOR_LIDAR = 1 << 2,
		SENSOR_FLOW = 1 << 3,
		SENSOR_SONAR = 1 << 4,
		SENSOR_VISION = 1 << 5,
		SENSOR_MOCAP = 1 << 6,
		SENSOR_LAND = 1 << 7,
		SENSOR_LAND_TARGET = 1 << 8,
	};

	enum estimate_t {
		EST_XY = 1 << 0,
		EST_Z = 1 << 1,
		EST_TZ = 1 << 2,
	};

	// methods
	// ----------------------------
	//
	Vector<float, n_x> dynamics(
		float t,
		const Vector<float, n_x> &x,
		const Vector<float, n_u> &u);
	void initP();
	void initSS();
	void updateSSStates();
	void updateSSParams();

	// predict the next state
	void predict(const sensor_combined_s &imu);

	// lidar
	int  lidarMeasure(Vector<float, n_y_lidar> &y);
	void lidarCorrect();
	void lidarInit();
	void lidarCheckTimeout();

	// sonar
	int  sonarMeasure(Vector<float, n_y_sonar> &y);
	void sonarCorrect();
	void sonarInit();
	void sonarCheckTimeout();

	// baro
	int  baroMeasure(Vector<float, n_y_baro> &y);
	void baroCorrect();
	void baroInit();
	void baroCheckTimeout();

	// gps
	int  gpsMeasure(Vector<double, n_y_gps> &y);
	void gpsCorrect();
	void gpsInit();
	void gpsCheckTimeout();

	// flow
	int  flowMeasure(Vector<float, n_y_flow> &y);
	void flowCorrect();
	void flowInit();
	void flowCheckTimeout();

	// vision
	int  visionMeasure(Vector<float, n_y_vision> &y);
	void visionCorrect();
	void visionInit();
	void visionCheckTimeout();

	// mocap
	int  mocapMeasure(Vector<float, n_y_mocap> &y);
	void mocapCorrect();
	void mocapInit();
	void mocapCheckTimeout();

	// land
	int  landMeasure(Vector<float, n_y_land> &y);
	void landCorrect();
	void landInit();
	void landCheckTimeout();

	// landing target
	int  landingTargetMeasure(Vector<float, n_y_target> &y);
	void landingTargetCorrect();
	void landingTargetInit();
	void landingTargetCheckTimeout();

	// timeouts
	void checkTimeouts();

	// misc
	inline float agl()
	{
		return _x(X_tz) - _x(X_z);
	}
	bool landed();
	int getDelayPeriods(float delay, uint8_t *periods);

	// publications
	void publishLocalPos();
	void publishGlobalPos();
	void publishOdom();
	void publishEstimatorStatus();
	void publishEk2fTimestamps();

	// attributes
	// ----------------------------

	// subscriptions
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_COMBINED>::type *_sub_sensors;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_COMMAND>::type *_sub_vehicle_command;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ACTUATOR_ARMED>::type *_sub_armed;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_LAND_DETECTED>::type *_sub_land;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ATTITUDE>::type *_sub_att;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ANGULAR_VELOCITY>::type *_sub_angular_velocity;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_OPTICAL_FLOW>::type *_sub_flow;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_GPS_POSITION>::type *_sub_gps;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ODOMETRY>::type *_sub_visual_odom;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ODOMETRY>::type *_sub_mocap_odom;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_LANDING_TARGET_POSE>::type *_sub_landing_target_pose;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_AIR_DATA>::type *_sub_airdata;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_sub_dist0;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_sub_dist1;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_sub_dist2;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_sub_dist3;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_dist_subs[N_DIST_SUBS] {};
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_sub_lidar{nullptr};
    uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_sub_sonar{nullptr};

	// publications
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_LOCAL_POSITION>::type _pub_lpos;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_GLOBAL_POSITION>::type _pub_gpos;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ODOMETRY>::type _pub_odom;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_STATES>::type _pub_est_states;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_STATUS>::type _pub_est_status;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_INNOVATIONS>::type _pub_innov;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_INNOVATIONS>::type _pub_innov_var;

    hrt_abstime _prev_timestamp;
    
	// map projection
	MapProjection _map_ref;

	MapProjection _global_local_proj_ref{};
	float _global_local_alt0{NAN};

	// target mode paramters from landing_target_estimator module
	enum TargetMode {
		Target_Moving = 0,
		Target_Stationary = 1
	};

	// flow gyro filter
	BlockHighPass _flow_gyro_x_high_pass;
	BlockHighPass _flow_gyro_y_high_pass;

	// stats
	BlockStats<float, n_y_baro> _baroStats;
	BlockStats<float, n_y_sonar> _sonarStats;
	BlockStats<float, n_y_lidar> _lidarStats;
	BlockStats<float, 1> _flowQStats;
	BlockStats<float, n_y_vision> _visionStats;
	BlockStats<float, n_y_mocap> _mocapStats;
	BlockStats<double, n_y_gps> _gpsStats;
	uint16_t _landCount;

	// low pass
	BlockLowPassVector<float, n_x> _xLowPass;
	BlockLowPass _aglLowPass;

	// delay blocks
	BlockDelay<float, n_x, 1, HIST_LEN> _xDelay;
	BlockDelay<uint64_t, 1, 1, HIST_LEN> _tDelay;

	// misc
	uint64_t _timeStamp;
	uint64_t _time_origin;
	uint64_t _timeStampLastBaro;
	uint64_t _time_last_hist;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_init_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_mocap;
	uint64_t _time_last_land;
	uint64_t _time_last_target;

	// reference altitudes
	float _altOrigin;
	bool _altOriginInitialized;
	bool _altOriginGlobal; // true when the altitude of the origin is defined wrt a global reference frame
	float _baroAltOrigin;
	float _gpsAltOrigin;

	// status
	bool _receivedGps;
	bool _lastArmedState;

	// masks
	uint16_t _sensorTimeout;
	uint16_t _sensorFault;
	uint8_t _estimatorInitialized;

	// sensor update flags
	bool _flowUpdated;
	bool _gpsUpdated;
	bool _visionUpdated;
	bool _mocapUpdated;
	bool _lidarUpdated;
	bool _sonarUpdated;
	bool _landUpdated;
	bool _baroUpdated;

	// sensor validation flags
	bool _vision_xy_valid;
	bool _vision_z_valid;
	bool _mocap_xy_valid;
	bool _mocap_z_valid;

	// sensor std deviations
	float _vision_eph;
	float _vision_epv;
	float _mocap_eph;
	float _mocap_epv;

	// local to global coversion related variables
	bool _is_global_cov_init;
	double _ref_lat;
	double _ref_lon;
	float _ref_alt;

	// state space
	Vector<float, n_x>  _x;	// state vector
	Vector<float, n_u>  _u;	// input vector
	Matrix<float, n_x, n_x> m_P;	// state covariance matrix

	matrix::Dcm<float> _R_att;

	Matrix<float, n_x, n_x>  m_A;	// dynamics matrix
	Matrix<float, n_x, n_u>  m_B;	// input matrix
	Matrix<float, n_u, n_u>  m_R;	// input covariance
	Matrix<float, n_x, n_x>  m_Q;	// process noise covariance

    // general parameters

    ParamInt<UParams::LPE_FUSION> _param_lpe_fusion;      //LPE_FUSION
    ParamFloat<UParams::LPE_VXY_PUB> _param_lpe_vxy_pub;      //LPE_VXY_PUB
    ParamFloat<UParams::LPE_Z_PUB> _param_lpe_z_pub;          //LPE_Z_PUB

    // sonar parameters
    ParamFloat<UParams::LPE_SNR_Z> _param_lpe_snr_z;          //LPE_SNR_Z
    ParamFloat<UParams::LPE_SNR_OFF_Z> _param_lpe_snr_off_z;      //LPE_SNR_OFF_Z

    // lidar parameters
    ParamFloat<UParams::LPE_LDR_Z> _param_lpe_ldr_z;              //LPE_LDR_Z
    ParamFloat<UParams::LPE_LDR_OFF_Z> _param_lpe_ldr_off_z;      //LPE_LDR_OFF_Z

    // accel parameters
    ParamFloat<UParams::LPE_ACC_XY> _param_lpe_acc_xy;                //LPE_ACC_XY
    ParamFloat<UParams::LPE_ACC_Z> _param_lpe_acc_z;          //LPE_ACC_Z

    // baro parameters
    ParamFloat<UParams::LPE_BAR_Z> _param_lpe_bar_z;              //LPE_BAR_Z

    // gps parameters
    ParamFloat<UParams::LPE_GPS_DELAY> _param_lpe_gps_delay;          //LPE_GPS_DELAY
    ParamFloat<UParams::LPE_GPS_XY> _param_lpe_gps_xy;                //LPE_GPS_XY
    ParamFloat<UParams::LPE_GPS_Z> _param_lpe_gps_z;                  //LPE_GPS_Z
    ParamFloat<UParams::LPE_GPS_VXY> _param_lpe_gps_vxy;              //LPE_GPS_VXY
    ParamFloat<UParams::LPE_GPS_VZ> _param_lpe_gps_vz;                //LPE_GPS_VZ
    ParamFloat<UParams::LPE_EPH_MAX> _param_lpe_eph_max;              //LPE_EPH_MAX
    ParamFloat<UParams::LPE_EPV_MAX> _param_lpe_epv_max;              //LPE_EPV_MAX

    // vision parameters
    ParamFloat<UParams::LPE_VIS_XY> _param_lpe_vis_xy;                //LPE_VIS_XY
    ParamFloat<UParams::LPE_VIS_Z> _param_lpe_vis_z;                  //LPE_VIS_Z
    ParamFloat<UParams::LPE_VIS_DELAY> _param_lpe_vis_delay;          //LPE_VIS_DELAY

    // mocap parameters
    ParamFloat<UParams::LPE_VIC_P> _param_lpe_vic_p;                  //LPE_VIC_P

    // flow parameters
    ParamFloat<UParams::LPE_FLW_OFF_Z> _param_lpe_flw_off_z;              //LPE_FLW_OFF_Z
    ParamFloat<UParams::LPE_FLW_SCALE> _param_lpe_flw_scale;              //LPE_FLW_SCALE
    ParamInt<UParams::LPE_FLW_QMIN> _param_lpe_flw_qmin;                      //LPE_FLW_QMIN
    ParamFloat<UParams::LPE_FLW_R> _param_lpe_flw_r;                      //LPE_FLW_R
    ParamFloat<UParams::LPE_FLW_RR> _param_lpe_flw_rr;                    //LPE_FLW_RR

    // land parameters
    ParamFloat<UParams::LPE_LAND_Z> _param_lpe_land_z;                    //LPE_LAND_Z
    ParamFloat<UParams::LPE_LAND_VXY> _param_lpe_land_vxy;                //LPE_LAND_VXY

    // process noise
    ParamFloat<UParams::LPE_PN_P> _param_lpe_pn_p;                        //LPE_PN_P
    ParamFloat<UParams::LPE_PN_V> _param_lpe_pn_v;                        //LPE_PN_V
    ParamFloat<UParams::LPE_PN_B> _param_lpe_pn_b;                        //LPE_PN_B
    ParamFloat<UParams::LPE_PN_T> _param_lpe_pn_t;                        //LPE_PN_T
    ParamFloat<UParams::LPE_T_MAX_GRADE> _param_lpe_t_max_grade;          //LPE_T_MAX_GRADE

    ParamFloat<UParams::LPE_LT_COV> _param_lpe_lt_cov;                        //LPE_LT_COV
    ParamInt<UParams::LTEST_MODE> _param_ltest_mode;                      //LTEST_MODE

    // init origin
    ParamInt<UParams::LPE_FAKE_ORIGIN> _param_lpe_fake_origin;                    //LPE_FAKE_ORIGIN
    ParamFloat<UParams::LPE_LAT> _param_lpe_lat;                  //LPE_LAT
    ParamFloat<UParams::LPE_LON> _param_lpe_lon;                       //LPE_LON
};
