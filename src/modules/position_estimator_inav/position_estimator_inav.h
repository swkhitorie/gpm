#pragma once

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <geo/geo.h>
#include <platform_defines.h>
#include <terrain_estimation/terrain_estimator.h>

#include "param/parameters.hpp"
#include "uORB_topic_define.hpp"
#include "mavlink_vehicle.h"

#define CBRK_NO_VISION_KEY	328754

class PositionEstimatorINav
{
public:
	PositionEstimatorINav() = default;
	~PositionEstimatorINav() = default;

	void init();

	void run();

protected:

	void inertial_filter_predict(float dt, float x[2], float acc);

	void inertial_filter_correct(float e, float dt, float x[2], int i, float w);

	float _param_w_z_baro {0.5f};
	float _param_w_z_gps_p{0.005f};
	float _param_w_z_gps_v{0.0f};
	float _param_w_z_vision_p{5.0f};
	float _param_w_z_lidar{3.0f};
	float _param_w_xy_gps_p{1.0f};
	float _param_w_xy_gps_v{2.0f};
	float _param_w_xy_vision_p{7.0f};
	float _param_w_xy_vision_v{0.0f};
	float _param_w_mocap_p{10.0f};
	float _param_w_xy_flow{0.8f};
	float _param_w_xy_res_v{0.5f};
	float _param_w_gps_flow{0.1f};
	float _param_w_acc_bias{0.05f};
	float _param_flow_k{1.35f};
	float _param_flow_q_min{0.3f};
	float _param_lidar_err{0.2f};
	float _param_land_t{3.0f};
	float _param_land_disp{0.7f};
	float _param_land_thr{0.2f};
	float _param_delay_gps{0.2f};	
	float _param_flow_module_offset_x{0.0f};
	float _param_flow_module_offset_y{0.0f};
	int _param_disable_mocap{0};
	int _param_enable_lidar_alt_est{0};
	float _param_lidar_calibration_offset{0.0f};
	int _param_no_vision{0};
	// ParamFloat<UParams::INAV_W_Z_BARO> _param_w_z_baro;					//0.5f
	// ParamFloat<UParams::INAV_W_Z_GPS_P> _param_w_z_gps_p;				//0.005f
	// ParamFloat<UParams::INAV_W_Z_GPS_V> _param_w_z_gps_v;				//0.0f
	// ParamFloat<UParams::INAV_W_Z_VIS_P> _param_w_z_vision_p;			//5.0f
	// ParamFloat<UParams::INAV_W_Z_LIDAR> _param_w_z_lidar;				//3.0f
	// ParamFloat<UParams::INAV_W_XY_GPS_P> _param_w_xy_gps_p;				//1.0f
	// ParamFloat<UParams::INAV_W_XY_GPS_V> _param_w_xy_gps_v;				//2.0f
	// ParamFloat<UParams::INAV_W_XY_VIS_P> _param_w_xy_vision_p;			//7.0f
	// ParamFloat<UParams::INAV_W_XY_VIS_V> _param_w_xy_vision_v;			//0.0f
	// ParamFloat<UParams::INAV_W_MOC_P> _param_w_mocap_p;					//10.0f
	// ParamFloat<UParams::INAV_W_XY_FLOW> _param_w_xy_flow;				//0.8f
	// ParamFloat<UParams::INAV_W_XY_RES_V> _param_w_xy_res_v;				//0.5f
	// ParamFloat<UParams::INAV_W_GPS_FLOW> _param_w_gps_flow;				//0.1f
	// ParamFloat<UParams::INAV_W_ACC_BIAS> _param_w_acc_bias;				//0.05f
	// ParamFloat<UParams::INAV_FLOW_K> _param_flow_k;						//1.35f
	// ParamFloat<UParams::INAV_FLOW_Q_MIN> _param_flow_q_min;				//0.3f
	// ParamFloat<UParams::INAV_LIDAR_ERR> _param_lidar_err;				//0.2f
	// ParamFloat<UParams::INAV_LAND_T> _param_land_t;						//3.0f
	// ParamFloat<UParams::INAV_LAND_DISP> _param_land_disp;				//0.7f
	// ParamFloat<UParams::INAV_LAND_THR> _param_land_thr;					//0.2f
	// ParamFloat<UParams::INAV_DELAY_GPS> _param_delay_gps;				//0.2f
	// ParamFloat<UParams::INAV_FLOW_DIST_X> _param_flow_module_offset_x;	//0.0f
	// ParamFloat<UParams::INAV_DISAB_MOCAP> _param_flow_module_offset_y;	//0.0f
	// ParamInt<UParams::INAV_DISAB_MOCAP> _param_disable_mocap;			//0
	// ParamInt<UParams::INAV_LIDAR_EST> _param_enable_lidar_alt_est;		//0
	// ParamFloat<UParams::INAV_LIDAR_OFF> _param_lidar_calibration_offset;	//0.0f
	// ParamInt<UParams::CBRK_NO_VISION> _param_no_vision;					//0
};

