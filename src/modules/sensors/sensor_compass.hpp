#pragma once

#include "sensor_calibration/compass_calibrator.h"
#include "sensor_calibration/compass_calibrator_bak.h"
#include "mathlib/math/Limits.hpp"
#include "matrix/matrix/math.hpp"
#include "mathlib/math/WelfordMean.hpp"
#include "mathlib/math/WelfordMeanVector.hpp"
#include "conversion/rotation.h"
#include "mathlib/math/filter/LowPassFilter2p.hpp"
#include "utility/log.hpp"
#include "platform_defines.h"
#include "uORB/uORB_topic_define.hpp"
#include "param/parameters.hpp"
#include "integrator.hpp"
#include "ringbuffersensor.hpp"

namespace sensors
{
class SensorCompass
{
public:
	SensorCompass() = default;
	~SensorCompass() = default;

	bool start();
    void run();
    bool param_update();

public:
	/**
	 * Calculates the magnitude in Gauss of the largest difference between the primary and any other magnetometers
	 */
	void calc_maginconsistency();               // 计算多个磁力计的不一致性
    void check_failover(const hrt_abstime &time_now_us);
    void update_status();
	void update_magbiasestimate();              // 更新磁力估计器的统计误差
	void update_magcalibration();               // 更新估计器的磁力计统计误差
	void update_powercompensation();            // 更新电源电流磁干扰补偿

    // uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_STATUS>::type _pub_sensor_status_mag;
    // uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_PREFLIGHT_MAG>::type _pub_sensor_preflight_mag;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_MAGNETOMETER>::type _pub_vehicle_mag;

    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_MAG>::type *_sub_sensor_mag;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_MAGNETOMETER_BIAS_ESTIMATE>::type *_sub_magnetometer_bias_estimate;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_SENSOR_BIAS>::type *_sub_estimator_sensor_bias;

    uint32_t _compass_rate{100};
    
	struct MagCal {
		uint32_t device_id{0};
		matrix::Vector3f offset{};
		matrix::Vector3f variance{};
	} _mag_cal {};

	hrt_abstime _last_calibration_update{0};
    hrt_abstime _compass_timestamp_sample_last{0};
    
	matrix::Vector3f _calibration_estimator_bias {};
    matrix::Vector3f _compass_prev {};
        
    // calibration::CompassCalibratorBak _calibration;
    calibration::CompassCalibrator _calibration;
    math::LowPassFilter2p<matrix::Vector3f> _lp_compass_filter{};
    
    ParamFloat<UParams::COMPASS_FILTER> _param_compass_filter_cutoff;
};

}; // namespace sensors
