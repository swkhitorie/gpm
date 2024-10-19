#pragma once

#include "sensor_calibration/accel_calibrator_bak.h"
#include "sensor_calibration/gyro_calibrator.h"
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

class SensorIMU
{
public:
	SensorIMU() = default;
	~SensorIMU() = default;

	bool start();

    bool run();

    bool param_update();

    calibration::AccelCalibratorBak _accel_calibration{};
	calibration::GyroscopeCalibrator _gyro_calibration{};
        
private:
	bool publish();

    bool update_accel();
    bool update_gyro();
	void update_integrator_configuration();
    inline void update_accel_vibrationmetrics(const matrix::Vector3f &acceleration);
    inline void update_gyro_vibrationmetrics(const matrix::Vector3f &angular_velocity);

	void sensor_calibration_update();                       //根据estimator的bias更新校准参数, 并使能_cal_available标志位
	void sensor_calibration_save_accel();                   //根据estimator的bias更新校准参数, 并保存
	void sensor_calibration_save_gyro();

	// return the square of two floating point numbers
	static constexpr float sq(float var) { return var * var; }

    // uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_IMU>::type _pub_vehicle_imu;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_COMBINED>::type _pub_sensor_combined;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_IMU_STATUS>::type _pub_vehicle_imu_status;
    
	static constexpr hrt_abstime kIMUStatusPublishingInterval{100};  //100ms
    
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_CONTROL_MODE>::type *_sub_vehicle_controlmode;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_BIAS>::type *_sub_estimator_sensor_bias;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_ESTIMATOR_STATES>::type *_sub_estimator_state;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_ACCEL>::type *_sub_sensor_accel;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_GYRO>::type *_sub_sensor_gyro;

	sensors::Integrator       _accel_integrator{};
	sensors::IntegratorConing _gyro_integrator{};  
	float _coning_norm_accum{0};
	float _coning_norm_accum_total_time_s{0};
	bool _update_integrator_config{true};
	bool _intervals_configured{false};

    uint32_t _imu_integ_rate{200};                              // imu 积分频率hz
	uint32_t _imu_integration_interval_us{5000};                // imu 积分周期us
	hrt_abstime _accel_timestamp_sample_last{0};                // imu accel 采样时间戳last
	hrt_abstime _gyro_timestamp_sample_last{0};                 // imu gyro 采样时间戳last
	hrt_abstime _gyro_timestamp_last{0};                        // imu gyro 运行时间戳
	math::WelfordMeanVector<float, 3> _raw_accel_mean{};        // 统计期望, 原始accel数据
	math::WelfordMeanVector<float, 3> _raw_gyro_mean{};         // 统计期望, 原始gyro数据
	math::WelfordMean<float> _accel_mean_interval_us{};         // 统计期望, accel采样间隔 
	math::WelfordMean<float> _gyro_mean_interval_us{};          // 统计期望, gyro采样间隔
	math::WelfordMean<float> _gyro_update_latency_mean_us{};
	math::WelfordMean<float> _gyro_publish_latency_mean_us{};
	float _accel_interval_us{NAN};                              // accel 采样间隔时间
	float _gyro_interval_us{NAN};                               // gyro 采样间隔时间

	matrix::Vector3f _acceleration_prev{};     // acceleration from the previous IMU measurement for vibration metrics
	matrix::Vector3f _angular_velocity_prev{}; // angular velocity from the previous IMU measurement for vibration metrics
        
 	matrix::Vector3f _accel_bias{};
	matrix::Vector3f _gyro_bias{};
        
	bool _accel_cal_available{false};
	bool _gyro_cal_available{false};
    
	struct preflightCalibration {
		matrix::Vector3f offset{};
		matrix::Vector3f scale{};
        matrix::Vector3f bias{};    
		bool valid{false};
	};
    
	preflightCalibration _accel_learned_calibration {};
	preflightCalibration _gyro_learned_calibration {};

    math::LowPassFilter2p<matrix::Vector3f> _lp_accel_filter{};
    math::LowPassFilter2p<matrix::Vector3f> _lp_gyro_filter{};

    ParamFloat<UParams::INS_ACCEL_FILTER> _param_imu_accel_filter_cutoff;
    ParamFloat<UParams::INS_GYRO_FILTER> _param_imu_gyro_filter_cutoff;
};

} // namespace sensors
