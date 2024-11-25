/**
 * @file sensor_range_finder.hpp
 * Range finder class containing all the required checks
 * 包含所有必需检查的测距仪类
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */
#pragma once

#include "Sensor.hpp"
#include <matrix/math.hpp>

namespace estimator
{
namespace sensor
{
/**
	struct rangeSample {
		float       rng;		采样值
		uint64_t    time_us;	时间戳
		int8_t	    quality;    百分比质量评估值, -1:未知
	};
*/
class SensorRangeFinder : public Sensor
{
public:
	SensorRangeFinder() = default;
	~SensorRangeFinder() override = default;

	void runChecks(uint64_t current_time_us, const matrix::Dcmf &R_to_earth);
	bool isHealthy() const override { return _is_sample_valid; }
	bool isDataHealthy() const override { return _is_sample_ready && _is_sample_valid; }

	/***************************
		更新采样值
	***************************/
	void setSample(const rangeSample &sample) {
		_sample = sample;
		_is_sample_ready = true;
	}

	/***************************
		获取当前采样值
	***************************/
	// This is required because of the ring buffer
	// TODO: move the ring buffer here
	rangeSample* getSampleAddress() { return &_sample; }

	/***************************
		设置测距传感器的安装俯仰偏移
	***************************/
	void setPitchOffset(float new_pitch_offset)
	{
		if (fabsf(_pitch_offset_rad - new_pitch_offset) > FLT_EPSILON) {
			_sin_pitch_offset = sinf(new_pitch_offset);
			_cos_pitch_offset = cosf(new_pitch_offset);
			_pitch_offset_rad = new_pitch_offset;
		}
	}

	/***************************
		允许使用测距仪和流量数据的垂直方向最大倾斜角度的余弦
	***************************/
	void setCosMaxTilt(float cos_max_tilt) { _range_cos_max_tilt = cos_max_tilt; }

	/***************************
		设置有效值范围
	***************************/
	void setLimits(float min_distance, float max_distance) {
		_rng_valid_min_val = min_distance;
		_rng_valid_max_val = max_distance;
	}

	float getCosTilt() const { return _cos_tilt_rng_to_earth; }

	void setRange(float rng) { _sample.rng = rng; }
	float getRange() const { return _sample.rng; }

	float getDistBottom() const { return _sample.rng * _cos_tilt_rng_to_earth; }

	void setDataReadiness(bool is_ready) { _is_sample_ready = is_ready; }
	void setValidity(bool is_valid) { _is_sample_valid = is_valid; }

	float getValidMinVal() const { return _rng_valid_min_val; }
	float getValidMaxVal() const { return _rng_valid_max_val; }

private:
	void updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth);

	void updateValidity(uint64_t current_time_us);

	/***************************
		对采样周期的lpf滤波器
	***************************/
	void updateDtDataLpf(uint64_t current_time_us);

	/***************************
		测距传感器更新是否超时
	***************************/
	bool isSampleOutOfDate(uint64_t current_time_us) const;

	/***************************
		测距传感器是否连续 (周期是否<2s?)
	***************************/
	bool isDataContinuous() const { return _dt_data_lpf < 2e6f; }

	/***************************
		传感器倾斜角度是否在允许范围内?
	***************************/
	bool isTiltOk() const { return _cos_tilt_rng_to_earth > _range_cos_max_tilt; }

	/***************************
		传感器数据是否在范围内??
	***************************/
	bool isDataInRange() const;

	/***************************
		检测测距传感器的值是否被卡住?
	***************************/
	void updateStuckCheck();

	rangeSample _sample{};

	bool _is_sample_ready{};	///< true when new range finder data has fallen behind the fusion time horizon and is available to be fused
	bool _is_sample_valid{};	///< true if range finder sample retrieved from buffer is valid
	uint64_t _time_last_valid_us{};	///< time the last range finder measurement was ready (uSec)

	/*
	 * Stuck check
	 */
	bool _is_stuck{};
	float _stuck_threshold{0.1f};	///< minimum variation in range finder reading required to declare a range finder 'unstuck' when readings recommence after being out of range (m)
	float _stuck_min_val{};		///< minimum value for new rng measurement when being stuck
	float _stuck_max_val{};		///< maximum value for new rng measurement when being stuck

	/*
	 * Data regularity check
	 */
	static constexpr float _dt_update{0.01f}; 	///< delta time since last ekf update TODO: this should be a parameter
	float _dt_data_lpf{};	///< filtered value of the delta time elapsed since the last range measurement came into the filter (uSec)

	/*
	 * Tilt check
	 */
	float _cos_tilt_rng_to_earth{};		///< 2,2 element of the rotation matrix from sensor frame to earth frame
	float _range_cos_max_tilt{0.7071f};	///< cosine of the maximum tilt angle from the vertical that permits use of range finder and flow data
	float _pitch_offset_rad{3.14f}; 		///< range finder tilt rotation about the Y body axis
	float _sin_pitch_offset{0.0f}; 		///< sine of the range finder tilt rotation about the Y body axis
	float _cos_pitch_offset{-1.0f}; 		///< cosine of the range finder tilt rotation about the Y body axis

	/*
	 * Range check
	 */
	float _rng_valid_min_val{};	///< minimum distance that the rangefinder can measure (m)
	float _rng_valid_max_val{};	///< maximum distance that the rangefinder can measure (m)

	/*
	 * Quality check
	 */
	uint64_t _time_bad_quality_us{};	///< timestamp at which range finder signal quality was 0 (used for hysteresis)
	uint64_t _quality_hyst_us{1000000}; 	///< minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (us)
};

} // namespace sensor
} // namespace estimator
