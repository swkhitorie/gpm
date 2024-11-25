/**
 * Downsamples IMU data to a lower rate such that EKF predicition can happen less frequent
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include "common.h"

using namespace estimator;

class ImuDownSampler
{
public:
	explicit ImuDownSampler(float target_dt_sec);
	~ImuDownSampler() = default;

	/***************************
		更新IMU采样值, 并开始累积
	***************************/
	bool update(const imuSample &imu_sample_new);

	/***************************
		获取IMU平均采样值, 并重新开始累积
	***************************/
	imuSample getDownSampledImuAndTriggerReset()
	{
		_do_reset = true;
		return _imu_down_sampled;
	}

private:
	void reset();

	imuSample _imu_down_sampled{};
	Quatf _delta_angle_accumulated{};
	const float _target_dt;  // [sec]
	float _imu_collection_time_adj{0.f};
	bool _do_reset{true};
};
