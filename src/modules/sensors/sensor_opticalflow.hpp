#pragma once

#include "sensor_calibration/accel_calibrator_bak.h"
#include "sensor_calibration/gyro_calibrator.hpp"
#include "mathlib/math/Limits.hpp"
#include "matrix/matrix/math.hpp"
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

class VehicleOpticalFlow
{
public:
	VehicleOpticalFlow();
	~VehicleOpticalFlow() = default;

	bool start();
	
	void run();
	
	void param_update();
	
private:
	void clear_accumulated();
	void update_distancesensor();
	void update_sensorgyro();

	uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_OPTICAL_FLOW>::type _pub_vehicle_optical_flow;
	uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_OPTICAL_FLOW_VEL>::type _pub_vehicle_optical_flow_vel;

	uORB::uORBtopicsTypeMap<uORB::TOPIC_DISTANCE_SENSOR>::type *_sub_distance_sensor;
	uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_ATTITUDE>::type *_sub_vehicle_attitude;
	uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_OPTICALFLOW>::type *_sub_sensor_opticalflow;
	uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_GYRO>::type *_sub_sensor_gyro;
	uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_SELECTION>::type *_sub_sensor_selection;

	sensors::IntegratorConing _gyro_integrator{};
	hrt_abstime _gyro_timestamp_sample_last{0};
	calibration::Gyroscope _gyro_calibration{};

	matrix::Dcmf _flow_rotation{matrix::eye<float, 3>()};
	hrt_abstime _flow_timestamp_sample_last{0};
	
	matrix::Vector2f _flow_integral{};
	matrix::Vector3f _delta_angle{};
	uint32_t _integration_timespan_us{};
	float _distance_sum{NAN};
	uint8_t _distance_sum_count{0};
	uint16_t _quality_sum{0};
	uint8_t _accumulated_count{0};

	int _distance_sensor_selected{-1};
	hrt_abstime _last_range_sensor_update{0};
	bool _delta_angle_available{false};

	struct gyroSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		matrix::Vector3f data{};
		float dt{0.f};
	};

	struct rangeSample {
		uint64_t time_us{}; ///< timestamp of the measurement (uSec)
		float data{};
	};

	RingBufferSensor<gyroSample, 32> _gyro_buffer{};
	RingBufferSensor<rangeSample, 5> _range_buffer{};

	ParamInt<UParams::SENS_FLOW_ROT> _param_sens_flow_rot;
	ParamFloat<UParams::SENS_FLOW_MINHGT> _param_sens_flow_minhgt;
	ParamFloat<UParams::SENS_FLOW_MAXHGT> _param_sens_flow_maxhgt;
	ParamFloat<UParams::SENS_FLOW_MAXR> _param_sens_flow_maxr;
	ParamFloat<UParams::SENS_FLOW_RATE> _param_sens_flow_rate;
	
};
}; // namespace sensors
