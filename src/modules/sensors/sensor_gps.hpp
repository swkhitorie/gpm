#pragma once

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
class SensorGPSPosition
{
public:
	SensorGPSPosition() = default;
	~SensorGPSPosition() = default;

	bool start();

    void run();
    
    bool param_update();

private:
    
    uORB::uORBtopicsTypeMap<uORB::TOPIC_VEHICLE_GPS_POSITION>::type _pub_vehicle_gps_position;
    uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_GPS>::type *_sub_sensor_gps;
    hrt_abstime _gps_timestamp_sample_last{0};
//	GpsBlending _gps_blending;

};
}; // namespace sensors
