#ifndef UORB_TOPIC_DEFINE_H_
#define UORB_TOPIC_DEFINE_H_

#include "string.h"
#include "topics/actuator_armed.h"
#include "topics/distance_sensor.h"
#include "topics/ekf2_timestamps.h"
#include "topics/estimator_bias.h"
#include "topics/estimator_bias_3d.h"
#include "topics/estimator_event_flags.h"
#include "topics/estimator_innovations.h"
#include "topics/estimator_selector_status.h"
#include "topics/estimator_sensor_bias.h"
#include "topics/estimator_states.h"
#include "topics/estimator_status.h"
#include "topics/estimator_status_flags.h"
#include "topics/landing_target_pose.h"
#include "topics/magnetometer_bias_estimate.h"
#include "topics/parameter_update.h"
#include "topics/sensor_accel.h"
#include "topics/sensor_baro.h"
#include "topics/sensor_combined.h"
#include "topics/sensor_gps.h"
#include "topics/sensor_gyro.h"
#include "topics/sensor_gyro_fft.h"
#include "topics/sensor_gyro_fifo.h"
#include "topics/sensor_mag.h"
#include "topics/sensor_opticalflow.h"
#include "topics/sensor_preflight_mag.h"
#include "topics/sensor_selection.h"
#include "topics/sensors_status.h"
#include "topics/sensors_status_imu.h"
#include "topics/vehicle_acceleration.h"
#include "topics/vehicle_air_data.h"
#include "topics/vehicle_angular_velocity.h"
#include "topics/vehicle_attitude.h"
#include "topics/vehicle_attitude_setpoint.h"
#include "topics/vehicle_command.h"
#include "topics/vehicle_control_mode.h"
#include "topics/vehicle_global_position.h"
#include "topics/vehicle_gps_position.h"
#include "topics/vehicle_imu.h"
#include "topics/vehicle_imu_status.h"
#include "topics/vehicle_land_detected.h"
#include "topics/vehicle_local_position.h"
#include "topics/vehicle_local_position_setpoint.h"
#include "topics/vehicle_magnetometer.h"
#include "topics/vehicle_odometry.h"
#include "topics/vehicle_optical_flow.h"
#include "topics/vehicle_optical_flow_vel.h"
#include "topics/vehicle_status.h"
#include "topics/wind.h"
#include "topics/yaw_estimator_status.h"

#include "topics/airspeed.h"
#include "topics/ekf_gps_drift.h"
#include "topics/ekf_gps_position.h"
#include "topics/optical_flow.h"
#include "topics/wind_estimate.h"

#define ORB_NAME( _name, topics_name ) topics_name
namespace uORB
{
typedef enum
{
    ORB_NAME( actuator_armed_s,             TOPIC_ACTUATOR_ARMED ) = 0x00,
    ORB_NAME( distance_sensor_s,            TOPIC_DISTANCE_SENSOR ),
    ORB_NAME( ekf2_timestamps_s,            TOPIC_EKF2_TIMESTAMPS ),
    ORB_NAME( estimator_bias_s,             TOPIC_ESTIMATOR_BIAS ),
    ORB_NAME( estimator_bias_3d_s,          TOPIC_ESTIMATOR_BIAS_3D ),
    ORB_NAME( estimator_event_flags_s,      TOPIC_ESTIMATOR_EVENT_FLAGS ),
    ORB_NAME( estimator_innovations_s,      TOPIC_ESTIMATOR_INNOVATIONS ),
    ORB_NAME( estimator_selector_status_s,  TOPIC_ESTIMATOR_SELECTOR_STATUS ),
    ORB_NAME( estimator_sensor_bias_s,      TOPIC_ESTIMATOR_SENSOR_BIAS ),
    ORB_NAME( estimator_states_s,           TOPIC_ESTIMATOR_STATES ),
    ORB_NAME( estimator_status_s,           TOPIC_ESTIMATOR_STATUS ),
    ORB_NAME( estimator_status_flags_s,     TOPIC_ESTIMATOR_STATUS_FLAGS ),
    ORB_NAME( landing_target_pose_s,        TOPIC_LANDING_TARGET_POSE ),
    ORB_NAME( magnetometer_bias_estimate_s, TOPIC_MAGNETOMETER_BIAS_ESTIMATE ),
    ORB_NAME( parameter_update_s,           TOPIC_PARAMETER_UPDATE ),
    ORB_NAME( sensor_accel_s,               TOPIC_SENSOR_ACCEL ),
    ORB_NAME( sensor_baro_s,                TOPIC_SENSOR_BARO ),
    ORB_NAME( sensor_combined_s,            TOPIC_SENSOR_COMBINED ),
    ORB_NAME( sensor_gps_s,                 TOPIC_SENSOR_GPS ),
    ORB_NAME( sensor_gyro_s,                TOPIC_SENSOR_GYRO ),
    ORB_NAME( sensor_gyro_fft_s,            TOPIC_SENSOR_GYRO_FFT ),
    ORB_NAME( sensor_gyro_fifo_s,           TOPIC_SENSOR_GYRO_FIFO ),
    ORB_NAME( sensor_mag_s,                 TOPIC_SENSOR_MAG ),
    ORB_NAME( sensor_opticalflow_s,         TOPIC_SENSOR_OPTICALFLOW ),
    ORB_NAME( sensor_preflight_mag_s,       TOPIC_SENSOR_PREFLIGHT_MAG),
    ORB_NAME( sensor_selections_s,          TOPIC_SENSOR_SELECTION ),
    ORB_NAME( sensors_status_s,             TOPIC_SENSOR_STATUS ),
    ORB_NAME( sensors_status_imu_s,         TOPIC_SENSORS_STATUS_IMU ),
    ORB_NAME( vehicle_acceleration_s,       TOPIC_VEHICLE_ACCELERATION),
    ORB_NAME( vehicle_air_data_s,           TOPIC_VEHICLE_AIR_DATA ),
    ORB_NAME( vehicle_angular_velocity_s,   TOPIC_VEHICLE_ANGULAR_VELOCITY ),
    ORB_NAME( vehicle_attitude_s,           TOPIC_VEHICLE_ATTITUDE ),
    ORB_NAME( vehicle_attitude_setpoint_s,  TOPIC_VEHICLE_ATTITUDE_SETPOINT ),
    ORB_NAME( vehicle_command_s,            TOPIC_VEHICLE_COMMAND ),
    ORB_NAME( vehicle_control_mode_s,       TOPIC_VEHICLE_CONTROL_MODE ),
    ORB_NAME( vehicle_global_position_s,    TOPIC_VEHICLE_GLOBAL_POSITION ),
    ORB_NAME( vehicle_gps_position_s,       TOPIC_VEHICLE_GPS_POSITION ),
    ORB_NAME( vehicle_imu_s,                TOPIC_VEHICLE_IMU ),
    ORB_NAME( vehicle_imu_status_s,         TOPIC_VEHICLE_IMU_STATUS ),
    ORB_NAME( vehicle_land_detected_s,      TOPIC_VEHICLE_LAND_DETECTED ),
    ORB_NAME( vehicle_local_position_s,     TOPIC_VEHICLE_LOCAL_POSITION ),
    ORB_NAME( vehicle_local_position_setpoint_s,  TOPIC_VEHICLE_LOCAL_POSITION_SETPOINT ),
    ORB_NAME( vehicle_magnetometer_s,       TOPIC_VEHICLE_MAGNETOMETER ),
    ORB_NAME( vehicle_odometry_s,           TOPIC_VEHICLE_ODOMETRY ),
    ORB_NAME( vehicle_optical_flow_s,       TOPIC_VEHICLE_OPTICAL_FLOW ),
    ORB_NAME( vehicle_optical_flow_vel_s,   TOPIC_VEHICLE_OPTICAL_FLOW_VEL ),
    ORB_NAME( vehicle_status_s,             TOPIC_VEHICLE_STATUS ),
    ORB_NAME( wind_s,                       TOPIC_WIND ),
    ORB_NAME( yaw_estimator_status_s,       TOPIC_YAW_ESTIMATOR_STATUS ),
    ORB_NAME( airspeed_s,                   TOPIC_AIRSPEED ),
    ORB_NAME( ekf_gps_drift_s,              TOPIC_EKF_GPS_DRIFT ),
    ORB_NAME( ekf_gps_position_s,           TOPIC_EKF_GPS_POSITION ),
    ORB_NAME( optical_flow_s,               TOPIC_OPTICAL_FLOW ),
    ORB_NAME( wind_estimate_s,              TOPIC_WIND_ESTIMATE ),
    TOTAL_TOPICS,
} uORBTopicsName;

template <uORB::uORBTopicsName T>
struct uORBtopicsTypeMap { typedef void type; };

#define ORB_DECALRE( _name, topics_name ) extern uORB::uORBtopicsTypeMap<uORB::topics_name>::type __orb_##_name;
#define ORB_DEFINE( _name, topics_name) uORB::uORBtopicsTypeMap<uORB::topics_name>::type __orb_##_name;
#define ORB_REFERENCE( _name, topics_name) reinterpret_cast<void *>(&__orb_##_name)
#define ORB_TYPEMAP_DECLARE( _name, topics_name ) \
        template <> struct uORBtopicsTypeMap<topics_name> { typedef struct _name type;};

ORB_TYPEMAP_DECLARE( actuator_armed_s,             TOPIC_ACTUATOR_ARMED );
ORB_TYPEMAP_DECLARE( distance_sensor_s,            TOPIC_DISTANCE_SENSOR );
ORB_TYPEMAP_DECLARE( ekf2_timestamps_s,            TOPIC_EKF2_TIMESTAMPS );
ORB_TYPEMAP_DECLARE( estimator_bias_s,             TOPIC_ESTIMATOR_BIAS );
ORB_TYPEMAP_DECLARE( estimator_bias_3d_s,          TOPIC_ESTIMATOR_BIAS_3D );
ORB_TYPEMAP_DECLARE( estimator_event_flags_s,      TOPIC_ESTIMATOR_EVENT_FLAGS );
ORB_TYPEMAP_DECLARE( estimator_innovations_s,      TOPIC_ESTIMATOR_INNOVATIONS );
ORB_TYPEMAP_DECLARE( estimator_selector_status_s,  TOPIC_ESTIMATOR_SELECTOR_STATUS );
ORB_TYPEMAP_DECLARE( estimator_sensor_bias_s,      TOPIC_ESTIMATOR_SENSOR_BIAS );
ORB_TYPEMAP_DECLARE( estimator_states_s,           TOPIC_ESTIMATOR_STATES );
ORB_TYPEMAP_DECLARE( estimator_status_s,           TOPIC_ESTIMATOR_STATUS );
ORB_TYPEMAP_DECLARE( estimator_status_flags_s,     TOPIC_ESTIMATOR_STATUS_FLAGS );
ORB_TYPEMAP_DECLARE( landing_target_pose_s,        TOPIC_LANDING_TARGET_POSE );
ORB_TYPEMAP_DECLARE( magnetometer_bias_estimate_s, TOPIC_MAGNETOMETER_BIAS_ESTIMATE );
ORB_TYPEMAP_DECLARE( parameter_update_s,           TOPIC_PARAMETER_UPDATE );
ORB_TYPEMAP_DECLARE( sensor_accel_s,               TOPIC_SENSOR_ACCEL );
ORB_TYPEMAP_DECLARE( sensor_baro_s,                TOPIC_SENSOR_BARO );
ORB_TYPEMAP_DECLARE( sensor_combined_s,            TOPIC_SENSOR_COMBINED );
ORB_TYPEMAP_DECLARE( sensor_gps_s,                 TOPIC_SENSOR_GPS );
ORB_TYPEMAP_DECLARE( sensor_gyro_s,                TOPIC_SENSOR_GYRO );
ORB_TYPEMAP_DECLARE( sensor_gyro_fft_s,            TOPIC_SENSOR_GYRO_FFT );
ORB_TYPEMAP_DECLARE( sensor_gyro_fifo_s,           TOPIC_SENSOR_GYRO_FIFO );
ORB_TYPEMAP_DECLARE( sensor_mag_s,                 TOPIC_SENSOR_MAG );
ORB_TYPEMAP_DECLARE( sensor_opticalflow_s,         TOPIC_SENSOR_OPTICALFLOW );
ORB_TYPEMAP_DECLARE( sensor_preflight_mag_s,       TOPIC_SENSOR_PREFLIGHT_MAG);
ORB_TYPEMAP_DECLARE( sensor_selections_s,          TOPIC_SENSOR_SELECTION );
ORB_TYPEMAP_DECLARE( sensors_status_s,             TOPIC_SENSOR_STATUS );
ORB_TYPEMAP_DECLARE( sensors_status_imu_s,         TOPIC_SENSORS_STATUS_IMU );
ORB_TYPEMAP_DECLARE( vehicle_acceleration_s,       TOPIC_VEHICLE_ACCELERATION);
ORB_TYPEMAP_DECLARE( vehicle_air_data_s,           TOPIC_VEHICLE_AIR_DATA );
ORB_TYPEMAP_DECLARE( vehicle_angular_velocity_s,   TOPIC_VEHICLE_ANGULAR_VELOCITY );
ORB_TYPEMAP_DECLARE( vehicle_attitude_s,           TOPIC_VEHICLE_ATTITUDE );
ORB_TYPEMAP_DECLARE( vehicle_attitude_setpoint_s,  TOPIC_VEHICLE_ATTITUDE_SETPOINT );
ORB_TYPEMAP_DECLARE( vehicle_command_s,            TOPIC_VEHICLE_COMMAND );
ORB_TYPEMAP_DECLARE( vehicle_control_mode_s,       TOPIC_VEHICLE_CONTROL_MODE );
ORB_TYPEMAP_DECLARE( vehicle_global_position_s,    TOPIC_VEHICLE_GLOBAL_POSITION );
ORB_TYPEMAP_DECLARE( vehicle_gps_position_s,       TOPIC_VEHICLE_GPS_POSITION );
ORB_TYPEMAP_DECLARE( vehicle_imu_s,                TOPIC_VEHICLE_IMU );
ORB_TYPEMAP_DECLARE( vehicle_imu_status_s,         TOPIC_VEHICLE_IMU_STATUS );
ORB_TYPEMAP_DECLARE( vehicle_land_detected_s,      TOPIC_VEHICLE_LAND_DETECTED );
ORB_TYPEMAP_DECLARE( vehicle_local_position_s,     TOPIC_VEHICLE_LOCAL_POSITION );
ORB_TYPEMAP_DECLARE( vehicle_local_position_setpoint_s,  TOPIC_VEHICLE_LOCAL_POSITION_SETPOINT );
ORB_TYPEMAP_DECLARE( vehicle_magnetometer_s,       TOPIC_VEHICLE_MAGNETOMETER );
ORB_TYPEMAP_DECLARE( vehicle_odometry_s,           TOPIC_VEHICLE_ODOMETRY );
ORB_TYPEMAP_DECLARE( vehicle_optical_flow_s,       TOPIC_VEHICLE_OPTICAL_FLOW );
ORB_TYPEMAP_DECLARE( vehicle_optical_flow_vel_s,   TOPIC_VEHICLE_OPTICAL_FLOW_VEL );
ORB_TYPEMAP_DECLARE( vehicle_status_s,             TOPIC_VEHICLE_STATUS );
ORB_TYPEMAP_DECLARE( wind_s,                       TOPIC_WIND );
ORB_TYPEMAP_DECLARE( yaw_estimator_status_s,       TOPIC_YAW_ESTIMATOR_STATUS );
ORB_TYPEMAP_DECLARE( airspeed_s,                   TOPIC_AIRSPEED );
ORB_TYPEMAP_DECLARE( ekf_gps_drift_s,              TOPIC_EKF_GPS_DRIFT );
ORB_TYPEMAP_DECLARE( ekf_gps_position_s,           TOPIC_EKF_GPS_POSITION );
ORB_TYPEMAP_DECLARE( optical_flow_s,               TOPIC_OPTICAL_FLOW );
ORB_TYPEMAP_DECLARE( wind_estimate_s,              TOPIC_WIND_ESTIMATE );
}

extern void *uorb_topics_list[uORB::TOTAL_TOPICS];

template<uORB::uORBTopicsName topic>
void uorb_subscribe_topics(typename uORB::uORBtopicsTypeMap<topic>::type **data)
{
    *data = static_cast<typename uORB::uORBtopicsTypeMap<topic>::type *>(uorb_topics_list[topic]);
}

template<uORB::uORBTopicsName topic>
typename uORB::uORBtopicsTypeMap<topic>::type uorb_get_topics()
{
    typename uORB::uORBtopicsTypeMap<topic>::type ans;
    typename uORB::uORBtopicsTypeMap<topic>::type *psrc = 
        static_cast<typename uORB::uORBtopicsTypeMap<topic>::type *>(uorb_topics_list[topic]);
    memcpy(&ans, psrc, sizeof (typename uORB::uORBtopicsTypeMap<topic>::type));
    
    return ans;
}

template<uORB::uORBTopicsName topic>
void uorb_publish_topics(typename uORB::uORBtopicsTypeMap<topic>::type *pdata)
{
    typename uORB::uORBtopicsTypeMap<topic>::type *psrc = 
        static_cast<typename uORB::uORBtopicsTypeMap<topic>::type *>(uorb_topics_list[topic]);
    memcpy(psrc, pdata, sizeof (typename uORB::uORBtopicsTypeMap<topic>::type));
}    

/* 
    ##### publish #####
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_ACCEL>::type _accel;
        _accel.device_id = 52;
        _accel.samples = 245;
        uorb_publish_topics<uORB::TOPIC_SENSOR_ACCEL>(&_accel);
   
    ##### subscribe #####
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_ACCEL>::type *_rcv_accel;
        uorb_subscribe_topics<uORB::TOPIC_SENSOR_ACCEL>(&_rcv_accel);
   
    ##### get value #####
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_ACCEL>::type _rcv_accel;
        _rcv_accel = uorb_get_topics<uORB::TOPIC_SENSOR_ACCEL>();
*/

#endif
