#include "sensor_gps.hpp"

namespace sensors
{
bool SensorGPSPosition::param_update() { return true; }
bool SensorGPSPosition::start() 
{ 
    uorb_subscribe_topics<uORB::TOPIC_SENSOR_GPS>(&_sub_sensor_gps);
    return param_update(); 
}

//300ms
void SensorGPSPosition::run()
{
    if (_sub_sensor_gps->timestamp > _gps_timestamp_sample_last) {
        _gps_timestamp_sample_last = _sub_sensor_gps->timestamp;
        _pub_vehicle_gps_position.timestamp = _sub_sensor_gps->timestamp;
        _pub_vehicle_gps_position.time_utc_usec = _sub_sensor_gps->time_utc_usec;
        _pub_vehicle_gps_position.latitude_deg = _sub_sensor_gps->latitude_deg;
        _pub_vehicle_gps_position.longitude_deg = _sub_sensor_gps->longitude_deg;
        _pub_vehicle_gps_position.altitude_msl_m = _sub_sensor_gps->altitude_msl_m;
        _pub_vehicle_gps_position.altitude_ellipsoid_m = _sub_sensor_gps->altitude_ellipsoid_m;
        _pub_vehicle_gps_position.lat = _sub_sensor_gps->lat;
        _pub_vehicle_gps_position.lon = _sub_sensor_gps->lon;
        _pub_vehicle_gps_position.alt = _sub_sensor_gps->alt;
        _pub_vehicle_gps_position.alt_ellipsoid = _sub_sensor_gps->alt_ellipsoid;
        _pub_vehicle_gps_position.s_variance_m_s = _sub_sensor_gps->s_variance_m_s;
        _pub_vehicle_gps_position.c_variance_rad = _sub_sensor_gps->c_variance_rad;
        _pub_vehicle_gps_position.eph = _sub_sensor_gps->eph;
        _pub_vehicle_gps_position.epv = _sub_sensor_gps->epv;
        _pub_vehicle_gps_position.hdop = _sub_sensor_gps->hdop;
        _pub_vehicle_gps_position.vdop = _sub_sensor_gps->vdop;
        _pub_vehicle_gps_position.noise_per_ms = _sub_sensor_gps->noise_per_ms;
        _pub_vehicle_gps_position.jamming_indicator = _sub_sensor_gps->jamming_indicator;
        _pub_vehicle_gps_position.vel_m_s = _sub_sensor_gps->vel_m_s;
        _pub_vehicle_gps_position.vel_n_m_s = _sub_sensor_gps->vel_n_m_s;
        _pub_vehicle_gps_position.vel_e_m_s = _sub_sensor_gps->vel_e_m_s;
        _pub_vehicle_gps_position.vel_d_m_s = _sub_sensor_gps->vel_d_m_s;
        _pub_vehicle_gps_position.cog_rad = _sub_sensor_gps->cog_rad;
        _pub_vehicle_gps_position.timestamp_time_relative = _sub_sensor_gps->timestamp_time_relative;
        _pub_vehicle_gps_position.heading = _sub_sensor_gps->heading;
        _pub_vehicle_gps_position.heading_offset = _sub_sensor_gps->heading_offset;
        _pub_vehicle_gps_position.fix_type = _sub_sensor_gps->fix_type;
        _pub_vehicle_gps_position.jamming_state = _sub_sensor_gps->jamming_state;
        _pub_vehicle_gps_position.vel_ned_valid = _sub_sensor_gps->vel_ned_valid;
        _pub_vehicle_gps_position.satellites_used = _sub_sensor_gps->satellites_used;
        _pub_vehicle_gps_position.selected = 0;
        uorb_publish_topics<uORB::TOPIC_VEHICLE_GPS_POSITION>(&_pub_vehicle_gps_position);        
    }

}

}; // namespace sensors
