
#ifndef __NMEA0813_PARSER_H_
#define __NMEA0813_PARSER_H_
#include <stdint.h>
#include <cstring>

namespace ESAF
{
struct _gps_sensor_t {
	uint64_t timestamp;
	uint64_t time_utc_usec;
	uint32_t device_id;
    double latitude_deg;
    double longitude_deg;
    double altitude_msl_m;
    double altitude_ellipsoid_m;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t alt_ellipsoid;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad; 
	int32_t timestamp_time_relative;
	float heading;
	float heading_offset;
	float heading_accuracy;
	uint16_t automatic_gain_control;
	uint8_t fix_type;
    //0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 
    //6: Real-Time Kinematic, fixed, 
    //8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	uint8_t jamming_state;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t _padding0[2]; // required for logger
};

class nmea0813_parser
{
  public:
    /**
     * @brief 	calculate the nmea0813 frame checksum
     * @param buf the data buffer that start from '$'
     * @return uint8_t the checksum of nmea0813 frame
     */
    static uint8_t nmea_sum_check(const uint8_t *buf);
    static uint32_t nmea_pow(uint8_t base, uint8_t order);
    static uint8_t stringhex_to_num(const uint8_t *buf, uint8_t dx);
  
    /*! @brief depack GNGGA message
     *  @param buf original bytestream
     *  @param gpsx output msg
     */
    static int handleMessage(const uint8_t *rxbuf, uint16_t len);

    static uint8_t cmd_turnon_gga[11];
    static uint8_t cmd_turnoff_gga[11];
    
    static uint8_t cmd_turnon_gsa[11];
    static uint8_t cmd_turnoff_gsa[11];
    
    static uint8_t cmd_turnon_rmc[11];
    static uint8_t cmd_turnoff_rmc[11];
    
    static uint8_t cmd_turnon_vtg[11];
    static uint8_t cmd_turnoff_vtg[11];

    static uint8_t cmd_turnon_gst[11];
    static uint8_t cmd_turnoff_gst[11];
    
    static struct _gps_sensor_t _gps_position;
};

}  // namespace ESAF

#endif  // nmea0813_parser_H
