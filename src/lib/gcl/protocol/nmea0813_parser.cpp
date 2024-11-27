#include "nmea0813_parser.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
using namespace ESAF;

uint8_t nmea0813_parser::cmd_turnon_gga[11] =  {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10};
uint8_t nmea0813_parser::cmd_turnoff_gga[11] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F};

uint8_t nmea0813_parser::cmd_turnon_gsa[11] =  {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x01, 0xFD, 0x14};
uint8_t nmea0813_parser::cmd_turnoff_gsa[11] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};

uint8_t nmea0813_parser::cmd_turnon_rmc[11] =  {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x01, 0x00, 0x1A};
uint8_t nmea0813_parser::cmd_turnoff_rmc[11] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};

uint8_t nmea0813_parser::cmd_turnon_vtg[11] =  {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x01, 0x01, 0x1C};
uint8_t nmea0813_parser::cmd_turnoff_vtg[11] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B};

uint8_t nmea0813_parser::cmd_turnon_gst[11] =  {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x01, 0x03, 0x20};
uint8_t nmea0813_parser::cmd_turnoff_gst[11] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F}; 

struct _gps_sensor_t nmea0813_parser::_gps_position;

uint8_t nmea0813_parser::nmea_sum_check(const uint8_t *buf)
{
    uint8_t i      = 0;
    uint8_t result = buf[1];
    for (i = 2; buf[i] != '*'; i++)
        result ^= buf[i];
    return result;
}

uint32_t nmea0813_parser::nmea_pow(uint8_t base, uint8_t order)
{
    uint32_t result = 1;
    while (order--)
        result *= base;
    return result;
}

uint8_t nmea0813_parser::stringhex_to_num(const uint8_t *buf, uint8_t dx)
{
    uint8_t result = 0;
    for (uint8_t i = 0; i < dx; i++)
    {
        if (buf[i] >= '0' && buf[i] <= '9')
        {
            result += (buf[i] - '0') * nmea_pow(16, (dx - i - 1));
        }
        else if (buf[i] >= 'A' && buf[i] <= 'F')
        {
            result += (buf[i] - 'A' + 10) * nmea_pow(16, (dx - i - 1));
        }
    }
    return result;
}

int nmea0813_parser::handleMessage(const uint8_t *rxbuf, uint16_t len)
{
	uint8_t _sat_num_gga{0};
	uint8_t _sat_num_gns{0};
	uint8_t _sat_num_gsv{0};
	uint8_t _sat_num_gpgsv{0};
	uint8_t _sat_num_glgsv{0};
	uint8_t _sat_num_gagsv{0};
	uint8_t _sat_num_gbgsv{0};
	uint8_t _sat_num_bdgsv{0};
    
    char *endp;

	if (len < 7) {
		return 0;
	}
    
    int uiCalcComma = 0;
    
	for (int i = 0 ; i < len; i++) {
		if (rxbuf[i] == ',') { uiCalcComma++; }
	}
    
    char *bufptr = (char *)(rxbuf + 6);   
    int ret = 0;

    if ((memcmp(rxbuf + 3, "RMC,", 4) == 0) && (uiCalcComma >= 11)) {
        
		/*
		Position, velocity, and time
		The RMC string is:

		$xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
		The Talker ID ($--) will vary depending on the satellite system used for the position solution:
		$GNRMC,092721.00,A,2926.688113,N,11127.771644,E,0.780,,200520,,,D,V*1D

		GPRMC message fields
		Field	Meaning
		0	Message ID $GPRMC
		1	UTC of position fix
		2	Status A=active or V=void
		3	Latitude
		4	Longitude
		5	Speed over the ground in knots
		6	Track angle in degrees (True)
		7	Date
		8	Magnetic variation in degrees
		9	The checksum data, always begins with *
		*/
        double utc_time = 0.0;
        char Status = 'V';
        double lat = 0.0, lon = 0.0;
        float ground_speed_K = 0.f;
        float track_true = 0.f;
        int nmea_date = 0;
        float Mag_var = 0.f;
        char ns = '?', ew = '?';
        
        if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }
        
		if (bufptr && *(++bufptr) != ',') { Status = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { ground_speed_K = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { track_true = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { nmea_date = static_cast<int>(strtol(bufptr, &endp, 10)); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { Mag_var = strtof(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}
        
		if (Status == 'V') {
			_gps_position.fix_type = 0;
		}
        
		float track_rad = track_true * 3.14159265f / 180.0f; // rad in range [0, 2pi]

		if (track_rad > 3.14159265f) {
			track_rad -= 2.f * 3.14159265f; // rad in range [-pi, pi]
		}
        
		float velocity_ms = ground_speed_K / 1.9438445f;
		float velocity_north = velocity_ms * cosf(track_rad);
		float velocity_east  = velocity_ms * sinf(track_rad);
        
        _gps_position.latitude_deg = int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0;
        _gps_position.longitude_deg = int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0;
		_gps_position.vel_m_s = velocity_ms;
		_gps_position.vel_n_m_s = velocity_north;
		_gps_position.vel_e_m_s = velocity_east;
		_gps_position.cog_rad = track_rad;
		_gps_position.vel_ned_valid = true; /**< Flag to indicate if NED speed is valid */
		_gps_position.c_variance_rad = 0.1f;
		_gps_position.s_variance_m_s = 0;
        
        // EDEBUG("lat: %.8f, lon:%.8f\n", _gps_position.latitude_deg, _gps_position.longitude_deg);
        
    } else if ((memcmp(rxbuf + 3, "VTG,", 4) == 0) && (uiCalcComma >= 8)) {
        
		/*$GNVTG,,T,,M,0.683,N,1.265,K*30
		  $GNVTG,,T,,M,0.780,N,1.445,K*33

		Field	Meaning
		0	Message ID $GPVTG
		1	Track made good (degrees true)
		2	T: track made good is relative to true north
		3	Track made good (degrees magnetic)
		4	M: track made good is relative to magnetic north
		5	Speed, in knots
		6	N: speed is measured in knots
		7	Speed over ground in kilometers/hour (kph)
		8	K: speed over ground is measured in kph
		9	The checksum data, always begins with *
		*/ 
        
		float track_true = 0.f;
		char T;
		float track_mag = 0.f;
		char M;
		float ground_speed = 0.f;
		char N;
		float ground_speed_K = 0.f;
		char K;
        
		if (bufptr && *(++bufptr) != ',') {track_true = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { T = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') {track_mag = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { M = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { ground_speed = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { N = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { ground_speed_K = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { K = *(bufptr++); }

		float track_rad = track_true * 3.14159265f / 180.0f; // rad in range [0, 2pi]

		if (track_rad > 3.14159265f) {
			track_rad -= 2.f * 3.14159265f; // rad in range [-pi, pi]
		}

		float velocity_ms = ground_speed / 1.9438445f;
		float velocity_north = velocity_ms * cosf(track_rad);
		float velocity_east  = velocity_ms * sinf(track_rad);

		_gps_position.vel_m_s = velocity_ms;
		_gps_position.vel_n_m_s = velocity_north;
		_gps_position.vel_e_m_s = velocity_east;
		_gps_position.cog_rad = track_rad;
		_gps_position.vel_ned_valid = true; /** Flag to indicate if NED speed is valid */
		_gps_position.c_variance_rad = 0.1f;
		_gps_position.s_variance_m_s = 0;
        
        // EDEBUG("vel: %f, vel_n: %f, vel_e: %f\n", _gps_position.vel_m_s, _gps_position.vel_n_m_s, _gps_position.vel_e_m_s);
        
    } else if ((memcmp(rxbuf + 3, "GGA,", 4) == 0) && (uiCalcComma >= 14)) {
		/*
		  Time, position, and fix related data
		  An example of the GBS message string is:
		  $xxGGA,time,lat,NS,long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
		  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F
		  $GNGGA,092721.00,2926.688113,N,11127.771644,E,2,08,1.11,106.3,M,-20,M,1.0,3721*53

		  Note - The data string exceeds the nmea standard length.
		  GGA message fields
		  Field   Meaning
		  0   Message ID $GPGGA
		  1   UTC of position fix
		  2   Latitude
		  3   Direction of latitude:
		  N: North
		  S: South
		  4   Longitude
		  5   Direction of longitude:
		  E: East
		  W: West
		  6   GPS Quality indicator:
		  0: Fix not valid
		  1: GPS fix
		  2: Differential GPS fix, OmniSTAR VBS
		  4: Real-Time Kinematic, fixed integers
		  5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
		  7   Number of SVs in use, range from 00 through to 24+
		  8   HDOP
		  9   Orthometric height (MSL reference)
		  10  M: unit of measure for orthometric height is meters
		  11  Geoid separation
		  12  M: geoid separation measured in meters
		  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
		  14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
		  15
		  The checksum data, always begins with *
		*/  
		double utc_time = 0.0, lat = 0.0, lon = 0.0;
		float alt = 0.f, geoid_h = 0.f;
		float hdop = 99.9f, dgps_age = NAN;
		int  num_of_sv = 0, fix_quality = 0;
		char ns = '?', ew = '?';

		if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { hdop = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt = strtof(bufptr, &endp); bufptr = endp; }

		while (*(++bufptr) != ',') {} //skip M

		if (bufptr && *(++bufptr) != ',') { geoid_h = strtof(bufptr, &endp); bufptr = endp; }

		while (*(++bufptr) != ',') {} //skip M

		if (bufptr && *(++bufptr) != ',') { dgps_age = strtof(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}

		/* convert from degrees, minutes and seconds to degrees */
		_gps_position.longitude_deg = int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0;
		_gps_position.latitude_deg = int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0;
		_gps_position.hdop = hdop;
		_gps_position.altitude_msl_m = (double)alt;
		_gps_position.altitude_ellipsoid_m = (double)(alt + geoid_h);
		_sat_num_gga = static_cast<int>(num_of_sv);

		if (fix_quality <= 0) {
			_gps_position.fix_type = 0;

		} else {
			/*
			 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
			 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
			 */
			if (fix_quality == 5) { fix_quality = 3; }

			/*
			 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
			 */
			_gps_position.fix_type = 3 + fix_quality - 1;
		}

		_gps_position.c_variance_rad = 0.1f;
        _gps_position.satellites_used = _sat_num_gga;
        
        // EDEBUG("nsv: %d, fixtype: %d, lat: %.8f, lon:%.8f, hdop: %f, alt:%f\n", _sat_num_gga, _gps_position.fix_type, 
        //     _gps_position.latitude_deg, _gps_position.longitude_deg,hdop, _gps_position.altitude_msl_m);
    } else if ((memcmp(rxbuf + 3, "GSA,", 4) == 0) && (uiCalcComma >= 17)) {
		/*
		GPS DOP and active satellites
		An example of the GSA message string is:
		$GPGSA,<1>,<2>,<3>,<3>,,,,,<3>,<3>,<3>,<4>,<5>,<6>*<7><CR><LF>
		$GNGSA,A,3,82,67,74,68,73,80,83,,,,,,0.99,0.53,0.84,2*09
		$GNGSA,A,3,12,19,06,17,02,09,28,05,,,,,2.38,1.10,2.11,1*05
		$GNGSA,A,3,27,04,16,08,09,26,31,11,,,,,1.96,1.05,1.65,1*08

		GSA message fields
		Field	Meaning
		0	Message ID $GPGSA
		1	Mode 1, M = manual, A = automatic
		2	Mode 2, Fix type, 1 = not available, 2 = 2D, 3 = 3D
		3	PRN number, 01 through 32 for GPS, 33 through 64 for SBAS, 64+ for GLONASS
		4 	PDOP: 0.5 through 99.9
		5	HDOP: 0.5 through 99.9
		6	VDOP: 0.5 through 99.9
		7	The checksum data, always begins with *
		*/
		char M_pos = ' ';
		int fix_mode = 0;
		int sat_id[12] {0};
		float pdop = 99.9f, hdop = 99.9f, vdop = 99.9f;

		if (bufptr && *(++bufptr) != ',') { M_pos = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { fix_mode = strtol(bufptr, &endp, 10); bufptr = endp; }

		for (int y = 0; y < 12; y++) {
			if (bufptr && *(++bufptr) != ',') {sat_id[y] = strtol(bufptr, &endp, 10); bufptr = endp; }
		}

		if (bufptr && *(++bufptr) != ',') { pdop = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { hdop = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vdop = strtof(bufptr, &endp); bufptr = endp; }

		if (fix_mode <= 1) {
			_gps_position.fix_type = 0;

		} else {
			_gps_position.hdop = static_cast<float>(hdop);
			_gps_position.vdop = static_cast<float>(vdop);
		}
        
        // EDEBUG("hdop: %f, vdop: %f\n", _gps_position.hdop, _gps_position.vdop);
        
    } else if ((memcmp(rxbuf + 3, "GST,", 4) == 0) && (uiCalcComma == 8)) {
		/*
		Position error statistics
		An example of the GST message string is:

		$GPGST,172814.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031*6A
		$GNGST,091200.54,45,,,,1.2,0.77,2.2*70
		$GNGST,092720.50,43,,,,2.6,2.6,5.9*49

		The Talker ID ($--) will vary depending on the satellite system used for the position solution:

		$GP - GPS only
		$GL - GLONASS only
		$GN - Combined
		GST message fields
		Field   Meaning
		0   Message ID $GPGST
		1   UTC of position fix
		2   RMS value of the pseudorange residuals; includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing
		3   Error ellipse semi-major axis 1 sigma error, in meters
		4   Error ellipse semi-minor axis 1 sigma error, in meters
		5   Error ellipse orientation, degrees from true north
		6   Latitude 1 sigma error, in meters
		7   Longitude 1 sigma error, in meters
		8   Height 1 sigma error, in meters
		9   The checksum data, always begins with *
		*/ 
		double utc_time = 0.0;
		float lat_err = 0.f, lon_err = 0.f, alt_err = 0.f;
		float min_err = 0.f, maj_err = 0.f, deg_from_north = 0.f, rms_err = 0.f;

		if (bufptr && *(++bufptr) != ',') { utc_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { rms_err = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { maj_err = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { min_err = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { deg_from_north = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lat_err = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon_err = strtof(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt_err = strtof(bufptr, &endp); bufptr = endp; }

		_gps_position.eph = sqrtf(static_cast<float>(lat_err) * static_cast<float>(lat_err)
					   + static_cast<float>(lon_err) * static_cast<float>(lon_err));
		_gps_position.epv = static_cast<float>(alt_err);
        
        // EDEBUG("eph: %f epv: %f\n", _gps_position.eph, _gps_position.epv);
    }
    
    
    return ret;
}

