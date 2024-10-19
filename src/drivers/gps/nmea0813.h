
#ifndef __NMEA0813_H_
#define __NMEA0813_H_

#include "protocol/nmea0813_parser.h"
#include "ebus_uart.hpp"

#define uOSB_PUBLISH_NMEA_GPS
#ifdef uOSB_PUBLISH_NMEA_GPS
    #include "uORB/uORB_topic_define.hpp"
#endif

class nmea0813
{
public:
    nmea0813() = delete;
    nmea0813(uint32_t _id, ESAF::ebus_uart *com);
    ~nmea0813();
		
    uint8_t parser_process(float dt);

    void devbuf_retrieve(uint16_t len);

public:
    uint32_t device_id;
    uint64_t timestamp_start_block;

    // ESAF::nmea_msg _msg;

    float pass_rate;
    uint64_t checkfault_cnt;
    uint64_t checkpass_cnt;

	uint8_t retrieve[50];

	uint8_t parse_window[512];
	
    ESAF::ebus_uart *_com;

    #ifdef uOSB_PUBLISH_NMEA_GPS
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_GPS>::type _pub_sensor_gps;
    #endif
};

#endif
