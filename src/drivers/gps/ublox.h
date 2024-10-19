#ifndef UBLOX_H_
#define UBLOX_H_

#include "protocol/ublox_parser.h"
#include "ebus_uart.hpp"

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#define uOSB_PUBLISH_UBLOX_GPS
#ifdef uOSB_PUBLISH_UBLOX_GPS
    #include "uORB/uORB_topic_define.hpp"
#endif

class ublox
{
public:
    ublox() = delete;
    ublox(uint32_t _id, ESAF::ebus_uart *com);
    ~ublox() = default;
		
    uint8_t update();

    void devbuf_retrieve(uint16_t len);

public:
    uint32_t device_id;
    uint64_t timestamp_start_block;

    ubx_payload_rx_nav_pvt_t _msg_pvt;
    ubx_payload_rx_nav_dop_t _msg_dop;

	uint8_t _retrieve[50];

	uint8_t _check_data[256];
	
    ESAF::ebus_uart *_com;

    #ifdef uOSB_PUBLISH_UBLOX_GPS
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_GPS>::type _pub_sensor_gps;
    #endif
};







#endif
