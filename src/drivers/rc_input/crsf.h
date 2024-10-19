
#ifndef __CRSF_H_
#define __CRSF_H_

#include "protocol/crsf_parser.hpp"
#include "ebus_uart.hpp"

class crsf
{
public:
    crsf();
    ~crsf();

    void configbus(ESAF::ebus_uart *com);
		
    uint8_t parser_process(float dt);

    void devbuf_retrieve(uint16_t len);

    bool is_connect();

public:
	bool connectonline;
    uint8_t onlinecnt;

    uint8_t channel[16];
    uint8_t raw[50];
    uint8_t retrieve[50];
	ESAF::crsf_parser::crsf_link_t _link;
	ESAF::crsf_parser::crsf_channels_t _channel;
    ESAF::ebus_uart *_com;
};

#endif
