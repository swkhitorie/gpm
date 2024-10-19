
#ifndef __SBUS_H_
#define __SBUS_H_

#include "protocol/sbus_parser.hpp"
#include "ebus_uart.hpp"

enum Remote_Order {
	CHANNEL_1 = 0x00,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4,
	CHANNEL_5,
	CHANNEL_6,
	CHANNEL_7,
	CHANNEL_8,
	CHANNEL_9,
	CHANNEL_10,
	CHANNEL_11,
	CHANNEL_12,
	CHANNEL_13,
	CHANNEL_14,
	CHANNEL_15,
	CHANNEL_16,
	
	NUMBER_OF_ORDER,
};

class sbus
{
public:
    sbus();
    ~sbus();

    void configbus(ESAF::ebus_uart *com);
		
    uint8_t parser_process(float dt);

    void devbuf_retrieve(uint16_t len);

    bool is_connect();
    bool is_joyonline();
  
    float get_signalstrength();
    float get_lostsignalstrength();

public:
	bool joyonline;
	bool connectonline;
	float lost_signal_rate;
	float signal_rate;

    uint8_t joyonline_v;
    uint8_t onlinecnt;
    uint32_t cnt_lost;
    uint32_t cnt_update;
    uint32_t lst_cnt_lost;
    uint32_t lst_cnt_update;
    float update_hz;
    float lost_hz;

    uint16_t channel[16];
    uint8_t raw[25];
    uint8_t retrieve[50];
    ESAF::ebus_uart *_com;
};

#endif
