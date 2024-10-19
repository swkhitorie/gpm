#include "sbus.h"

sbus::sbus() : 
    joyonline(false),
    connectonline(false),
    lost_signal_rate(0.0f),
    signal_rate(0.0f),
    joyonline_v(0),
    onlinecnt(0), 
	cnt_lost(0), 
	cnt_update(0), 
	lst_cnt_lost(0), 
	lst_cnt_update(0),
	update_hz(0.0f),
	lost_hz(0.0f)
{
    for (int i = 0; i < 16; i++)
        channel[i] = 0;
    for (int i = 0; i < 25; i++) {
        raw[i] = 0;
    }
    _com = nullptr;
}

sbus::~sbus(){}

void sbus::configbus(ESAF::ebus_uart *com)
{
	_com = com;
}

uint8_t sbus::parser_process(float dt)
{
	int res = ESAF::EDev::ENONE;
    uint16_t preframe_size = 0;
	uint16_t index = 0;
	uint8_t tmp = 0;
    uint16_t retrieve_size = 0;
    
    if (++onlinecnt > 10) {
        connectonline = false;
        joyonline = false;
    }
    
    if (_com->esize(ESAF::ebus_uart::STREAMIN) < 25)
        return 0x01;
    
	do {
		res = _com->query(index, &tmp, 1);
		if (tmp == 0x0F)
			break;
        else 
            index++;
	} while (res == ESAF::EDev::EOK);

	preframe_size = index;
	if (res == ESAF::EDev::ENONE) {
        devbuf_retrieve(preframe_size);
		return 0x02;
	}

	if (_com->esize(ESAF::ebus_uart::STREAMIN) - preframe_size < 25) {
        devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
		return 0x03;
	}

	_com->query(index + 24, &tmp, 1);
	if (tmp != 0x00) {
        devbuf_retrieve(preframe_size + 25);
		return 0x04;
	}

	_com->query(index, &raw[0], 25);
	ESAF::sbus_parser::sbus_channel_parser(&raw[0], &channel[0], joyonline_v, cnt_lost, cnt_update);

	update_hz = update_hz + 0.015f * ((cnt_update - lst_cnt_update) / dt - update_hz);
	lost_hz = lost_hz + 0.015f * ((cnt_lost - lst_cnt_lost) / dt - lost_hz);
	lst_cnt_update = cnt_update;
	lst_cnt_lost = cnt_lost;
    
    signal_rate = update_hz;
    lost_signal_rate = lost_hz;

	connectonline = true;
    onlinecnt = 0;
    if (joyonline_v == 0)
        joyonline = false;
    else if (joyonline_v == 1)
        joyonline = true;
    
    devbuf_retrieve(preframe_size + 25);
    
	if (_com->esize(ESAF::ebus_uart::STREAMIN) >= 25)
		parser_process(dt);
    return 0x00;
}

void sbus::devbuf_retrieve(uint16_t len)
{
    while(1) {
        if (len > 50) {
            _com->read(&retrieve[0], 50);
        }else if (len <= 50){
            _com->read(&retrieve[0], len);
            return;
        }
        len -= 50;
    }
}

bool sbus::is_connect()
{
    return connectonline;
}

bool sbus::is_joyonline()
{
    return joyonline;
}

float sbus::get_signalstrength()
{
    return signal_rate;
}

float sbus::get_lostsignalstrength()
{
    return lost_signal_rate;
}
