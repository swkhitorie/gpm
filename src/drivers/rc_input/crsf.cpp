#include "crsf.h"

crsf::crsf() : 
    connectonline(false),
	onlinecnt(0)
{
    for (int i = 0; i < 16; i++)
        channel[i] = 0;
    for (int i = 0; i < 50; i++) {
        raw[i] = 0;
    }
    _com = nullptr;
}

crsf::~crsf(){}

void crsf::configbus(ESAF::ebus_uart *com)
{
	_com = com;
}

uint8_t crsf::parser_process(float dt)
{
	int res = ESAF::EDev::ENONE;
    uint16_t preframe_size = 0;
	uint16_t index = 0;
	uint8_t tmp = 0;
    uint16_t retrieve_size = 0;
    
	uint8_t crc8_v = 0;
	uint8_t len = 0;
	uint8_t type = 0;
	
    if (++onlinecnt > 10) {
        connectonline = false;
    }
    
    if (_com->esize(ESAF::ebus_uart::STREAMIN) < 5)
        return 0x01;
    
	do {
		res = _com->query(index, &tmp, 1);
		if (tmp == 0xC8)
			break;
        else 
            index++;
	} while (res == ESAF::EDev::EOK);

	preframe_size = index;
	if (res == ESAF::EDev::ENONE) {
        devbuf_retrieve(preframe_size);
		return 0x02;
	}

	if (_com->esize(ESAF::ebus_uart::STREAMIN) - preframe_size < 5) {
        devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
		return 0x03;
	}

	_com->query(index + 1, &len, 1);
	if (len != ESAF::crsf_parser::CRSFLEN_LINK_STATISTICS && len != ESAF::crsf_parser::CRSFLEN_RC_CHANNELS_PACKED) {
        devbuf_retrieve(preframe_size + 25);
		return 0x04;
	}
	
	_com->query(index + 2, &type, 1);
	if (type != ESAF::crsf_parser::CRSF_LINK_STATISTICS && type != ESAF::crsf_parser::CRSF_RC_CHANNELS_PACKED) {
        devbuf_retrieve(preframe_size + 25);
		return 0x05;
	}
	
	_com->query(index + 2 + len - 1, &crc8_v, 1);
	_com->query(index + 2, &raw[0], len - 1);
	uint8_t crc_v = ESAF::crsf_parser::crc8(&raw[0], len - 1);
	if (crc_v != crc8_v) {
        devbuf_retrieve(preframe_size + len + 2);
		return 0x06;		
	}

	switch (type) {
	case ESAF::crsf_parser::CRSF_LINK_STATISTICS:
		_com->query(index + 3, (uint8_t *)&_link, len - 2);
		break;
	case ESAF::crsf_parser::CRSF_RC_CHANNELS_PACKED:
		_com->query(index + 3, (uint8_t *)&_channel, len - 2);
		break;	
	}
	
	channel[0] = ESAF::crsf_parser::value_map(_channel.ch0, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[1] = ESAF::crsf_parser::value_map(_channel.ch1, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[2] = ESAF::crsf_parser::value_map(_channel.ch2, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[3] = ESAF::crsf_parser::value_map(_channel.ch3, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[4] = ESAF::crsf_parser::value_map(_channel.ch4, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[5] = ESAF::crsf_parser::value_map(_channel.ch5, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[6] = ESAF::crsf_parser::value_map(_channel.ch6, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[7] = ESAF::crsf_parser::value_map(_channel.ch7, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[8] = ESAF::crsf_parser::value_map(_channel.ch8, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[9] = ESAF::crsf_parser::value_map(_channel.ch9, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[10] = ESAF::crsf_parser::value_map(_channel.ch10, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[11] = ESAF::crsf_parser::value_map(_channel.ch11, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[12] = ESAF::crsf_parser::value_map(_channel.ch12, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[13] = ESAF::crsf_parser::value_map(_channel.ch13, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[14] = ESAF::crsf_parser::value_map(_channel.ch14, CRSF_VALUE_MIN, CRSF_VALUE_MAX);
	channel[15] = ESAF::crsf_parser::value_map(_channel.ch15, CRSF_VALUE_MIN, CRSF_VALUE_MAX);

	connectonline = true;
    onlinecnt = 0;
    devbuf_retrieve(preframe_size + len + 2);
    
	if (_com->esize(ESAF::ebus_uart::STREAMIN) >= 5)
		parser_process(dt);
    return 0x00;
}

void crsf::devbuf_retrieve(uint16_t len)
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

bool crsf::is_connect()
{
    return connectonline;
}
