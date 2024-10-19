#include "linktrack_aoa.h"
#include "utility/log.hpp"
linktrack_aoa::linktrack_aoa(ESAF::ebus_uart *com) : _com(com)
{
}

void linktrack_aoa::devbuf_retrieve(uint16_t len)
{
    while(len > 0) {
        if (len > 50) {
            _com->read(&_retrieve[0], 50);
        }else if (len <= 50){
            _com->read(&_retrieve[0], len);
            return;
        }
        len -= 50;
    }
}

uint8_t linktrack_aoa::checksum(uint8_t *p, uint32_t len)
{
    uint8_t sum = LINKTRACK_FRAME_HEADER + LINKTRACK_FUNC_MARK;
    for (int i = 0; i < len - 1; ++i) {
        sum += p[i];
    }
    return sum;  
}

int8_t linktrack_aoa::update()
{
	int res = ESAF::EDev::ENONE;
    uint16_t preframe_size = 0;
    uint16_t start_index = 0;
    uint16_t check_index = 0;
	uint16_t index = 0;
	uint8_t tmp = 0;

    uint8_t func_mark = 0;
    
    /* too small buffer to found frame, clear all buffer */
    if (_com->esize(ESAF::ebus_uart::STREAMIN) < LINKTRACK_MIN_LEN)
        return 0x01;
    
	do {
		res = _com->query(index, &tmp, 1);
		if (tmp == LINKTRACK_FRAME_HEADER)
			break;
        else 
            index++;
	} while (res == ESAF::EDev::EOK);
	
	preframe_size = index;
    start_index = index;
	index++;
    
    /* no header found, clear all buffer */
	if (res == ESAF::EDev::ENONE) {
        devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
		return 0x02;
	}

    _com->query(index++, &func_mark, 1);
    
    /* no func_mark found, clear preframe size + header + func_mark */
	if (func_mark != LINKTRACK_FUNC_MARK) {
        devbuf_retrieve(preframe_size + 2);
		return 0x03;
	}
    
    _com->query(index++, reinterpret_cast<uint8_t *>(&_pack), sizeof(_pack));
    uint8_t crc = checksum(reinterpret_cast<uint8_t *>(&_pack), sizeof(_pack) - 1);
    
    /* crc error, clear preframe size + header + func_mark + dataframe */
    if (crc != _pack.check) {
        devbuf_retrieve(preframe_size + sizeof(_pack) + 2);
        return 0x04;
    }
    
    
    _distance = ((int32_t)(_pack.dis1 << 8 | _pack.dis2 << 16 | _pack.dis3 << 24) / 256) / 1000.0f;
    _lst_angle = _pack.angle / 100.0f;
    
    _voltage = _pack.voltage_org / 1000.0f;
    _localtime = _pack.local_time / -2.0f;
    _systemtime = _pack.system_time / -2.0f;
    _rxRssi = _pack.rx_rssi * -2.0f;
    _fpRssi = _pack.fp_rssi * -2.0f;

    _angle = _angle + 0.5f * (_lst_angle - _angle);
    
    _localtime == _lst_localtime ? _effective = false: _effective = true;
    _lst_localtime = _localtime;
    
    devbuf_retrieve(preframe_size + sizeof(_pack) + 2);
	if (_com->esize(ESAF::ebus_uart::STREAMIN) >= (sizeof(_pack) + 2))
		update();
    
	return 0;
}
