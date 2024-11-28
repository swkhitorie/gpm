#include "ttl_can.h"
using namespace ESAF;

ttl_can::ttl_can(size_t size, Mutex *rm, Mutex *wm) : 
	edevbuffer<CANFrame>(size, rm, wm),
	_com(nullptr) {}
ttl_can::ttl_can(size_t read_size, size_t write_size, Mutex *rm, Mutex *wm) : 
	edevbuffer<CANFrame>(read_size, write_size, rm, wm),
	_com(nullptr) {}
ttl_can::~ttl_can() {}

void ttl_can::configttlbus(ESAF::ebus_uart *com)
{
	_com = com;
}

int ttl_can::init() { return EOK; }
int ttl_can::ioctl(unsigned int operation, unsigned int &arg) { return EOK; }

int ttl_can::update()
{
	transmit();
	receive();
	return EOK;
}

int ttl_can::read(void *pdata, unsigned int count)
{
	bool res = gets((CANFrame *)pdata, count, DEVBUFIN);
	if (res)
		return EOK;
	else
		return ENONE;
}

int ttl_can::read_through(void *pdata, unsigned int count, int rwway)
{
	return EOK;
}

int ttl_can::write(const void *pdata, unsigned int count)
{
	bool res = puts((const CANFrame *)pdata, count, DEVBUFOUT);
	if (res)
		return EOK;
	else
		return ENONE;
}

int ttl_can::write_through(const void *pdata, unsigned int count, int rwway)
{
	CANFrame *p = (CANFrame *)pdata;
	for (int i = 0; i < count; i++)
		sendframe(&p[i]);
	return EOK;
}

int ttl_can::query(unsigned int offset, void *pdata, unsigned int count)
{
	bool rs = rxquery(offset, (CANFrame *)pdata, count);
	if (rs) 
        return EOK;
	else 
        return ENONE;
}

void ttl_can::flush(int rwstream)
{
	int i = 0;
	switch (rwstream) {
	case STREAMOUT:
		{
			size_t _buffer_size;
			size(_buffer_size, DEVBUFOUT);
			for (i = 0; i < _buffer_size; i++) {
				if (gets(&_write_value, 1, DEVBUFOUT)) {
					sendframe(&_write_value);			
				}
			}
			break;
		}
	default: break;
	}
}

void ttl_can::eclear(int rwstream)
{
	switch (rwstream) {
	case STREAMIN:
		clear(DEVBUFIN);
	case STREAMOUT:
		clear(DEVBUFOUT);
	default : break;
	}
}

unsigned int ttl_can::esize(int rwstream)
{
	unsigned int res_size;
	switch (rwstream) {
	case STREAMIN:
		size(res_size, DEVBUFIN);
		return res_size;
	case STREAMOUT:
		size(res_size, DEVBUFOUT);
		return res_size;
	default : break;
	}
	return 0;
}

void ttl_can::transmit()
{
	CANFrame msg;
	if (gets(&msg, 1, ESAF::DEVBUFOUT))
		sendframe(&msg);
}

void ttl_can::sendframe(CANFrame *msg)
{
	struct TTL_CAN_Frame _tmsg;

	_tmsg._thead = TTLCAN_FRAMEHEAD;

	switch (msg->_idtype) {
	case ESAF::CANFrame::STANDARD_ID:
	    _tmsg._tidyype = TTLCAN_STRANDARD_FRAME;
		break;
	case ESAF::CANFrame::EXTENDED_ID:
	    _tmsg._tidyype = TTLCAN_EXTENDED_FRAME;
		break;
	}

	switch (msg->_frametype) {
	case ESAF::CANFrame::DATA_FRAME:
	    _tmsg._tdatatype = 0x00;
		break;
	case ESAF::CANFrame::REMOTE_FRAME:
	    _tmsg._tdatatype = 0x01;
		break;
	}

    _tmsg._tdatalength = msg->_datalength;
    _tmsg._tid = Serialize::bytes_to_u32((uint8_t *)(&msg->_identifier), true);

    for (uint8_t i = 0; i < msg->_datalength; i++)
    	_tmsg._tdata[i] = msg->_data[i];
    _tmsg._ttail = TTLCAN_FRAMETAIL;

    _com->write_through((uint8_t *)(&_tmsg), sizeof(_tmsg), ESAF::EDev::NONBLOCK);
}

uint8_t ttl_can::receive()
{
	int res = ESAF::EDev::ENONE;
    uint16_t preframe_size = 0;
	uint16_t index = 0;
	uint8_t tmp = 0;
    uint16_t retrieve_size = 0;

	struct TTL_CAN_Frame _rmsg;
	ESAF::CANFrame _rcmsg;

    if (_com->esize(ESAF::ebus_uart::STREAMIN) < sizeof(_rmsg)) {
        return 0x01;
    }

	do {
		res = _com->query(index, &tmp, 1);
		if (tmp == TTLCAN_FRAMEHEAD)
			break;
        else 
            index++;
	} while (res == ESAF::EDev::EOK);

	preframe_size = index;
	if (res == ESAF::EDev::ENONE) {
        devbuf_retrieve(preframe_size);
		return 0x02;
	}

	if (_com->esize(ESAF::ebus_uart::STREAMIN) - preframe_size < sizeof(_rmsg)) {
        devbuf_retrieve(_com->esize(ESAF::ebus_uart::STREAMIN));
		return 0x03;
	}

	_com->query(index + sizeof(_rmsg) - 1, &tmp, 1);
	if (tmp != TTLCAN_FRAMETAIL) {
        devbuf_retrieve(preframe_size + sizeof(_rmsg));
		return 0x04;
	}

	_com->query(index, (uint8_t *)&_rmsg, sizeof(_rmsg));
	_rcmsg._identifier = Serialize::bytes_to_u32((uint8_t *)(&_rmsg._tid), true);
	_rcmsg._datalength = _rmsg._tdatalength;

	switch (_rmsg._tidyype) {
	case TTLCAN_STRANDARD_FRAME:
	    _rcmsg._idtype = ESAF::CANFrame::STANDARD_ID;
		break;
	case TTLCAN_EXTENDED_FRAME:
	    _rcmsg._idtype = ESAF::CANFrame::EXTENDED_ID;
		break;
	}

	switch (_rmsg._tdatatype) {
	case 0x00:
	    _rcmsg._frametype = ESAF::CANFrame::DATA_FRAME;
		break;
	case 0x01:
	    _rcmsg._frametype = ESAF::CANFrame::REMOTE_FRAME;
		break;
	}

	for (int i = 0; i < 8; i++)
		_rcmsg._data[i] = _rmsg._tdata[i];

	puts(&_rcmsg, 1, ESAF::DEVBUFIN);

	devbuf_retrieve(preframe_size + sizeof(_rmsg));
	if (_com->esize(ESAF::ebus_uart::STREAMIN) >= sizeof(_rmsg))
		receive();
	return 0x00;
}

void ttl_can::devbuf_retrieve(uint16_t len)
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



