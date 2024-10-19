#include "dbs_3001_brake.h"
#include "serialize.hpp"


DBS_3001_Brake::DBS_3001_Brake(ESAF::ebus_can *can, uint32_t id) :
    canDriver(can, id),
	statusbits(0),
	verrorcode(0),
	ctrl_press(0.0f),
	fb_press(0.0f) {}
DBS_3001_Brake::~DBS_3001_Brake() {}

void DBS_3001_Brake::enable(){}
void DBS_3001_Brake::disable(){}
void DBS_3001_Brake::poll_operation() { setmotion(); }
void DBS_3001_Brake::poll_recvparser(const ESAF::CANFrame *msg)
{
    ESAF::CANFrame rxMsg = (*msg);
	if (rxMsg._identifier == DBSRECV_ID) {	
		_offlineCnt = 0;
		_online = true;

		statusbits = (rxMsg._data[0] & 0x03);
		park_warning = (rxMsg._data[0] & 0xC0) >> 6;
		running_mode = rxMsg._data[1];
		press_busy = rxMsg._data[2];
		estop_flag = (rxMsg._data[6] & 0x40) >> 6;
		pedal_flag = (rxMsg._data[6] & 0x80) >> 7;
		
		pedal_throttle = rxMsg._data[4];
		fb_press = 0.0f + rxMsg._data[3] * 0.1f;
        ccurrent = 0.0f + (rxMsg._data[5] * 0.5 - 20);
        
		cstatus = (uint32_t)statusbits | (uint32_t)park_warning << 2 | 
					(uint32_t)estop_flag << 4 | (uint32_t)pedal_flag << 5 | 
					(uint32_t)press_busy << 6 | (uint32_t)running_mode << 8;
	}else if (rxMsg._identifier == DBSRECV_ID2) {
		_offlineCnt = 0;
		_online = true;
		
		verrorcode = ESAF::Serialize::bytes_to_u16(&rxMsg._data[0], false);
		warning_code = ESAF::Serialize::bytes_to_u16(&rxMsg._data[3], false);
		real_press = 0.0f + rxMsg._data[5] * 0.1f;
		
		cerrorcode = (uint32_t)verrorcode | (uint32_t)warning_code << 16;
	}else if (++_offlineCnt > 200) {
		_offlineCnt = 300;
		_online = false;
	}
}

void DBS_3001_Brake::init() {}
void DBS_3001_Brake::getstatus() {}
void DBS_3001_Brake::setmode() {}
void DBS_3001_Brake::fault_recover() {}

void DBS_3001_Brake::setmotion()
{
	if (ctrl_press <= 0.0f)
		ctrl_press = 0.0f;
	if (ctrl_press > 10.0f)
		ctrl_press = 10.0f;
	
	uint8_t ctrlPress_value = ctrl_press * 10;

	ESAF::CANFrame msg;
	msg._identifier = _id;
	msg._datalength = 8;
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._data[0] = 0x01;
	msg._data[1] = MODE_WIRE;
	msg._data[2] = ctrlPress_value;
	msg._data[3] = 0;
	msg._data[4] = 0;
	msg._data[5] = 0;
	msg._data[6] = 0;
	msg._data[7] = 0;
	_can->write(&msg, 1);
}

void DBS_3001_Brake::setctrlpress(float press) { ctrl_press = press; }
float DBS_3001_Brake::getpress() { return fb_press; }
