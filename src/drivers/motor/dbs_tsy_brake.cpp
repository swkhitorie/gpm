#include "dbs_tsy_brake.h"

DBS_Brake::DBS_Brake(ESAF::ebus_can *can, uint32_t id) :
    canDriver(can, id),
    isZeroPress(false),
    ZeroPressTimes(4),
    statusbits(0),
    verrorcode(0),
	ctrl_press(0.0f),
	fb_press(0.0f) {}
DBS_Brake::~DBS_Brake() {}

void DBS_Brake::enable() {}
void DBS_Brake::disable() {}

void DBS_Brake::poll_operation() { setmotion(); }

void DBS_Brake::poll_recvparser(const ESAF::CANFrame *msg)
{
    ESAF::CANFrame rxMsg = (*msg);
	if (rxMsg._identifier == DBSRECV_ID) {	
		_offlineCnt = 0;
		_online = true;

		statusbits = (rxMsg._data[0] & 0x03);
		verrorcode = rxMsg._data[4];
		
		fb_press = 0.0f + (((uint16_t)rxMsg._data[3]<<8) | (uint16_t)rxMsg._data[2])  * 0.01;
        cerrorcode = verrorcode;
        cerrorcode &= 0xFFFFFFDF;
		cstatus = statusbits;
	}else if(++_offlineCnt > 200) {
		_offlineCnt = 300;
		_online = false;
	}
}

void DBS_Brake::init() {}
void DBS_Brake::getstatus() {}
void DBS_Brake::setmode() {}
void DBS_Brake::fault_recover() {}
    
void DBS_Brake::setmotion()
{
	if (ctrl_press <= 0.0f)
		ctrl_press = 0.0f;
	if (ctrl_press > 8.0f)
		ctrl_press = 8.0f;
	
	uint16_t ctrlPress_value = ctrl_press * 100;
	ESAF::CANFrame msg;
	msg._identifier = _id;
	msg._datalength = 8;
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._data[0] = 0x00;
	msg._data[1] = 0x00;
	msg._data[2] = 0x02;
	msg._data[3] = 0x00;
	msg._data[4] = 0x00;
	msg._data[5] = uint8_t(ctrlPress_value & 0x00FF);
	msg._data[6] = uint8_t((ctrlPress_value & 0xFF00) >> 8);
	msg._data[7] = 0x00;
	
	if (ctrlPress_value == 0) isZeroPress = true;
	else 	isZeroPress = false;
	
	if (isZeroPress && ZeroPressTimes > 0) {
		_can->write(&msg, 1);	
		ZeroPressTimes--;
	}else if(!isZeroPress) {
		_can->write(&msg, 1);	
		ZeroPressTimes = 4;
	}
}

void DBS_Brake::setctrlpress(float press){ ctrl_press = press; }
float DBS_Brake::getpress() { return fb_press; }
