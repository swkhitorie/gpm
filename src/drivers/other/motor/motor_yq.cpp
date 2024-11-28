#include "motor_yq.h"

Motor_YQ::Motor_YQ(ESAF::ebus_can *can, uint32_t id) : 
	canDriver(can, id),
	driving(true)
{
    _idtag = ((uint8_t)(id >> 8) & 0x0F);
}

Motor_YQ::~Motor_YQ() {}

void Motor_YQ::enable()
{
    _state = INITIALIZATION;
}

void Motor_YQ::disable()
{
	sendcommand(RUNMODE_FORWARD,0,CTLMODE_THROTTLE,0,0);
	_state = STOPPED;
}

void Motor_YQ::poll_operation()
{
	switch (_state) {
	case STOPPED:
		break;
	case INITIALIZATION:
		init();
		_state = OPERATIONAL;
		break;
	case PRE_OPERATIONAL:
		break;
	case OPERATIONAL:
		{
			if(driving) setmotion();
			break;
		}
	}
}

void Motor_YQ::poll_recvparser(const ESAF::CANFrame *msg)
{
    ESAF::CANFrame rxMsg = (*msg);
	if (((uint8_t)(rxMsg._identifier >> 8) & 0x0F) == _idtag) {
		_offlineCnt = 0;
		_online = true;
		switch ((rxMsg._identifier & 0x000000FF)) {
		case 0x9A:
			{
				rcv_mode = rxMsg._data[0];
				fb_throttle = ((uint16_t)rxMsg._data[2] << 8) | rxMsg._data[1];
				statusbit = rxMsg._data[3];
				fb_velocity = ((uint16_t)rxMsg._data[5] << 8) | rxMsg._data[4];
				fb_velocity /= 4;
				fb_velocity = fb_velocity * (3.14f / 30.0f) * 0.3f;
				rcv_motortemp = rxMsg._data[6];
				rcv_ctrlertemp = rxMsg._data[7];
				break;
			}
		case 0x8D:
			{
				rcv_volts = ((uint16_t)rxMsg._data[1]<<8) | rxMsg._data[0];
				rcv_volts *= 0.1f;
				rcv_current = ((uint16_t)rxMsg._data[3]<<8) | rxMsg._data[2];
				rcv_current *= 0.1f;
				rcv_cntrotate = ((uint16_t)rxMsg._data[5]<<8) | rxMsg._data[4];
				errorcode = ((uint16_t)rxMsg._data[7]<<8) | rxMsg._data[6];
				break;
			}
		case 0x7B:
			{
				fb_torque = 0.0f + (((uint16_t)rxMsg._data[5]<<8) | rxMsg._data[4])  * 0.1f;
				break;
			}
			
		}
        cvoltage = 0.0f + (float)rcv_volts;
        ccurrent = 0.0f + (float)rcv_current;
        cerrorcode = errorcode;
		cstatus = (uint16_t)(rcv_mode << 8) | statusbit;
	}else if(++_offlineCnt > 200) {
		_offlineCnt = 300;
		_online = false;
		_state = STOPPED;
	}
}

void Motor_YQ::init()
{
    sendcommand(RUNMODE_NULL, 0, CTLMODE_THROTTLE, 0, 0);
}

void Motor_YQ::setmotion()
{
	if (ctrl_throttle > 255) 
		ctrl_throttle = 255;
    uint16_t throttle;
    
	if (ctrl_throttle > 0) {
        throttle = (uint16_t)ctrl_throttle;
		sendcommand(RUNMODE_FORWARD, throttle, CTLMODE_THROTTLE, 0, 0);
	}else if(ctrl_throttle < 0) {
        throttle = (uint16_t)((-1) * ctrl_throttle);
		sendcommand(RUNMODE_BACK, throttle,CTLMODE_THROTTLE, 0, 0);	
	}else {
		sendcommand(RUNMODE_FORWARD, 0, CTLMODE_THROTTLE, 0, 0);
    }
}

void Motor_YQ::getstatus() {}
void Motor_YQ::setmode() {}
void Motor_YQ::fault_recover() {}

void Motor_YQ::sendcommand(uint8_t runmode, uint16_t throttle,
            uint8_t ctrlmode, uint16_t torque, uint16_t rpm)
{
	ESAF::CANFrame msg;
	msg._data[0] = runmode;
	msg._data[1] = uint8_t(throttle & 0x00FF);
	msg._data[2] = uint8_t((throttle & 0xFF00) >> 8);
	msg._data[3] = ctrlmode;
	msg._data[4] = uint8_t(torque & 0x00FF);
	msg._data[5] = uint8_t((torque & 0xFF00) >> 8);
	msg._data[6] = uint8_t(rpm & 0x00FF);
	msg._data[7] = uint8_t((rpm & 0xFF00) >> 8);
	msg._identifier = _id;
	msg._idtype = ESAF::CANFrame::EXTENDED_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._datalength = 8;
	_can->write(&msg, 1);
}

void Motor_YQ::setdriving(bool value)
{
    driving = value;
}

uint8_t Motor_YQ::get_motortemp()
{
	return rcv_motortemp;
}

uint8_t Motor_YQ::get_ctrlertemp()
{
	return rcv_ctrlertemp;
}
		
	