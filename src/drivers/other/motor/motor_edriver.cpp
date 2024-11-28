#include "motor_edriver.h"

Motor_EDriver::Motor_EDriver(ESAF::ebus_can *can, uint32_t id, uint8_t drvmode, uint32_t ratio, uint32_t ppr) :
    canDriver(can, id),
	mreductratio(ratio),
	mppr(ppr),
	config_poweron(true),
	drv_mode(drvmode),
	posprocess(false), 
	posctrl(0), 
	posgo(1),
	cnter_findzero(PMODE_FIND_ZERO),
	cnter_poswait(PMODE_SEND_TIME), 
	cnter_posdt(PMODE_TIME) 
{ 
	_idtag = (uint8_t)((id & 0x0F) >> 4); 
}
Motor_EDriver::~Motor_EDriver() {}

bool Motor_EDriver::wait_nonblock(int16_t *timer, int16_t max_value)
{
	if (*timer <= 0) {
		*timer = max_value;
		return true;
	} else {
		*timer -= 1;
		return false;
	}	
}

void Motor_EDriver::sendcommand_1(uint16_t speedreq,uint8_t opermodreq,
	uint8_t fault_rst_en, uint8_t unlock, uint8_t pos_go, int32_t posreq)
{
	ESAF::CANFrame msg;
	
	speedreq -= OFFSET_MODE_SPEED;
	msg._data[0] = uint8_t (opermodreq << 4 | (speedreq & 0xF000) >> 12);
	msg._data[1] = uint8_t ((speedreq & 0x0FF0) >> 4);
	msg._data[2] = uint8_t ((speedreq & 0x000F) << 4 | pos_go << 3 | unlock << 2 | fault_rst_en << 1);
	msg._data[3] = 0;
	ESAF::Serialize::s32_to_bytes(posreq, &msg._data[4], true);

	msg._identifier = (_id << 4) | 0x00;
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._datalength = 8;
	_can->write(&msg, 1);
}

void Motor_EDriver::sendcommand_2(float pmode_max_speed,float pmode_max_accel,
	uint16_t zero_spd, uint8_t tq_lim, uint16_t tq_req, uint8_t roll_counter, uint8_t crc_check)
{
	ESAF::CANFrame msg;
	
	uint8_t max_speed, max_accel;
	zero_spd -= OFFSET_MODE_ZERO_SPEED;
	max_speed = pmode_max_speed / FACTOR_PMODE_MAX_SPEED;
	max_accel = pmode_max_accel / FACTOR_PMODE_MAX_ACCEL;
	tq_req += 800;
	tq_req *= 10;
	msg._data[0] = uint8_t (max_accel << 4 | max_speed);
	msg._data[1] = uint8_t (tq_lim << 4 | (zero_spd & 0x0F00) >> 8);
	msg._data[2] = uint8_t (zero_spd & 0x00FF);
	msg._data[3] = uint8_t ((tq_req & 0x3FC0) >> 6);
	msg._data[4] = uint8_t ((tq_req & 0x003F) << 2);
	msg._data[5] = 0;
	msg._data[6] = uint8_t (roll_counter << 4);
	msg._data[7] = uint8_t (crc_check);
	
	msg._identifier = (_id << 4) | 0x01;
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._datalength = 8;
	_can->write(&msg, 1);
}

void Motor_EDriver::init()
{
	if (config_poweron) {
		switch (drv_mode) {
		case DRIVER_SPEED:
			// pmode_max_speed, pmode_max_accel, zero_spd, tq_lim, tq_req
			sendcommand_2(0, 0, -500, 5, 0, 0, 0);
			sendcommand_2(0, 0, -500, 5, 0, 0, 0);
			sendcommand_2(0, 0, -500, 5, 0, 0, 0);
			break;
		case DRIVER_POSITION:
			// pmode_max_speed, pmode_max_accel, zero_spd, tq_lim, tq_req
			sendcommand_2(1500, 6000, -500, 5, 0, 0, 0);
			sendcommand_2(1500, 6000, -500, 5, 0, 0, 0);
			sendcommand_2(1500, 6000, -500, 5, 0, 0, 0);
			break;
		}
		config_poweron = false;
	}
	
	if (rcv_fault == 0) {
		//speedreq, mode, fault_rst, unlock, posgo, posreq
		sendcommand_1(0, MODE_FINDZERO, 0, 1, 0, 0);
		_state = PRE_OPERATIONAL;
	}

	//reset motor fault
	//speedreq, mode, fault_rst, unlock, posgo, posreq
	sendcommand_1(0, MODE_SHUTDOWN, 1, 1, 0, 0);
}

void Motor_EDriver::pre_operational()
{
	if (rcv_findzero == 1) {
		if (rcv_opermod == MODE_STANDBY) {
			if (wait_nonblock(&cnter_findzero, 500))
				_state = OPERATIONAL;
            sendcommand_1(0, MODE_SHUTDOWN, 0, 1, 0, 0);
		}else {
			switch (drv_mode) {
			case DRIVER_SPEED:
				if (rcv_startpos == 1) 
					sendcommand_1(0, MODE_SHUTDOWN, 0, 1, 0, 0);
				break;
			case DRIVER_POSITION:
				if (rcv_startpos == 1 || rcv_endpos == 1) 
					sendcommand_1(0, MODE_SHUTDOWN, 0, 1, 0, 0);
				break;
			}			
		}
	}else {
		//speedreq, mode, fault_rst, unlock, posgo, posreq
		sendcommand_1(0, MODE_FINDZERO, 0, 1, 0, 0);
	}
}

void Motor_EDriver::setup_posgo()
{
	posprocess = true;
	posctrl = 1;
	posgo = 0;
	cnter_poswait = PMODE_SEND_TIME;
}

void Motor_EDriver::setdown_posgo()
{
	posprocess = false;
	posctrl = 0;
	posgo = 0;
	cnter_poswait = PMODE_SEND_TIME;
}

void Motor_EDriver::setmotion()
{
	bool res;
	int16_t _ctrl_vel = (int16_t)ctrl_velocity;
	int32_t _ctrl_pos = (int32_t)ctrl_position;
	
	switch (drv_mode) {
	case DRIVER_SPEED:
		{
			if (_ctrl_vel >= 3000)
				_ctrl_vel = 3000;
			else if (_ctrl_vel <= -3000)
				_ctrl_vel = -3000;
            int16_t final_vel = (_ctrl_vel * 60 / 360) * mreductratio;
			sendcommand_1((uint16_t)final_vel, MODE_SPEED, 0, 1, 0, 0);
			break;
		}
	case DRIVER_POSITION:
		{
			if (wait_nonblock(&cnter_posdt, PMODE_TIME)) {
				if (posprocess) {
					res = wait_nonblock(&cnter_poswait, PMODE_SEND_TIME);
					if (res) {
						if (posctrl == 1 && posgo == 0) {
							posgo = 1;
							posctrl = 2;
							goto END_POS;
						}
						if (posctrl == 2 && posgo == 1) {
							posgo = 0;
							posprocess = false;
							goto END_POS;
						}
					}
				}
				END_POS:
				sendcommand_1(0, MODE_POSITION, 0, 1, posgo, _ctrl_pos);	
			}
			break;
		}
	}
}

void Motor_EDriver::getstatus(){}
void Motor_EDriver::setmode() {}
void Motor_EDriver::fault_recover() {}
void Motor_EDriver::poll_operation()
{
	switch (_state) {
	case STOPPED:
		break;
	case INITIALIZATION:
		init();
		break;
	case PRE_OPERATIONAL:
		pre_operational();
		break;
	case OPERATIONAL:
		setmotion();
		break;
	}
	getstatus();
}

void Motor_EDriver::poll_recvparser(const ESAF::CANFrame *msg)
{	
	uint16_t code2 = 0;
    ESAF::CANFrame rxMsg = (*msg);
	uint8_t req_tag = rxMsg._identifier & 0x0F;
	if ((uint8_t)((rxMsg._identifier & 0xF0) >> 4) == _id) {
		_offlineCnt = 0;
		_online = true;
		switch (req_tag) {
		case 0x05:
			rcv_startpos = uint8_t(rxMsg._data[1] & 0x01);
			rcv_endpos = uint8_t((rxMsg._data[1] & 0x02) >> 1);
			rcv_findzero = uint8_t((rxMsg._data[2] & 0x10) >> 4);
			rcv_fault = uint8_t((rxMsg._data[2] & 0xE0) >> 5);
			rcv_opermod = uint8_t(rxMsg._data[2] & 0x0F);
		
			rcv_mot_torque = int8_t(rxMsg._data[3]) * 0.1f - 12;
			rcv_mot_speed = ESAF::Serialize::bytes_to_u16(&rxMsg._data[0], true);
			rcv_mot_speed_rl = (rcv_mot_speed >> 2) - 8000;
            rcv_mot_postion = ESAF::Serialize::bytes_to_s32(&rxMsg._data[4], true);
			
			fb_velocity = ((float)rcv_mot_speed_rl / 60.0f) * 360.0f / (float)mreductratio;
			fb_position = (float)rcv_mot_postion / ((float)mreductratio * (float)mppr) * 360.0f;
			fb_torque = 0.0f + rcv_mot_torque;
			cstatus |= (uint8_t)(rcv_startpos << 0);
			cstatus |= (uint8_t)(rcv_endpos << 1);
			cstatus |= (uint8_t)(rcv_findzero << 2);
			break;
		case 0x07:
			code2 = (rxMsg._data[1] & 0xC0) >> 4;
			code2 |= (rxMsg._data[5] & 0xC0) >> 6;
			rcv_posbusy = uint8_t((rxMsg._data[4] & 0x04) >> 2);
			cerrorcode = ((uint16_t)rxMsg._data[4]) | (code2 << 8);
			cerrorcode &= ~(0x00000004);			//unmask posbusy
			
			cvoltage = 0.0f + (float)rxMsg._data[0];
			if (rcv_posbusy == 1) {
				cstatus |= 0x08;
			}else if (rcv_posbusy == 0) {
				cstatus &= ~(0x08);
			}
			break;
		case 0x08:
			int16_t tcurrent = ESAF::Serialize::bytes_to_s16(&rxMsg._data[0], true);
			tcurrent = (tcurrent >> 2) * 0.1 - 800;
			ccurrent = 0.0f + tcurrent;	
			break;
		}
	}else if(++_offlineCnt > 200) {
		_offlineCnt = 300;
		_online = false;
	}
}

void Motor_EDriver::enable()
{
	_state = INITIALIZATION;
}

void Motor_EDriver::disable()
{
	_state = STOPPED;
	
	cnter_findzero = PMODE_FIND_ZERO;
	cnter_poswait = PMODE_SEND_TIME;
	cnter_posdt = PMODE_TIME;
	
	rcv_findzero = 0;
	rcv_fault = 1;

	setdown_posgo();
}

