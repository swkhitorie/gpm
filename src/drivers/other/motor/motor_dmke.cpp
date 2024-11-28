#include "motor_dmke.h"

Motor_DMKE::Motor_DMKE(ESAF::ebus_can *can, uint32_t id, uint8_t mode,
    bool autoZero, float ratio, uint32_t ppr, float offset) : 
    canDriver(can, id),
    reductradio(ratio),
    ppr(ppr),
    posoffset(offset),
    autozero(autoZero),
    opermode(mode),
    motionfactor(1),
    statusbit(0),
    tagfindzero(false),
    tagservoon(false),
    tagfatalerror(false),
    tagvoltagenormal(false),
    tagmotorwarning(false),
    tagmotionstop(false) {}
Motor_DMKE::~Motor_DMKE() {}

void Motor_DMKE::enable()
{
	_state = INITIALIZATION;
}

void Motor_DMKE::disable()
{
    fault_recover();
    
	ESAF::CANFrame msg;
	CanData * pTxData = (CanData *)&msg._data[0];
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._identifier = (0x0600 | _id);
	pTxData->data = 0x07;
	pTxData->index = 0x60402B;
	_can->write(&msg, 1);
    _can->write(&msg, 1);
    _can->write(&msg, 1);
	_online = false;
	_state = STOPPED;
}

void Motor_DMKE::init()
{
    ESAF::CANFrame msg;
    CanData *pTxData = (CanData *)&msg._data[0];
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;

    msg._identifier = 0x0000;
	msg._datalength = 8;
	pTxData->data = 0x00;
	pTxData->index = (0x0001 | (_id << 8));
	_can->write(&msg, 1);
    
    if (autozero) {
        msg._identifier = (0x0600 | _id);
        pTxData->data = 0x06;
        pTxData->index = 0x60602F;
        _can->write(&msg, 1);
    } else {
        setmode();
        _state = OPERATIONAL;
    }
	/* enable the driver */
	msg._identifier = (0x0600 | _id);
	pTxData->data = 0x0F;
	pTxData->index = 0x60402B;
	_can->write(&msg, 1);
	
	/* close down the feedback of 0x181 */
	msg._identifier = (0x0600 | _id);
	pTxData->data = 0x00;
	pTxData->index = 0x1A002F;
	_can->write(&msg, 1);

	/* close down the feedback of 0x281 */
	msg._identifier = (0x0600 | _id);
	pTxData->data = 0x00;
	pTxData->index = 0x1A002F;
	_can->write(&msg, 1);


	/* start finding zero */
	msg._identifier = (0x0600 | _id);
	pTxData->data = 0x1F;
	pTxData->index = 0x60402B;
	_can->write(&msg, 1);
}

void Motor_DMKE::setmode()
{
	ESAF::CANFrame msg;
	CanData * pTxData = (CanData *)&msg._data[0];
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._identifier = (0x0600 | _id);
	msg._datalength = 8;
	
	switch (opermode) {
	case MOTOR_MODE_CUR:
		pTxData->data = 0x04;
		pTxData->index = 0x60602F;
		break;
	case MOTOR_MODE_VEL:
		pTxData->data = 0x03;
		pTxData->index = 0x60602F;
		break;
	case MOTOR_MODE_POS:
		pTxData->data = 0x01;
		pTxData->index = 0x60602F;
		break;
	}
	_can->write(&msg, 1);
	
	/* setting the accel of current change */
	if (opermode == MOTOR_MODE_CUR) {
		pTxData->data = 0x2710;			
		pTxData->index = 0x211323;
		_can->write(&msg, 1);
	}
}

void Motor_DMKE::setmotion()
{
	ESAF::CANFrame msg;
	CanData * pTxData = (CanData *)&msg._data[0];
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._identifier = (0x0600 | _id);
	msg._datalength = 8;
	switch (opermode) {
	case MOTOR_MODE_CUR:
		/* see details in datasheet */
		pTxData->data = int(ctrl_current * 0.1f);
		pTxData->index = 0x23402B;
		pTxData->data *= motionfactor;
		break;		
	case MOTOR_MODE_VEL:
		/* degree/s -> pulse/s */
		//圈/s = ((度/s * 50 / 360) * 10 ) 
		pTxData->data = int(ctrl_velocity * reductradio * ppr / 36.0f);
		pTxData->index = 0x60FF23;
		pTxData->data *= motionfactor;
		break;
	case MOTOR_MODE_POS:
		break;
	}
	_can->write(&msg, 1);
}

void Motor_DMKE::getstatus()
{
	ESAF::CANFrame msg;
	CanData * pTxData = (CanData *)&msg._data[0];
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._identifier = (0x0600 | _id);
	msg._datalength = 8;
	
	/* read the status bit */
	pTxData->data = 0x00;
	pTxData->index = 0x604140;
	_can->write(&msg, 1);
	
	/* read the position */
	pTxData->data = 0x00;
	pTxData->index = 0x606440;
	_can->write(&msg, 1);
	
	/* read the velocity */
	pTxData->data = 0x00;
	pTxData->index = 0x606940;
	_can->write(&msg, 1);
	
	/* read the current */
	pTxData->data = 0x00;
	pTxData->index = 0x221C40;
	_can->write(&msg, 1);
}



void Motor_DMKE::poll_operation()
{
    switch (_state) {
    case STOPPED: break;
    case INITIALIZATION:
        init();
        _state = PRE_OPERATIONAL;
        break;
    case PRE_OPERATIONAL:
		if (autozero) {
			if (tagfindzero == true) {
				setmode();
				_state = OPERATIONAL;			
			}
		}else {
			_state = OPERATIONAL;
		}
        break;
    case OPERATIONAL:
        setmotion();
        break;
    }
    getstatus();
}


void Motor_DMKE::poll_recvparser(const ESAF::CANFrame *msg)
{
	CanData * pRxData = (CanData *)&msg->_data[0];
	
	if (msg->_identifier == (0x0580 | _id)) {
		_online = true;
		_offlineCnt = 0;
		switch (pRxData->index) {
		case 0x604060:
			/* driver enable or disable  */
			break;
		case 0x60FF60:
			/* setting velocity feedback */
			break;
		case 0x606943:
			/* reading velocity feedback */
			fb_velocity = pRxData->data / (reductradio * ppr / 36.0f);
			fb_velocity *= motionfactor;
			break;
		case 0x606443:
			/* reading position feedback */
			fb_position = pRxData->data / (reductradio * ppr / 360.0f) - posoffset;
			fb_position *= motionfactor;
			break;
		case 0x221C4B:
			/* reading current feedback */
			/* turn 10mA into A*/
			fb_current = short(pRxData->data) * 0.01f;
			break;
		case 0x60414B:
			/* reading status bit feedback */
			statusbit = uint16_t(pRxData->data);
			tagservoon = statusbit & (0x0001 << 2);
			tagfatalerror = statusbit & (0x0001 << 3);
			tagvoltagenormal = statusbit & (0x0001 << 4);
			tagmotorwarning = statusbit & (0x0001 << 7);
			tagmotionstop = statusbit & (0x0001 << 8);
			if (uint16_t(pRxData->data) & (0x0001 << 12)) {
				tagfindzero = true;
			}else {
				tagfindzero = false;
			}
			break;
		}
        
		if (tagservoon)    	MOTOR_SET_BIT(cstatus, 0x01);
		else				MOTOR_CLEAR_BIT(cstatus, 0x01);
		if (tagfindzero)    MOTOR_SET_BIT(cstatus, 0x02);
		else				MOTOR_CLEAR_BIT(cstatus, 0x02);		
		if (tagmotionstop)   MOTOR_SET_BIT(cstatus, 0x04);
		else				MOTOR_CLEAR_BIT(cstatus, 0x04);				
		
		if (tagfatalerror)   	MOTOR_SET_BIT(cerrorcode, 0x01);
		else					MOTOR_CLEAR_BIT(cerrorcode, 0x01);
		if (!tagvoltagenormal)   MOTOR_SET_BIT(cerrorcode, 0x02);
		else					MOTOR_CLEAR_BIT(cerrorcode, 0x02);	
		if (tagmotorwarning)   	MOTOR_SET_BIT(cerrorcode, 0x04);
		else					MOTOR_CLEAR_BIT(cerrorcode, 0x04);
        
	}else if (++_offlineCnt > 100) {
		_offlineCnt = 200;
		_online = false;
		_state = STOPPED;
	}
}

void Motor_DMKE::fault_recover()
{
    recover();
}

void Motor_DMKE::recover()
{
	ESAF::CANFrame msg;
	CanData * pTxData = (CanData *)&msg._data[0];
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._identifier = 0x0000;
	pTxData->data = 0x00;
	pTxData->index = (0x0081 | (_id << 8));
	_can->write(&msg, 1);
    
//	_online = false;
//	_state = STOPPED;
}

void Motor_DMKE::motionreverse(bool value)
{
	if(value) motionfactor = -1;
	else motionfactor = 1;
}

