#include "ids850.h"

IDS850::IDS850(ESAF::ebus_can *can, uint32_t id) : 
    canDriver(can, id),
    step_init(0),
    step_status(1),
    errorcode(0),
    rcv_busvolts(0),
	rcv_buscurrent(0),
    rcv_pos(0) {}
IDS850::~IDS850() {}

void IDS850::enable()
{
	_state = INITIALIZATION;
	step_init = 0;
	step_status = 1;
}

void IDS850::disable()
{
	sendcommmand(GRP_1, WRITE_FUNC, IDS_POSMODE, 0x00D0, IDS_MOTORSTARTEND, 0x0000);
	step_init = 0;
	step_status = 1;
    _state = STOPPED;
}

void IDS850::init()
{
	/* setMotor Enable and Position Mode */
	sendcommmand(GRP_1, WRITE_FUNC, IDS_POSMODE,0x00D0, IDS_MOTORSTARTEND,0x0001);

	/* setMotor VelLimit 3000RPM and Absolute PositonControl */
	sendcommmand(GRP_1, WRITE_FUNC, IDS_POSMODE_VELLIMIT, 0x1055, IDS_POSMODE_RELABS,0x0000);
    
    fault_recover();
}

void IDS850::setmode() {}
void IDS850::setmotion()
{
	int16_t pos_high;
	int16_t pos_low;
	int32_t position = (int32_t)ctrl_position;

	pos_high = (int16_t)((position & 0xFFFF0000) >> 16);
	pos_low = (int16_t)(position & 0x0000FFFF);

	sendcommmand(GRP_1, WRITE_FUNC, IDS_POSVALUE_HIGHSHORT, pos_high, IDS_POSVALUE_LOWSHORT, pos_low);
}

void IDS850::getstatus()
{
	/*  get the position feedback */
	sendcommmand(GRP_1, READ_FUNC, IDS_RD_POSVALUE_HIGHSHORT, 0x0000, IDS_RD_POSVALUE_LOWSHORT, 0x0000);

	/* get the errorcode, rpm */
	sendcommmand(GRP_1, READ_FUNC, IDS_RD_ERRORCODE, 0x0000, IDS_RD_RPM, 0x0000);

	/* get the busVoltage, bus current */
	sendcommmand(GRP_1, READ_FUNC, IDS_RD_BUSVOLTS, 0x0000, IDS_RD_BUSCURRENT, 0x0000);
}

void IDS850::fault_recover()
{
	sendcommmand(GRP_1, WRITE_FUNC, 0x4a, 0x0000, 0xFF, 0x0000);
}


void IDS850::poll_operation()
{
	switch (_state) {
		case STOPPED: break;
		case INITIALIZATION:
			init();
			_state = OPERATIONAL;
			break;
		case PRE_OPERATIONAL:
			getstatus();
			_state = OPERATIONAL;
			break;
		case OPERATIONAL:
			setmotion();
			_state = PRE_OPERATIONAL;
			break;
	}
}

void IDS850::poll_recvparser(const ESAF::CANFrame *msg)
{
    ESAF::CANFrame rxMsg = (*msg);

	uint8_t rd_reg[2];
	uint8_t rd_func  = rxMsg._data[1];
	rd_reg[0] = rxMsg._data[2];
	rd_reg[1] = rxMsg._data[5];
    
    uint16_t pos_high = 0;
    uint16_t pos_low = 0;
    
	if ((((uint8_t)rxMsg._identifier) == _id) && rd_func == READBACK_FUNC) {
		_offlineCnt = 0;
		_online = true;

		for (int i = 0; i < 2; i++) {
			switch (rd_reg[i]) {
			case IDS_RD_ERRORCODE:
				errorcode = rxMsg._data[4 + 3 * i];
				break;
			case IDS_RD_POSVALUE_HIGHSHORT:
                pos_high = (uint16_t)((rxMsg._data[3 + 3 * i] << 8) | rxMsg._data[4 + 3 * i]);
                rcv_pos |= pos_high << 16;
				break;
			case IDS_RD_POSVALUE_LOWSHORT:
                pos_low = (uint16_t)((rxMsg._data[3 + 3 * i] << 8) | rxMsg._data[4 + 3 * i]);
                rcv_pos |= pos_low;
				break;
			case IDS_RD_BUSVOLTS:
				rcv_busvolts = ((rxMsg._data[3 + 3 * i] << 8) | rxMsg._data[4 + 3 * i]);
				break;
            case IDS_RD_BUSCURRENT:
				rcv_buscurrent = ((rxMsg._data[3 + 3 * i] << 8) | rxMsg._data[4 + 3 * i]);
				break;
            case IDS_RD_RPM:
                rcv_rpm = (int16_t)((rxMsg._data[3 + 3 * i] << 8) | rxMsg._data[4 + 3 * i]);
                break;
			default:
				break;				
			}
		}
        
        cerrorcode = errorcode;
        cerrorcode &= 0xFFFFFFFE;
        
        uint8_t isnormal = cerrorcode & 0x00000001;
        uint8_t iscontrolsource = (uint8_t)(cerrorcode & 0x00000080) >> 7;
        if (isnormal == 1) {
            cstatus |= 0x01;
        }else if (isnormal == 0) {
            cstatus &= ~0x01;
        }
        if (iscontrolsource == 1) {
            cstatus |= 0x02;
        }else if (iscontrolsource == 0) {
            cstatus &= ~0x02;
        }
        
        ccurrent = 0.0f + (float)rcv_buscurrent / 100.0f;
        cvoltage = 0.0f + (float)rcv_busvolts;
        
		fb_position = 0.0f + (float)rcv_pos;
        fb_velocity = 0.0f + (float)rcv_rpm / 8192.0f * 3000;
	}else if(++_offlineCnt > 200) {
		_offlineCnt = 300;
		_online = false;
	}
}

void IDS850::sendcommmand(uint8_t groupnum, uint8_t funcode, uint8_t reg1addr,
    uint16_t data1, uint16_t reg2addr, uint16_t data2)
{
	ESAF::CANFrame msg;
	msg._data[0] = groupnum;
	msg._data[1] = funcode;
	msg._data[2] = reg1addr;
	msg._data[3] = uint8_t((data1 & 0xFF00) >> 8);
	msg._data[4] = uint8_t(data1 & 0x00FF);
	msg._data[5] = reg2addr;
	msg._data[6] = uint8_t((data2 & 0xFF00) >> 8);
	msg._data[7] = uint8_t(data2 & 0x00FF);
	msg._identifier = _id;
	msg._idtype = ESAF::CANFrame::STANDARD_ID;
	msg._frametype = ESAF::CANFrame::DATA_FRAME;
	msg._datalength = 8;
	_can->write(&msg, 1);	
}

