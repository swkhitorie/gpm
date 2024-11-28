#include "dypa21.h"

dypa21::dypa21(ESAF::ebus_can *can, uint32_t id) :
    canDriver(can, id),
    _step(1), 
    _distance_h(0.0f),
    _distance_r(0.0f),
    _temperature(0.0f),
    _angleRating(0.0f),
    _hasObstacle(false) {}
dypa21::~dypa21() {}
 
void dypa21::enable(){}
void dypa21::disable(){}
void dypa21::poll_operation(){ setmotion(); }    
void dypa21::poll_recvparser(const ESAF::CANFrame *msg)
{
	ESAF::CANFrame rxMsg = (*msg);
    if (rxMsg._identifier == (_id | 0x0520)) {

        _distance_r = (float)(((uint16_t)rxMsg._data[3] << 8) | rxMsg._data[4]);
        _distance_r /= 10.0f;
        
        if(_distance_r > 500.0f){
            _distance_r = 500.0f;
        }
        
        if (_distance_r <= ONSTACLE_DIS)
            _hasObstacle = true;
        else
            _hasObstacle = false;
    }
}

void dypa21::init() {}
void dypa21::getstatus() {}
void dypa21::setmode() {}
void dypa21::fault_recover() {}
void dypa21::setmotion()
{
    ESAF::CANFrame msg;
    msg._identifier = (_id | 0x0520);
	msg._datalength = 6;
    msg._data[0] = _id;
    msg._data[1] = 0x03;
    msg._data[2] = 0x01;
    msg._data[3] = 0x01;
    msg._data[4] = 0x00;
    msg._data[5] = 0x01;
	_can->write(&msg, 1); 
}

