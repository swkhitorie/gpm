
#ifndef __CANDRIVER_H_
#define __CANDRIVER_H_

#include "ebus_can.hpp"
#include "CANFrame.hpp"

class canDriver
{
public:
    enum CANDEVICE_STATE
    {
        INITIALIZATION,
        PRE_OPERATIONAL,
        OPERATIONAL,
        STOPPED
    };

    canDriver() = delete;
    canDriver(ESAF::ebus_can *can, uint32_t id) :
        _id(id),
        _idtag(id),
        _state(STOPPED),
        _offlineCnt(0),
        _onlineCnt(0),
        _online(false),
        _frequence(0),
        _dt(0),
        _can(can)
    {

    }
    
    ~canDriver()
    {
        
    }

	virtual void enable() = 0;
	virtual void disable() = 0;
	virtual void poll_operation() = 0;
	virtual void poll_recvparser(const ESAF::CANFrame *msg) = 0;

    virtual void init() {}
    virtual void getstatus() {}
    virtual void setmode() {}
    virtual void setmotion() {}
    virtual void fault_recover() {}

	bool get_drvonline() { return _online; }
	uint8_t get_state() { return _state; }
	
protected:
    uint32_t _id;
    uint32_t _idtag;

    uint8_t _state;

    uint32_t _offlineCnt;
    uint32_t _onlineCnt;
    bool _online;

    float _frequence;
    float _dt;

    ESAF::ebus_can *_can;
};

#endif
