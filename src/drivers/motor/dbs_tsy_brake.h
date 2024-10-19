
#ifndef __DBS_TSY_BRAKE_H_
#define __DBS_TSY_BRAKE_H_

#include "motor.hpp"
#include "../candriver.hpp"

class DBS_Brake : public canDriver, public Motor
{
public:
    enum DBSID {
    	DBSSEND_ID = 0x154,
    	DBSRECV_ID = 0x142
    };

    enum DBSSTATUS {
        STATUS_NORMAL = 0x00,
        STATUS_OUTLIQUID = 0x01,
        STATUS_ERROR = 0x02,
        STATUS_RESERVED = 0x03
    };

    enum DBSERROR {
        DBS_NORMAL = 0x00,
        DBS_PRESS_SENSOR_ERROR = 0x01,
        DBS_PRESS_LOW_VOLTAGE = 0x02,
        DBS_PRESS_TEMPER_HIGH = 0x04,
        DBS_PRESS_MOTOR_ERROR = 0x08,
        DBS_PRESS_CONTROLER_ERROR = 0x10,
        DBS_PRESS_ORDER_MISS = 0x20
    };

    DBS_Brake() = delete;
    DBS_Brake(ESAF::ebus_can *can, uint32_t id = DBSSEND_ID);
    ~DBS_Brake();

	virtual void enable();
	virtual void disable();
	virtual void poll_operation();
	virtual void poll_recvparser(const ESAF::CANFrame *msg);
	
    virtual void init();
    virtual void getstatus();
    virtual void setmode();
    virtual void setmotion();
    virtual void fault_recover();
    
public:
	void setctrlpress(float ctrl_press);
	float getpress();

public:
	bool isZeroPress;
	uint8_t ZeroPressTimes;

    uint8_t statusbits;
    uint8_t verrorcode;

	float ctrl_press;
	float fb_press;
};

#endif
