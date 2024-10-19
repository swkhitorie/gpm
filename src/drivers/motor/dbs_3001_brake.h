
#ifndef __DBS_3001_BRAKE_H_
#define __DBS_3001_BRAKE_H_

#include "motor.hpp"
#include "../candriver.hpp"

class DBS_3001_Brake : public canDriver, public Motor 
{
public:
    enum DBSID {
    	DBSSEND_ID = 0x154,
    	DBSRECV_ID = 0x142,
		DBSRECV_ID2 = 0x143
    };

    enum DBSMODE {
        MODE_WIRE = 0x00,
        MODE_EXHAUST_GAS_AUTO = 0x01,
        MODE_PEDAL = 0x02,
        MODE_EXHAUST_GAS_HANDLE = 0x03
    };

    enum DBSSTATUS {
        STATUS_NORMAL = 0x00,
        STATUS_WARNING = 0x01,
        STATUS_ERROR = 0x02
    };

    DBS_3001_Brake() = delete;
    DBS_3001_Brake(ESAF::ebus_can *can, uint32_t id = DBSSEND_ID);
    ~DBS_3001_Brake();

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
	uint8_t status(void);
	uint8_t errorcode(void);
	void setctrlpress(float ctrl_press);
	float getpress();

private:
    uint8_t statusbits;
	uint8_t park_warning;
	uint8_t running_mode;
	uint8_t press_busy;
	uint8_t estop_flag;
	uint8_t pedal_flag;

    uint8_t verrorcode;
	uint16_t warning_code;
	uint8_t real_press;

	uint8_t pedal_throttle;

	float ctrl_press;
	float fb_press;
};

#endif
