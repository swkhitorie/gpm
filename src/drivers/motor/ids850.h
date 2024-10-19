
#ifndef __IDS850_H_
#define __IDS850_H_

#include "motor.hpp"
#include "../candriver.hpp"

class IDS850 : public canDriver, public Motor 
{
public:
	enum IDSMOTORID {
        LEFT_ID = 0x01,
        RIGHT_ID = 0x02
	};

	enum IDSGROUPID {
        GRP_1 = 0x00,
	};

	enum IDSFUNCODE {
		WRITE_FUNC = 0x1A,
		READ_FUNC = 0x2A,
		READBACK_FUNC = 0x2B
	};

	enum IDSREGADDR {
		IDS_MOTORSTARTEND = 0x00,          /*!< 0x0001 start  0x0000 end */
		IDS_POSMODE = 0x02,                /*!< select position mode -- 0x00d0 (CAN bus) */
		IDS_POSMODE_VELLIMIT = 0x1d,       /*!< limit of velocity mode 8192 -> 3000rpm */
		IDS_POSMODE_RELABS = 0x51,         /*!< switch of velocity mode 0x0000 -- absolute control 0x0001 -- relative control */
		IDS_POSVALUE_HIGHSHORT = 0x50,     /*!< position write order, high 16bits */ 
		IDS_POSVALUE_LOWSHORT = 0x05,      /*!< position write order, low 16bits */ 
		IDS_RD_BUSVOLTS = 0xE1,            /*!< request of data, bus voltage */
        IDS_RD_BUSCURRENT = 0xE2,          /*!< request of data, bus current */
        IDS_RD_RPM = 0xE4,                 /*!< request of data, velocity rpm */
		IDS_RD_POSVALUE_HIGHSHORT = 0xE8,  /*!< request of data, position feedback -- high 16bits */
		IDS_RD_POSVALUE_LOWSHORT = 0xE9,   /*!< request of data, position feedback -- low 16bits */
		IDS_RD_ERRORCODE = 0xE3            /*!< request of data, driver error code */
	};

    IDS850() = delete;
    IDS850(ESAF::ebus_can *can, uint32_t id = LEFT_ID);
    ~IDS850();

	virtual void enable();
	virtual void disable();
	virtual void poll_operation();
	virtual void poll_recvparser(const ESAF::CANFrame *msg);
	
    virtual void init();
    virtual void getstatus();
    virtual void setmode();
    virtual void setmotion();
    virtual void fault_recover();
    
	void sendcommmand(uint8_t groupnum, uint8_t funcode, uint8_t reg1addr,
                    uint16_t data1, uint16_t reg2addr, uint16_t data2);
    
protected:
	uint8_t step_init;
	uint8_t step_status;

	uint8_t errorcode;
	uint16_t rcv_busvolts;
    uint16_t rcv_buscurrent;
	uint32_t rcv_pos;
    int16_t rcv_rpm;
};

#endif
