
#ifndef __MOTOR_EDRIVER_H_
#define __MOTOR_EDRIVER_H_

#include "motor.hpp"
#include "../candriver.hpp"
#include "serialize.hpp"

class Motor_EDriver : public canDriver, public Motor
{
public:
	
	enum MOTORMODE {
		DRIVER_SPEED = (0x01),
		DRIVER_POSITION = (0x02)
	};
	
	enum RUNMODE {
		MODE_SPEED = 0x04,
		MODE_POSITION = 0x05,
		MODE_FINDZERO = 0x09,
		MODE_SHUTDOWN = 0x00,
		MODE_STANDBY = 0x01,
		MODE_STARTUP = 0x02,
	};
	
	enum FACTOROFFSET {
		OFFSET_MODE_SPEED = (-32000),
		OFFSET_MODE_ZERO_SPEED = (-2000),
		FACTOR_PMODE_MAX_SPEED = (300),
		FACTOR_PMODE_MAX_ACCEL = (300),
	};
	
	enum WAITTIME {
		PMODE_SEND_TIME = (15),
		PMODE_FIND_ZERO = (500),
		PMODE_TIME = (10),
	};
	
    Motor_EDriver() = delete;
    Motor_EDriver(ESAF::ebus_can *can, uint32_t id, uint8_t drvmode = DRIVER_SPEED, uint32_t ratio = 50, uint32_t ppr = 2500);
    ~Motor_EDriver();

	virtual void enable();
	virtual void disable();
	virtual void poll_operation();
	virtual void poll_recvparser(const ESAF::CANFrame *msg);
	
    virtual void init();
    virtual void getstatus();
    virtual void setmode();
    virtual void setmotion();
    virtual void fault_recover();

    void pre_operational();
	
public:
	bool wait_nonblock(int16_t *timer, int16_t max_value);

	void sendcommand_1(	uint16_t speedreq, uint8_t opermodreq, 
						uint8_t fault_rst_en, uint8_t unlock, 
						uint8_t pos_go, int32_t posreq);
	void sendcommand_2(	float pmode_max_speed, float pmode_max_accel, 
						uint16_t zero_spd, uint8_t tq_lim, 
						uint16_t tq_req, uint8_t roll_counter, uint8_t crc_check);
    void setup_posgo();
	void setdown_posgo();
	
public:
	uint32_t mreductratio;
	uint32_t mppr;

	bool config_poweron;
	uint8_t drv_mode;

	bool posprocess;
	uint8_t posctrl;
	uint8_t posgo;

	int16_t cnter_findzero;
	int16_t cnter_poswait;
	int16_t cnter_posdt;

	uint8_t rcv_fault;
	uint8_t rcv_findzero;
	uint8_t rcv_opermod;
	uint8_t rcv_startpos;
	uint8_t rcv_endpos;
	uint8_t rcv_posbusy;

	int8_t rcv_mot_torque;
	uint16_t rcv_mot_speed;
	int32_t rcv_mot_speed_rl;
	int32_t rcv_mot_postion;
};

#endif
