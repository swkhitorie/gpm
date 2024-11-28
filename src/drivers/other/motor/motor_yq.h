

#ifndef __MOTOR_YQ_H_
#define __MOTOR_YQ_H_

#include "motor.hpp"
#include "../candriver.hpp"

/*
 * @brief
 *	yq motor error code
 	Bit		故障描述			报警次数		
	0		电机霍尔故障			1						控制器和电机之间的信号线没有接好。
	1		油门踏板故障			2						油门没有回零，或者油门踏板坏。注意重启控制器时默认会显示故障，当自检通过后，故障会消失。
	2		电流保护重启			3						异常保护告警
	3		相电流过流			4						异常保护告警
	4		电压故障				5						电压太低或太高，超出控制器允许范围。
	5		防盗告警信号			6						防盗信号报警
	6		电机过温				7						电机温度太低或太高超出使用范围
	7		控制器过温			8						控制器温度太低或太高超出使用范围
	8		相电流溢出			9						异常保护告警
	9		相电流零点故障		10						控制器内部告警
	10		相线短路故障			11						相线短路，或电机故障。
	11		线电流零点故障		12						控制器内部告警
	12		MOSFET上桥故障		13						控制器上桥损坏
	13		MOSFET下桥故障		14						控制器下桥损坏
	14		峰值线电流保护		15						异常保护告警
	15		保留
*/
class Motor_YQ : public canDriver, public Motor 
{
public:
	enum MOTORYQ_ID {
        MOTOR_YQ_ID_1 = 0x10F8E3F3,
        MOTOR_YQ_ID_2 = 0x10F8E4F3
	};

	enum MOTORYQ_MODE {
        RUNMODE_NULL = 0x00,
        RUNMODE_FORWARD = 0x01,
        RUNMODE_BACK = 0x02,
        RUNMODE_LOWSPEED = 0x03       
	};

	enum MOTORYQ_CTLMODE {
        CTLMODE_TORQUE = 0x22,
        CTLMODE_THROTTLE = 0x28,
        CTLMODE_ROTATE = 0x02,     
	};

    Motor_YQ() = delete;
    Motor_YQ(ESAF::ebus_can *can, uint32_t id = MOTOR_YQ_ID_2);
    ~Motor_YQ();
	
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
	void sendcommand(uint8_t runmode, uint16_t throttle,
		    uint8_t ctrlmode, uint16_t torque, uint16_t rpm);
			
    void setdriving(bool value);
	
	uint8_t get_motortemp(void);
	uint8_t get_ctrlertemp(void);
protected:
    bool driving;

	uint16_t errorcode;
	uint8_t	statusbit;

	uint16_t rcv_rpm;
	uint16_t rcv_volts;
	uint16_t rcv_current;
	uint16_t rcv_cntrotate;
	uint8_t rcv_mode;

public:
	uint8_t	rcv_motortemp;
	uint8_t	rcv_ctrlertemp;	
};

#endif

