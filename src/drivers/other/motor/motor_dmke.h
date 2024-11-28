#ifndef __MOTOR_DMKE_H_
#define __MOTOR_DMKE_H_

#include "motor.hpp"
#include "../candriver.hpp"

#pragma pack(1)
struct CanData
{
	uint32_t index;	/*!< CANOpen index */
	int32_t  data;	/*!< CANOpen data */
};
#pragma pack()

class Motor_DMKE : public canDriver, public Motor
{
public:
    Motor_DMKE() = delete;
    Motor_DMKE(ESAF::ebus_can *can, uint32_t id, uint8_t mode = MOTOR_MODE_VEL,
            bool autoZero = true, float ratio = 50, 
            uint32_t ppr = 10000, float offset = 0);
    ~Motor_DMKE();

    virtual void enable();
    virtual void disable();
    virtual void poll_operation();
    virtual void poll_recvparser(const ESAF::CANFrame *msg);
    
    virtual void init();
    virtual void getstatus();
    virtual void setmode();
    virtual void setmotion();
    virtual void fault_recover();

    void recover();
    void motionreverse(bool value);

protected:
    const float reductradio;
    const uint32_t ppr;
    const float posoffset;
    bool autozero;
    uint8_t opermode;

    int motionfactor;

    uint16_t statusbit;
	bool tagfindzero;
	bool tagservoon;
	bool tagfatalerror;
    bool tagvoltagenormal;
    bool tagmotorwarning;
    bool tagmotionstop;
};

#endif
