#ifndef  DYP_A21_H_
#define  DYP_A21_H_

#include "../candriver.hpp"

class dypa21 : public canDriver
{
public:
	enum MAGIC {
		ObstacleDis = 100,
	};
	
	dypa21() = delete;
	dypa21(ESAF::ebus_can *can, uint32_t id = 1);
	~dypa21();
	
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
    //0x01 读取处理值数据
    //0x02 读取实时值数据
    //0x03 读取温度值数据
    //0x04 读取角度等级
	uint8_t _step;

    const float ONSTACLE_DIS = 80.0f;

    float _distance_h;   //处理值
    float _distance_r;    //实时距离
    float _temperature;           //温度
    float _angleRating;           //角度等级
    bool _hasObstacle; 
};



#endif