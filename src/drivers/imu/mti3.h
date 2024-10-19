#ifndef __MTI3_H_
#define __MTI3_H_

#include "serialize.hpp"
#include "imu.hpp"
#include "ebus_uart.hpp"

/**
 * @brief about MTI3
 *
 *	IMU坐标方向(右手坐标系):
 *	 ——————
 *	|		 ________
 *	|   |
 *	|   |
 *	|   |
 *					—————————————
 *					|  银色晶振					|							^^^
 *					|							|							|||			X轴
 *					|							|							|||
 *					|							|							|||
 *					|			IC缺口			|							|||
 *					|							|							|||
 *					|							/
 *					|						/
 *					|					/
 *					—————————
 *			<——————
 *			<——————				Y轴											
 *			<——————							(小圆点)
 *			
 *	MTi3_Setting..
 *	BaudRate ---  921600
 *	Orientation and InertialData ----- 100HZ
 *		datapack     FA FF MID:36 DataLen:3C(60)
 *			Euler Angles 	 	20 30 (ID + Size + data -> 15bytes)
 *			Acceleration 	 	40 20 (ID + Size + data -> 15bytes)
 *			RateofTurn	 	 	80 20 (ID + Size + data -> 15bytes)
 *			Free Acceleration	40 30 (ID + Size + data -> 15bytes)
 *			CheckSum
 *	High Rate Data Gyro					 ----- 1000HZ
 *		datapack     FA FF MID:36 DataLen:15
 *			RateofTurnHR 	 80 40 (ID + Size + data -> 15bytes)
 *			CheckSum
 *	High Rate Data Acc					 ----- 1000HZ	
 *		datapack     FA FF MID:36 DataLen:15
 *			AccelerationHR 40 40 (ID + Size + data -> 15bytes)
 *			CheckSum
*/

/**
 * @brief MTI3_IMU configuratoin, 1--enable, 0--diable
*/
#define MTI3_PACK_HEAD									(0xFA)
#define MTI3_PACK_BID									(0xFF)
#define MTI3_PACK_MID									(0x36)

#define MTI3_ORIENTATION_EULER_ANGLE					1
#define MTI3_ORIENTATION_EULER_ANGLE_ID				(0x2030)
#define MTI3_ORIENTATION_EULER_ANGLE_LEN			(0x0C)


#define MTI3_INERTIAL_ACCELERATION						1
#define MTI3_INERTIAL_ACCELERATION_ID				(0x4020)
#define MTI3_INERTIAL_ACCELERATION_LEN				(0x0C)


#define MTI3_INERTIAL_RATEOFTURN						1
#define MTI3_INERTIAL_RATEOFTURN_ID					(0x8020)
#define MTI3_INERTIAL_RATEOFTURN_LEN				(0x0C)


#define MTI3_INERTIAL_FREE_ACCELERATION					1
#define MTI3_INERTIAL_FREE_ACCELERATION_ID			(0x4030)
#define MTI3_INERTIAL_FREE_ACCELERATION_LEN			(0x0C)


#define MTI3_HIGHRATE_ACCELERATION						0
#define MTI3_HIGHRATE_ACCELERATION_ID				(0x8040)
#define MTI3_HIGHRATE_ACCELERATION_LEN				(0x0C)


#define MTI3_HIGHRATE_RATEOFTURN						0
#define MTI3_HIGHRATE_RATEOFTURN_ID					(0x4040)
#define MTI3_HIGHRATE_RATEOFTURN_LEN				(0x0C)

/**
 * @brief mti3 highrate data enable flag
*/
//#define MTI3_HIGHRATE_ENABLE
#define DATALEN_100HZ_PACK		15 * MTI3_ORIENTATION_EULER_ANGLE \
								+ 15 * MTI3_INERTIAL_ACCELERATION \
								+ 15 * MTI3_INERTIAL_RATEOFTURN \
								+ 15 * MTI3_INERTIAL_FREE_ACCELERATION
															
#define DATALEN_1000HZ_PACK		15 * MTI3_HIGHRATE_ACCELERATION \
								+ 15 * MTI3_HIGHRATE_RATEOFTURN			
#if (DATALEN_100HZ_PACK == 0) && (DATALEN_1000HZ_PACK == 0)
	#error no any data in imu_mti3 !!!
#endif

#pragma pack(1)
struct MTI3_data_lowspeed {
	uint8_t head;
	uint8_t bid;
	uint8_t mid;
	uint8_t len; 
 	
	#if	MTI3_ORIENTATION_EULER_ANGLE
		uint16_t dataid_euler;
		uint8_t datalen_euler;
		float data_euler_1;
		float data_euler_2;
		float data_euler_3;	
	#endif
	#if	MTI3_INERTIAL_ACCELERATION
		uint16_t dataid_accel;
		uint8_t datalen_accel;
		float data_accel_1;
		float data_accel_2;
		float data_accel_3;	
	#endif
	#if	MTI3_INERTIAL_FREE_ACCELERATION
		uint16_t dataid_free_accel;
		uint8_t datalen_free_accel;
		float data_free_accel_1;
		float data_free_accel_2;
		float data_free_accel_3;	
	#endif
	#if	MTI3_INERTIAL_RATEOFTURN
		uint16_t dataid_rate_turn;
		uint8_t datalen_rate_turn;
		float data_rate_turn_1;
		float data_rate_turn_2;
		float data_rate_turn_3;	
	#endif
	uint8_t sum;				
};
#pragma pack()

#pragma pack(1)
struct MTI3_data_highspeed {
	uint8_t head;
	uint8_t bid;
	uint8_t mid;
	uint8_t len; 
 	
	#if	MTI3_HIGHRATE_ACCELERATION
		uint16_t dataid_hr_accel;
		uint8_t datalen_hr_accel;
		float data_hr_accel_1;
		float data_hr_accel_2;
		float data_hr_accel_3;	
	#endif
	#if	MTI3_HIGHRATE_RATEOFTURN
		uint16_t dataid_hr_rate_turn;
		uint8_t datalen_hr_rate_turn;
		float data_hr_rate_turn_1;
		float data_hr_rate_turn_2;
		float data_hr_rate_turn_3;	
	#endif
	uint8_t sum;				
};
#pragma pack()

class Mti3 : public IMU
{
public:
    Mti3() = delete;
	Mti3(ESAF::ebus_uart *com);
    ~Mti3();

	virtual uint8_t device_update();
	virtual void copy_device_data();

	void devbuf_retrieve(uint16_t len);
	
	bool is_online();
private:
	Vector3f raw_angle;

	Vector3f raw_accel;
	Vector3f raw_gyro;
	Vector3f raw_free_accel;

	Vector3f raw_hr_accel;
	Vector3f raw_hr_gyro;

	uint8_t min_packlen;
	bool updated;
	bool online;
	int lostcnt;

	struct MTI3_data_lowspeed raw_lowspeed_pack;
	struct MTI3_data_highspeed raw_highspeed_pack;
	ESAF::ebus_uart *_com;
	uint8_t retrieve[50];
};

#endif
