
#ifndef __FSP8_H_
#define __FSP8_H_

#include "serialize.hpp"
#include "imu.hpp"
#include "ebus_uart.hpp"

#pragma pack(1)
struct FSP8Pack {
	uint8_t head;
	uint8_t head2;
	uint16_t id;
	uint16_t length;

	uint32_t timer;
	uint32_t reserve_1;
	uint32_t reserve_2;
	uint32_t reserve_3;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float imu_temper;
	
	uint8_t check_1;
	uint8_t check_2;
	uint8_t check_3;
	uint8_t check_4;	
};
#pragma pack()

class Fsp8 : public IMU
{
public:
	enum FSP8LEN {
		FSPACKLEN = 54
	};
	Fsp8() = delete;
	Fsp8(ESAF::ebus_uart *com);
	~Fsp8();

	virtual uint8_t device_update();
	virtual void copy_device_data();

	void devbuf_retrieve(uint16_t len);

public:
	float timer;
	Vector3f raw_accel;
	Vector3f raw_gyro;

	bool updated;
	bool online;
	int lostcnt;

	struct FSP8Pack fs_origin;
	ESAF::ebus_uart *_com;
	uint8_t retrieve[50];
};

#endif
