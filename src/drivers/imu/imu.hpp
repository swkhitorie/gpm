#ifndef __IMU_H_
#define __IMU_H_

#include "math/vector3.hpp"
#include <cstdint>

#define		SETTUP_OFFSET_CALIB_NUM		(200)
#define		GRAVITY_ACCEL				(9.7914f)

class IMU
{
public:
    IMU();
    ~IMU();
    uint8_t update(float dt);
	void diff_angvel(float dt, char method);
	void diff_angacc(float dt, char method);
	void calib_offset_angvel();
	virtual uint8_t device_update() = 0;
	virtual void copy_device_data() = 0;

private:
	Vector3f angle_lst[2];
	Vector3f angle_vel_offset;
	Vector3f angle_vel_sum;
	Vector3f angle_vel_lst[2];
    uint16_t angle_vel_offset_calib_cnt;  

public:
	Vector3f angle;
	Vector3f angle_vel;
	Vector3f angle_vel_diff;
	Vector3f angle_accel;
	Vector3f angle_accel_diff;
	Vector3f angle_accel_diff_filter;
	Vector3f accel;
	Vector3f free_accel;  
};

#endif
