#include "imu.hpp"

IMU::IMU() :
	angle(Vector3f(0, 0, 0)),
	angle_vel(Vector3f(0, 0, 0)),
	angle_vel_offset(Vector3f(0, 0, 0)),
	angle_vel_sum(Vector3f(0, 0, 0)),
	angle_vel_diff(Vector3f(0, 0, 0)),
	angle_accel(Vector3f(0, 0, 0)),
	angle_accel_diff(Vector3f(0, 0, 0)),
	angle_accel_diff_filter(Vector3f(0, 0, 0)),
	accel(Vector3f(0, 0, 0)),
	free_accel(Vector3f(0, 0, 0)),
	angle_vel_offset_calib_cnt(0) 
{
	angle_lst[0](0, 0, 0);
	angle_lst[1](0, 0, 0);
	angle_vel_lst[0](0, 0, 0);
	angle_vel_lst[1](0, 0, 0);
}
IMU::~IMU() {}

uint8_t IMU::update(float dt)
{
	uint8_t res = device_update();
	if (res != 0x00) {
		return res;
    }

	angle_lst[1](angle_lst[0].x, angle_lst[0].y, angle_lst[0].z);
	angle_vel_lst[1](angle_vel_lst[0].x, angle_vel_lst[0].y, angle_vel_lst[0].z);

	angle_lst[0](angle.x, angle.y, angle.z);
	angle_vel_lst[0](angle_vel.x, angle_vel.y, angle_vel.z);

    copy_device_data();

	calib_offset_angvel();
	
	diff_angvel(dt, '3');
	diff_angacc(dt, '3');

	return 0x00;
}

void IMU::diff_angvel(float dt, char method)
{
	if (method != '2' && method != '3')
		return;
	switch (method) {
	case '2':
		angle_vel_diff.x = (angle.x - angle_lst[0].x) / dt;
		angle_vel_diff.y = (angle.y - angle_lst[0].y) / dt;
		angle_vel_diff.z = (angle.z - angle_lst[0].z) / dt;
		break;
	case '3':
		angle_vel_diff.x = (3 * angle.x - 4 * angle_lst[0].x + angle_lst[1].x) / (2 * dt);
		angle_vel_diff.y = (3 * angle.y - 4 * angle_lst[0].y + angle_lst[1].y) / (2 * dt);
		angle_vel_diff.z = (3 * angle.z - 4 * angle_lst[0].z + angle_lst[1].z) / (2 * dt);		
		break;
	}
}

void IMU::diff_angacc(float dt, char method)
{
	if (method != '2' && method != '3')
		return;
	switch (method) {
	case '2':
		angle_accel_diff.x = (angle_vel.x - angle_vel_lst[0].x) / dt;
		angle_accel_diff.y = (angle_vel.y - angle_vel_lst[0].y) / dt;
		angle_accel_diff.z = (angle_vel.z - angle_vel_lst[0].z) / dt;
		break;
	case '3':
		angle_accel_diff.x = (3 * angle_vel.x - 4 * angle_vel_lst[0].x + angle_vel_lst[1].x) / (2 * dt);
		angle_accel_diff.y = (3 * angle_vel.y - 4 * angle_vel_lst[0].y + angle_vel_lst[1].y) / (2 * dt);
		angle_accel_diff.z = (3 * angle_vel.z - 4 * angle_vel_lst[0].z + angle_vel_lst[1].z) / (2 * dt);
		break;
	}
}

void IMU::calib_offset_angvel()
{
	if (angle_vel_offset_calib_cnt < SETTUP_OFFSET_CALIB_NUM) {
		angle_vel_sum += angle_vel;
		angle_vel_offset_calib_cnt++;
	}else if (angle_vel_offset_calib_cnt == SETTUP_OFFSET_CALIB_NUM) {
		angle_vel_offset.x = angle_vel_sum.x / SETTUP_OFFSET_CALIB_NUM;
		angle_vel_offset.y = angle_vel_sum.y / SETTUP_OFFSET_CALIB_NUM;
		angle_vel_offset.z = angle_vel_sum.z / SETTUP_OFFSET_CALIB_NUM;
		angle_vel_offset_calib_cnt++;
	}else if (angle_vel_offset_calib_cnt > SETTUP_OFFSET_CALIB_NUM) {
		angle_vel_offset_calib_cnt = SETTUP_OFFSET_CALIB_NUM + 1;
		angle_vel -= angle_vel_offset;
	}
}

