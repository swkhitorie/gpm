fd: mavlink帧头, 第一个00: SYS ID, 第二个00: COMP ID
fd (\b[0-9a-fA-F]+\b) 00 00 (\b[0-9a-fA-F]+\b) ff be

查找内容:
	 fd (\b[0-9a-fA-F]+\b) 00 00
	 fd (\b[0-9a-fA-F]+\b) 00 00 (\b[0-9a-fA-F]+\b) 01 01 1
替换内容
	\nfd $1 00 00
QGC端
	fd (\b[0-9a-fA-F]+\b) 00 00 (\b[0-9a-fA-F]+\b) ff be
	\nfd $1 00 00 $2 ff be

 ## gcs_preflight_calibration

```cpp
/*
=================================  gyro/gyro temperature calibration  =================================
gcs -> autopilot
	MSG_COMMAND_LONG + CMD_PREFLIGHT_CALIBRATION + PARAM1 == 1 (gyro calibration)

autopilot processing
autopilot gyro/gyro temperature calibration complete:

autopilot -> gcs
	MSG_COMMAND_ACK + CMD_PREFLIGHT_CALIBRATION + RESULT_ACCEPTED + process == 100
*/

/*
=================================  accel calibration  =================================
gcs -> autopilot
	MSG_COMMAND_LONG + CMD_PREFLIGHT_CALIBRATION + PARAM5 == 1/2/3/4 (accel calibration)

point1:
	autopilot processing
	autopilot check attitude

		autopilot -> gcs
			MSG_COMMAND_LONG + CMD_ACCELCAL_VEHICLE_POS + PARAM1(
				ACCELCAL_VEHICLE_POS_LEVEL/ACCELCAL_VEHICLE_POS_LEFT
				ACCELCAL_VEHICLE_POS_RIGHT/ACCELCAL_VEHICLE_POS_NOSEDOWN
				ACCELCAL_VEHICLE_POS_NOSEUP/ACCELCAL_VEHICLE_POS_BACK)
		gcs press next->
		gcs -> autopilot
			MSG_COMMAND_ACK
		when attitude can satify now posture(depend on euler)
		sample now euler in this posture
		autopilot start sample accel data
		when sample data enough		
		autopilot -> gcs
			MSG_COMMAND_LONG + CMD_ACCELCAL_VEHICLE_POS + PARAM1(
				ACCELCAL_VEHICLE_POS_LEVEL/ACCELCAL_VEHICLE_POS_LEFT
				ACCELCAL_VEHICLE_POS_RIGHT/ACCELCAL_VEHICLE_POS_NOSEDOWN
				ACCELCAL_VEHICLE_POS_NOSEUP/ACCELCAL_VEHICLE_POS_BACK)		
			if then posture after ACCELCAL_VEHICLE_POS_BACK, then send:
			MSG_COMMAND_LONG + CMD_ACCELCAL_VEHICLE_POS + PARAM1
				ACCELCAL_VEHICLE_POS_SUCCESS / ACCELCAL_VEHICLE_POS_FAILED
				
		MSG_COMMAND_ACK + CMD_PREFLIGHT_CALIBRATION + RESULT_ACCEPTED + process == 100??
*/


/*
=================================  compass calibration  =================================
https://github.com/mavlink/mavlink/issues/1660
parameters need be full in COMPASS list
	it used: 
	MAV_CMD_DO_START_MAG_CAL
	MAV_CMD_DO_ACCEPT_MAG_CAL
	MAV_CMD_DO_CANCEL_MAG_CAL
	MAV_CMD_FIXED_MAG_CAL
	MAV_CMD_FIXED_MAG_CAL_FIELD
	MAG_CAL_PROGRESS
	MAG_CAL_REPORT
	
gcs -> autopilot
	MSG_COMMAND_LONG + MAV_CMD_DO_CANCEL_MAG_CAL + PARAM2
	
autopilot cancel last compass calibration
autopilot -> gcs
	MSG_COMMAND_ACK + MAV_CMD_DO_CANCEL_MAG_CAL + RESULT_ACCEPTED
	
gcs -> autopilot
	MSG_COMMAND_LONG + MAV_CMD_DO_START_MAG_CAL
	...	
	
autopilot start sample magnetometer data (MAV_CMD_FIXED_MAG_CAL_YAW  MAG_CAL_ACK)
	autopilot -> gcs
	MSG_ID_MAG_CAL_PROGRESS
		id:0 
		cal_mask: 1
		cal_status: 2
				MAG_CAL_NOT_STARTED/MAG_CAL_WAITING_TO_START/
				MAG_CAL_RUNNING_STEP_ONE/MAG_CAL_RUNNING_STEP_TWO/
				MAG_CAL_SUCCESS/MAG_CAL_FAILED...
		attempt: 1
		completion_pct: 0~100%
		completion_mask: 0, 128, 0, 15, 2, 255, 119, 0, 0, 128
		direction_x y z : 0
	
	when complete:
	autopilot -> gcs
	MSG_ID_MAG_CAL_REPORT
		id:0 
		cal_mask: 1
		cal_status: 4
		autosaved: 1
		fitness: 19.6939
		ofs_x: 41.0423
		ofs_y: -152.577
		ofs_z: 50.5915
		diag_x y z: 1
		offdiag_x y z: 0
		orientation_confidence: 11.4853
		
		old_orientation: 0 new_orientation:0 scale_factor: 0
		
		

	MSG_COMMAND_ACK + CMD_PREFLIGHT_CALIBRATION + RESULT_ACCEPTED + process == 100??
*/
```

## px4_qgc_link_process

```cpp
step1_未连接_px4广播:
	ATTITUDE (0x1E) 2HZ
	VFR_HUD (0x4A) 2HZ
	AHRS2 (0xB2) 2HZ
	HEARTBEAT (0x00) 1HZ
	TIMESYNC (0x6F) 0.1HZ
	
** QGC在Vehicle Settup界面解析心跳包的 MAV_AUTOPILOT 字段来生成后续配置界面 ***
	例如: 设置 MAV_AUTOPILOT字段为: MAV_AUTOPILOT_ARDUPILOTMEGA
		 响应QGC PARAM_LIST 请求
		 
step2_px4_qgc连接:
	
	qgc: PARAM_REQUEST_LIST(0x15)
		 COMMAND_LONG(0x4c)
		 
		 SYSTEM_TIME(0x02)
		 
		 (px4为非目标组件)MISSION_REQUEST_LIST (0x2b)
		 (px4为非目标组件)MISSION_ACK (0x2f)
		 (px4为非目标组件)REQUEST_DATA_STREAM(0x42) 交互
	
	px4:
		COMMAND_ACK(0x4d)
		
		AUTOPILOT_VERSION(0x94)
		STATUSTEXT(0xfd) * 6
		
		PARAM_VALUE(0x16) * 976
	
stepx_稳定后_px4广播:
	PARAM: STAT_RUNTIME 30s

	HEARTBEAT(0x00) 1HZ
	TIMESYNC (0x6F) 0.1HZ
	SYSTIEM_TIME(0x02) 3HZ
	SYS_STATUS(0x01) 2HZ	
	
	ATTITUDE(0x1E) 10HZ
	BATTERY_STATUS(0x93) 3HZ
	GLOBAL_POSITION_INT (0x21) 3HZ
	GPS_RAW_INT(0x18) 2HZ
	RAW_IMU(0x1b) 2HZ
	RC_CHANNELS(0x41) 2HZ
	SCALED_IMU2(0x74) 2HZ
	SCALED_PRESSURE(0x1d) 2HZ	
		
	AHRS(0xA3) 3HZ
	AHRS2(0xb2) 10HZ
	EKF_STATUS_REPORT(0xc1) 3HZ
	MEMINFO(0x98) 2HZ
		##MISSION_COUNT(44) 0HZ
	MISSION_CURRENT(0x2a) 2HZ
	NAV_CONTROLLER_OUTPUT(0x3e) 2HZ
		##PARAM_VALUE(22) 0.1HZ???
	POWER_STATUS(0x7d) 2HZ
	SERVO_OUTPUT_RAW(0x24) 2HZ
	TERRAIN_REPORT(0x88) 3HZ
	VFR_HUD (0x4A) 10HZ
	VIBRATION(0xf1) 3HZ
	
修改参数协议 Parameters Protocol
	QGC:
		---> PARAM_SET
	PX4:
		---> PARAM_VALUE (可返回多个同一类型的)
...	
```

## reset_order

FD 20 00 00 00 FF BE 4C 00 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F6 00 01 01 0E C8 
fd 20 00 00 f0 ff be 4c 00 00 00 00 80 3f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 f6 00 01 01 25 2c

FD 20 00 00 00 01 01 4C 00 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 F6 00 01 01 AB 75

fd: 帧头
20: hex数据长度
00: INCOMPAT_FLAGS
00: COMPAT_FLAGS
f0: 帧序号
ff: 系统ID
be: 组件ID
4c 00 00: 帧ID

00 00 80 3f:	float param1 (1.0f)
00 00 00 00:	float param2 
00 00 00 00:	float param3 
00 00 00 00:	float param4 
00 00 00 00:	float param5 
00 00 00 00:	float param6
00 00 00 00:	float param7
f6 00: command (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
01: target_system
01: target_component

25 2c: checksum