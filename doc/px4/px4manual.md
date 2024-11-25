## 1.构建

`reference version: v1.14.0`

official:
	https://docs.px4.io/main/zh/
	https://docs.px4.io/main/zh/advanced_config/parameter_reference.html
	https://px4.io/
	https://github.com/PX4
	http://pix.1yuav.com/
	https://ardupilot.org/copter/docs/parameters.html

```bash
git clone -b v1.14.0 https://github.com/PX4/PX4-Autopilot.git --recursive
git submodule update --init --recursive
```

```c
./src
	./systemcmds: nuttex系统指令, 例如param, pwm, reboot等
	./templates: 功能模块(module)示例代码
	./drivers: 
		bootloader + 硬件驱动框架及类(例如barometer imu gps px4io...)
	./examples: 
		示例代码, (用于二次开发??)
	./include:
		通用头文件, 包括硬件平台头文件, 数据容器...
	./lib:
		重要库, 例如 
		adsb: 监视系统库
		airspeed: 空速传感器模块
		atmosphere: 大气层传感模块
		avoidance: 避障模块
		battery: 电池模块
		controllib控制库
		geo地理坐标系/经纬度转换库
		matrix矩阵库
		cdev硬件驱动类库
		crc校验库
		drivers硬件类库
		sensor_calibration传感校准库
		mathlib算法库
		matrix: 矩阵库
		l1: L1制导库
		motion_planning: 运动规划模块
		rate_control: 角速度控制
		terrain_estimation: 地形估计
		world_magnetic_model: 地球磁场模型
         ...
	./modules
		功能模块
		airship_att_control: 飞艇姿态控制模块
		airspeed_selector: 空速传感器模块
		attitude_estimator_q： 姿态估计模块
		battery_status: 电池状态模块
		camera_feedback: 相机数据模块
		commander: px4飞行安全状态机
		control_allocator: 控制分配器
		dataman: 数据存储模块??
		ekf2: EKF算法模块
		esc_battery: 电调模块
		events: 全局事件模块
		flight_mode_manager: 飞行模式管理器
		******************
		fw_att_control: 				固定翼控制模块----姿态控制
		fw_autotune_attitude_control:	 固定翼控制模块----
		fw_path_navigation: 			固定翼控制模块----路径规划
		fw_rate_control:			    固定翼控制模块----速度控制??
		******************
		gimbal: 云台模块
		gyro_calibration: 陀螺仪校准模块
		gyro_fft: 陀螺仪FFT滤波器模块
		land_detector: 着地检测模块
		landing_target_estimator: 着地目标估计模块
		load_mon:
		local_position_estimator: LPE基于扩展卡尔曼滤波器的姿态和位置估计器模块
		logger: 日志模块
		mag_bias_estimator: 磁力计误差估计模块
		manual_control: 手动控制模块
		mavlink: mavlink协议模块(外部)
		******************
		mc_att_control: 				多旋翼控制模块----姿态控制
		mc_autotune_attitude_control:	多旋翼控制模块----
		mc_hover_thrust_estimator: 		多旋翼控制模块----
		mc_pos_control: 				多旋翼控制模块----位置控制
		mc_rate_control:				多旋翼控制模块----速度控制??		
		******************
		microdds_client: micro-odds通讯模块????
		muorb: uORB(对象请求代理器)模块??
		navigator: 自主导航飞行控制模块(自主起飞/降落/返航/任务/gps失效保护)
		payload_deliverer: 载荷运输器模块??
		px4iofirmware: px4通用io模块??
		rc_update: 遥控器模块
		replay: 数据回放模块
		******************
		rover_pos_control:				车体控制模块----位置控制
		******************
		sensors: 传感器模块
		simulation: 仿真模块
		temperature_compensation: 温度补偿模块
		******************
		uuv_att_control:				无人潜水器控制模块----姿态控制
		uuv_pos_control:				无人潜水器控制模块----位置控制
		******************
		******************
		vtol_att_control:				垂直起降控制模块----姿态控制
		******************
```

## 2.bootloader解析

https://www.helloworld.net/p/4730169897

## 4.移植

```cpp
	// platform/defines.h:
	/**
		修改#include <px4_platform_common/defines.h>
		-> 保留数学相关宏
		-> 在PX4_ISFINITE声明中, 去掉常量表达式修饰符
		将__builtin_isfinite, 替换为std::isinf
	
	*/

	// src/lib/geo[gps地理坐标转换库]
	/**
		geo.h: 
			删除 
			1. #include <drivers/drv_hrt.h> 引用
			2. p205 inline void initReference(double lat_0, double lon_0)
			3. CMakeLists.txt 和 test文件/文件夹
			在p172的
			MapProjection(double lat_0, double lon_0)
			{
				// 此处添加一个0
				initReference(lat_0, lon_0, 0);
			}
			#include <lib/mathlib/mathlib.h>
			#include <lib/matrix/matrix/math.hpp>
			修改为:
			#include "lib/mathlib/mathlib.h"
			#include "lib/matrix/matrix/math.hpp"	
	
	 */
			
			
	// src/lib/matrix[矩阵向量等数学库]
	/**
		helper_functions.hpp
			替换
			#include <px4_platform_common/defines.h> ->
			#include "platform/defines.h" //将源文件中有用的宏保留
			去掉CMakeLists.txt 和 test文件/文件夹
	
	*/

	// src/lib/mathlib[常用数学库/滤波器库]
	/**
		Functions.hpp
			替换
			#include <px4_platform_common/defines.h>
			#include <matrix/matrix/math.hpp>
			为
			#include "platform/defines.h"
			#include "lib/matrix/matrix/math.hpp"
			
		Utilities.hpp:
			#include <matrix/math.hpp>
			替换为:
			#include "lib/matrix/matrix/math.hpp"
	*/
```



