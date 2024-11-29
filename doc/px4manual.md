## 1.build

`reference version: v1.11.2`

official:
	https://docs.px4.io/main/zh/
	https://docs.px4.io/main/zh/advanced_config/parameter_reference.html
	https://px4.io/
	https://github.com/PX4
	http://pix.1yuav.com/
	https://ardupilot.org/copter/docs/parameters.html

```bash
git clone -b v1.11.2 https://github.com/PX4/PX4-Autopilot.git --recursive
git submodule update --init --recursive
```

```c
// bootloader解析 https://www.helloworld.net/p/4730169897
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

`px4可用算法:`

```cpp
可用的估計器:


## EKF2 attitude, position and wind states estimator (recommended) - 
	An extended Kalman filter estimating attitude, 3D position / velocity and wind states.

## LPE position estimator (deprecated) - 
	An extended Kalman filter for 3D position and velocity states. (注意) LPE is deprecated. It works (at time of writing, in PX4 v1.14) but is no longer supported or maintained

## Q attitude estimator - 
	A very simple, quaternion based complementary filter for attitude. It does not require a GPS, magnetometer, or barometer.

如何启用不同的估计器
	对于多旋翼和垂直起降，使用参数SYS_MC_EST_GROUP在以下配置之间进行选择（固定翼不支持LPE）。
	SYS_MC_EST_GROUP: 1  Q Estimator 启用, LPE 启用, EKF2 闲置
	SYS_MC_EST_GROUP: 2  Q Estimator 闲置, LPE 闲置, EKF2 启用
	SYS_MC_EST_GROUP: 0  Q Estimator 启用, LPE 闲置, EKF2 闲置
```



## 2.porting

### 2.1 lib/matrix + lib/mathlib

#### 2.1.1 filter

```cpp
/**
 * @file AlphaFilter.hpp   一阶RC滤波器
 */
	/**
	 * class AlphaFilter<typename T>
	 */
	T updateCalculation(const T &sample);
	float _cutoff_freq{0.f};
	float _alpha{0.f};
	T _filter_state{};
	
	AlphaFilter() = default;
	explicit AlphaFilter(float alpha) : _alpha(alpha) {}
	~AlphaFilter() = default;
 
	void setParameters(float sample_interval, float time_constant)
	bool setCutoffFreq(float sample_freq, float cutoff_freq)
		// 设置一阶截止频率
	void setAlpha(float alpha) { _alpha = alpha; }
	void reset(const T &sample) { _filter_state = sample; }
	const T &update(const T &sample)
 	const T &getState() const { return _filter_state; }
	float getCutoffFreq() const { return _cutoff_freq; }
 
	// 简单的一阶RC滤波器
	template <typename T>
	T AlphaFilter<T>::updateCalculation(const T &sample) { 
		return _filter_state + _alpha * (sample - _filter_state); 
	}
		
	// 四元数的一阶RC滤波器
	template <> inline
	matrix::Quatf AlphaFilter<matrix::Quatf>::updateCalculation(const matrix::Quatf &sample)
	{
		matrix::Quatf q_error(_filter_state.inversed() * sample);
		q_error.canonicalize(); // prevent unwrapping
		return _filter_state * matrix::Quatf(
			matrix::AxisAnglef(_alpha * matrix::AxisAnglef(q_error)));
	}
 
/**
 * @file LowPassFilter2p.hpp   二阶巴特沃斯滤波器
 */
 	/**
	 * class LowPassFilter2p<typename T>
	 */
	T _delay_element_1{}; // buffered sample -1
	T _delay_element_2{}; // buffered sample -2
	float _a1{0.f};
	float _a2{0.f};
	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};
	float _cutoff_freq{0.f};
	float _sample_freq{0.f};
	
 	LowPassFilter2p() = default;
	LowPassFilter2p(float sample_freq, float cutoff_freq)
	void set_cutoff_frequency(float sample_freq, float cutoff_freq)
		// 设置采样频率, 截止频率
	inline T apply(const T &sample)
	inline void applyArray(T samples[], int num_samples)
 		// 迭代运行
	float get_cutoff_freq() const { return _cutoff_freq; }
	float get_sample_freq() const { return _sample_freq; }
	float getMagnitudeResponse(float frequency) const;
	T reset(const T &sample)
	void disable()
		// 变量重置
 
 
/**
 * @file MedianFilter.hpp  中值窗口滤波器
 */
 	/**
	 * class MedianFilter<typename T, int WINDOW = 3>
	 */
 	T _buffer[WINDOW] {};
	uint8_t _head{0};
 
 	MedianFilter() = default;

	void insert(const T &sample)
		// 向数组插入值
	T median()
		// 拷贝排序后取中值
	T apply(const T &sample)
		// 迭代运行
	
 /**
 * @file NotchFilter.hpp  陷波滤波器
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * @author Samuel Garcin <samuel.garcin@wecorpindustries.com>
 */
	/**
	 * class NotchFilter<typename T>
	 */
	 bool setParameters(float sample_freq, float notch_freq, float bandwidth);
		//设置频率, 采样频率, 陷波频率, 频率带宽
	 inline T apply(const T &sample)
	 inline void applyArray(T samples[], int num_samples)
		//迭代运行
 	 float getNotchFreq() const { return _notch_freq; }
	 float getBandwidth() const { return _bandwidth; }
	 void getCoefficients(float a[3], float b[3]) const
	 float getMagnitudeResponse(float frequency) const
	 void setCoefficients(float a[2], float b[3])
	 bool initialized() const { return _initialized; }
	 void reset() { _initialized = false; }
	 void reset(const T &sample)
	 void disable()
	 ......
	 
 /**
 * @file second_order_reference_model.hpp  具有可选速率前馈的二阶系统的实现。
 * @author Thomas Stastny <thomas.stastny@auterion.com>
 */
	/**
	 * class SecondOrderReferenceModel<typename T>
	 */
	SecondOrderReferenceModel() = default;
	SecondOrderReferenceModel(const float natural_freq, const float damping_ratio)
		// 设置参数-- 系统频率和系统阻尼比
	enum DiscretizationMethod {
		kForwardEuler,
		kBilinear
	};
	void setDiscretizationMethod(const DiscretizationMethod method)	
		// 设置用于状态积分的时间离散化方法
	bool setParameters(const float natural_freq, const float damping_ratio)
		// 设置参数-- 系统频率和系统阻尼比
	const T &getState() const { return filter_state_; }
	const T &getRate() const { return filter_rate_; }
	const T &getAccel() const { return filter_accel_; }
	void update(const float time_step, const T &state_sample, const T &rate_sample = T())
	void reset(const T &state, const T &rate = T())
		// 更新/重置系统状态
	.........
```

#### 2.1.2 Functions_SearchMin

```cpp
/**
 * @file Seach.hpp 二进制黄金分割搜索
 */
template<typename _Tp>
_Tp abs_t(_Tp val) { return ((val > (_Tp)0) ? val : -val); }
	// 类型安全的取绝对值函数

/**
 * @function 求函数极值的黄金分割搜索
 * @params arg1 区间下限
 * @params arg1 区间上限
 * @params *fun 迭代函数
 * @params tol 迭代精度
 */	
// 求函数极值的黄金分割搜索
// @param
template<typename _Tp>
inline const _Tp goldensection(const _Tp &arg1, const _Tp &arg2, _Tp(*fun)(_Tp), const _Tp &tol)


/**
 * @file Functions.hpp 反复使用的相当简单的数学函数的集合
 */
template<typename T>
int signNoZero(T val) 
	//类型安全的取符号函数

inline int signFromBool(bool positive)
	//bool转int
	
template<typename T>
T sq(T val)
	// 取平方
	
template<typename T>
const T expo(const T &value, const T &e)
	// 指数曲线函数实现  value [-1,1]函数输入  e [0,1] 用于设置线性形状和立方体形状之间的比率
	//e: 0-纯线性函数
	//e: 1-纯三次函数

const T superexpo(const T &value, const T &e, const T &g)
	// 其为1/(1-x)函数
	// @param value[-1,1]函数的输入值
	// @param e[0,1]函数参数，用于设置线性形状和立方体形状之间的比率（请参阅expo）
	// @param g[0,1）设置SuperExpo形状的函数参数
	//      0-纯expo函数
	//      0.99-非常强的弯曲曲线，在最大杆输入之前保持为零

const T deadzone(const T &value, const T &dz)
	// 死区连续线性函数
	// @param value[-1,1]函数的输入值
	// @param dz[0,1）deazone和完整跨度之间的比率
	//      0-无死区，线性-1到1
	//      0.5-死区为跨度的一半[-0.5,0.5]
	//      0.99-几乎整个跨度都是死区

template<typename T>
const T expo_deadzone(const T &value, const T &e, const T &dz)
	// 含有死区的指数曲线


/*
 * 两个点之间的常数/线性函数
 * y_high          -------
 *                /
 *               /
 *              /
 * y_low -------
 *         x_low   x_high
 */
 
template<typename T>
const T interpolate(const T &value, const T &x_low, const T &x_high, const T &y_low, const T &y_high)

/*
 * 常数、分段线性、常数函数，具有1/N个大小的区间和N个角点作为参数
 * y[N-1]               -------
 *                     /
 *                   /
 * y[1]            /
 *               /
 *              /
 *             /
 * y[0] -------
 *        0 1/(N-1) 2/(N-1) ... 1
 */
template<typename T, size_t N>
const T interpolateN(const T &value, const T(&y)[N])
/*
 * 以N个角点为参数的常数、分段线性、常数函数
 * y[N-1]               -------
 *                     /
 *                   /
 * y[1]            /
 *               /
 *              /
 *             /
 * y[0] -------
 *          x[0] x[1] ... x[N-1]
 * x[N] 角坐标必须按升序排序
 */
template<typename T, size_t N>
const T interpolateNXY(const T &value, const T(&x)[N], const T(&y)[N])

/*
 * 平方根，具有固定交点的线性函数（1,1）
 *                     /
 *      linear        /
 *                   /
 * 1                /
 *                /
 *      sqrt     |
 *              |
 * 0     -------
 *             0    1
 */
template<typename T>
const T sqrt_linear(const T &value)


/*
 * 点a和b之间的线性插值。
 * s=0 return a
 * s=1 returns b
 */
template<typename T>
const T lerp(const T &a, const T &b, const T &s)

template<typename T>
constexpr T negate(T value)

template<>
constexpr int16_t negate<int16_t>(int16_t value)


//此函数计算汉明权重，即计算在给定整数中设置的位数。
template<typename T>
int countSetBits(T n)

inline bool isFinite(const float &value)

inline bool isFinite(const matrix::Vector3f &value)
```

#### 2.1.3 Utilites_TrajMath

```cpp
/**
 * @file Utilities.hpp   实用的函数接口
 */
static constexpr float sq(float var) { return var * var; }
	//返回两个浮点数的平方
	
inline matrix::Dcmf taitBryan312ToRotMat(const matrix::Vector3f &rot312)
	//将Tait Bryan 312从第1帧到第2帧的旋转序列转换为从第2帧旋转到第1帧的相应旋转矩阵
	// rot312(0) - 第一次旋转是绕Z轴的RH旋转（rad）
	// rot312(1) - 第二次旋转是绕X轴的RH旋转（rad）
	// rot312(2) - 第三次旋转是绕Y轴的RH旋转（rad）	
	// See http://www.atacolorado.com/eulersequences.doc
	
inline matrix::Dcmf quatToInverseRotMat(const matrix::Quatf &quat)	
	// 将四元数转换为旋转矩阵
	
inline bool shouldUse321RotationSequence(const matrix::Dcmf &R)
	// 判断使用何种旋转序列
	
inline float getEuler321Yaw(const matrix::Dcmf &R)
inline float getEuler312Yaw(const matrix::Dcmf &R)
inline float getEuler321Yaw(const matrix::Quatf &q)
inline float getEuler312Yaw(const matrix::Quatf &q)
	// 从方向余弦矩阵或四元数中提取在对应的欧拉角旋转序列的偏航角度
inline float getEulerYaw(const matrix::Dcmf &R)
	// 总接口
	
inline matrix::Dcmf updateEuler321YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
inline matrix::Dcmf updateEuler312YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
inline matrix::Dcmf updateYawInRotMat(float yaw, const matrix::Dcmf &rot_in)
	// 用外部偏航角矫正已有的方向余弦矩阵
	// 检查要使用的欧拉旋转序列，并更新旋转矩阵中的偏航
	
	
	
/**
 * @file TrajMath.hpp  用于轨迹生成的函数集合
 */	
	
inline float computeMaxSpeedFromDistance(
	const float jerk, const float accel, 
	const float braking_distance, const float final_speed)
	// 在给定期望速度的情况下，计算轨道上的最大可能速度，剩余距离、最大加速度和最大急动
	// 我们假设一个延迟为2*accel/jerk的恒定加速度分布
	// (从相反的最大加速度达到所需加速度的时间）
	// 方程式：vel_final ^2=vel_initial ^2-2*加速度*（x-vel_initical*2*加速度/急动）
	// @param jerk 最大制动
	// @param accel 最大加速度
	// @param braking_distance 到所需点的距离
	// @param final_speed 车辆达到制动状态时的剩余速度
	// @return maximum speed
	
inline float computeMaxSpeedInWaypoint(const float alpha, const float accel, const float d)
	/*
	计算由两条长度为“d”的线段定义的圆中的最大切向速度，
	这两条线段形成一个V形，以角度“a”打开。该圆与两个线段的末端相切，如下所示
	*      \\
	*      | \ d
	*      /  \
	*  __='___a\
	*      d
	@param alpha 两条线段之间的alpha角度
	@param accel 最大横向加速度
	@param d 两条线段的长度
	@return 最大切向速度
	*/
	
inline float computeBrakingDistanceFromVelocity(const float velocity, const float jerk, const float accel,
		const float accel_delay_max)
	// 计算给定最大加速度、最大急动和最大延迟加速度的制动距离
	// 我们假设一个延迟为accel_delay_max/jerk的恒定加速度分布
	// （从相反的最大加速度达到所需加速度的时间）
	// vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
	// @param velocity 速度初始速度
	// @param jerk 最大制动
	// @param accel 制动操纵过程中的最大目标加速度
	// @param accel_delay_max 定义上述延迟的加速度
	// @return 制动距离
	
	
inline float getMaxDistanceToCircle(
	const matrix::Vector2f &pos, const matrix::Vector2f &circle_pos, 
	float radius, const matrix::Vector2f &direction)
	// 给定从点指向圆的方向向量，计算点和圆之间的最大距离。点可以在圆内也可以在圆外
/*
 *                  _
 *               ,=' '=,               __
 *    P-->------/-------A   Distance = PA
 *       Dir   |    x    |
 *              \       /
 *               "=,_,="
*/
	// 要求解的方程
	// ||(point - circle_pos) + direction_unit * distance_to_circle|| = radius
	// @param pos 点的位置
	// @param circle_pos 圆心的位置
	// @param radius 圆的半径
	// @param direction 从点指向圆的矢量
	// @return 在矢量指示的方向上，点与圆之间的最长距离，或者如果矢量不指向圆，则为NAN
```

#### 2.1.4 Welford

```cpp
/**
 * @file WelfordMean.hpp   均值和均方差方法类
 */
	/**
	 * class WelfordMean<typename Type = float>
	 */
	Type _mean{};
	Type _M2{};
	Type _mean_accum{};  ///< kahan summation algorithm accumulator for mean
	Type _M2_accum{};    ///< kahan summation algorithm accumulator for M2
	uint16_t _count{0};
	bool update(const Type &new_value)
		// 更新状态, 计算均值,方差,均方差等...
	bool valid() const { return _count > 2; }
	auto count() const { return _count; }
	void reset()
	Type mean() const { return _mean; }
		// 获取均值
	Type variance() const { return _M2 / (_count - 1); }
		// 获取方差
	Type standard_deviation() const { return std::sqrt(variance()); }
		// 获取标准差

/**
 * @file WelfordMeanVector.hpp   计算向量均值和协方差的Welford在线算法
 */
	/**
	 * class WelfordMeanVector<typename Type, size_t N>
	 */
	matrix::Vector<Type, N> _mean{};
	matrix::Vector<Type, N> _mean_accum{};
	matrix::SquareMatrix<Type, N> _M2{};
	matrix::SquareMatrix<Type, N> _M2_accum{};
	uint16_t _count{0};
	
	bool update(const matrix::Vector<Type, N> &new_value)
		// 更新状态, 计算均值,方差,均方差等...
	bool valid() const { return _count > 2; }
	auto count() const { return _count; }
	void reset()
	matrix::Vector<Type, N> mean() const { return _mean; }
		// 获取均值
	matrix::Vector<Type, N> variance() const { return _M2.diag() / (_count - 1); }
		// 获取方差
	matrix::SquareMatrix<Type, N> covariance() const { return _M2 / (_count - 1); }
		// 获取协方差矩阵
	Type covariance(int x, int y) const { return _M2(x, y) / (_count - 1); }
		// 获取协方差
```

#### 2.1.5 axisangle_dcm_dcm2_euler_quaternion

```cpp
/**
 * @file AxisAngle.hpp 轴角类定义
 */	 
	/**
	 * class AxisAngle<typename Type> : public Vector3<Type>
	 */	 
	explicit AxisAngle(const Type data_[3]) : Vector3<Type>(data_)
	AxisAngle() = default;
	AxisAngle(const Matrix31 &other) : Vector3<Type>(other)
	AxisAngle(const Quaternion<Type> &q)
		// 通过四元数表示的旋转来初始化轴角
	AxisAngle(const Dcm<Type> &dcm)
		// 通过方向余弦矩阵来初始化
	AxisAngle(const Euler<Type> &euler)
		// 通过欧拉角来初始化
	AxisAngle(Type x, Type y, Type z)
		// 直接通过轴角来初始化
	AxisAngle(const Matrix31 &axis_, Type angle_)
	Vector3<Type> axis()	//返回轴角所代表的坐标系
	Type angle() // 返回轴角

/**
 * @file Euler.hpp 欧拉角定义
 */	 
	/**
	 * class Euler<typename Type> : public Vector<Type, 3>
	 */	 	 
	 Euler() = default;
	 Euler(const Vector<Type, 3> &other) 
	 Euler(const Matrix<Type, 3, 1> &other)
	 Euler(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
		// 通过欧拉角来初始化: phi_: roll  theta_: pitch psi_: yaw
	 Euler(const Dcm<Type> &dcm)
		// 通过方向余弦矩阵来初始化
	 Euler(const Quaternion<Type> &q) : Vector<Type, 3>(Euler(Dcm<Type>(q)))
		// 通过四元数来初始化
	 inline Type phi() const
	 inline Type theta() const
	 inline Type psi() const
	 inline Type &phi()
	 inline Type &theta()
	 inline Type &psi()
		// 取出对应的欧拉角
	
/**
 * @file Dcm.hpp 方向余弦矩阵类(三维旋转)
 */	 
	/**
	 * class Dcm<typename Type> : public SquareMatrix<Type, 3>
	 */		
	Dcm() : SquareMatrix<Type, 3>(eye<Type, 3>()) {}
	explicit Dcm(const Type data_[3][3]) : SquareMatrix<Type, 3>(data_)
	explicit Dcm(const Type data_[9]) : SquareMatrix<Type, 3>(data_)
	Dcm(const Matrix<Type, 3, 3> &other) : SquareMatrix<Type, 3>(other)
	Dcm(const Quaternion<Type> &q)
		// 通过四元数初始化
	Dcm(const Euler<Type> &euler)
		// 通过欧拉角初始化
	Dcm(const AxisAngle<Type> &aa)
		// 通过具体姿态角初始化
	Vector3<Type> vee() const
		// 通过方向余弦矩阵返回其方向向量
	void renormalize()
		// 单位化
		
/**
 * @file Dcm2.hpp 方向余弦矩阵类(二维旋转)
 */	 
	/**
	 * class Dcm2<typename Type> : public SquareMatrix<Type, 2>
	 */		
	Dcm2() : SquareMatrix<Type, 2>(eye<Type, 2>()) {}
	explicit Dcm2(const Type data_[2][2]) : SquareMatrix<Type, 2>(data_)
	explicit Dcm2(const Type data_[4]) : SquareMatrix<Type, 2>(data_)
	Dcm2(const Matrix<Type, 2, 2> &other) : SquareMatrix<Type, 2>(other)
	Dcm2(const Type angle)
		// 通过角度来初始化
	void renormalize()
	
	
/**
 * @file Quaternion.hpp 四元数类
 */	 
	/**
	 * class Quaternion<typename Type> : public Vector4<Type>
	 */			
	explicit Quaternion(const Type data_[4])
	Quaternion()
	Quaternion(const Matrix41 &other)
	Quaternion(const Dcm<Type> &R)
		// 通过方向余弦矩阵来初始化
	Quaternion(const Euler<Type> &euler)
		// 通过欧拉角来初始化
	Quaternion(const AxisAngle<Type> &aa)
		// 通过轴角来初始化
	Quaternion(const Vector3<Type> &src, const Vector3<Type> &dst, const Type eps = Type(1e-5))
		// 通过旋转前的向量和旋转后的向量来初始化
	Quaternion(Type a, Type b, Type c, Type d)
	
	Quaternion operator*(const Quaternion &p)
	void operator*=(const Quaternion &other)
	Quaternion operator*(Type scalar)
	void operator*=(Type scalar)
		// 四元数标量乘法和旋转乘法
	Matrix41 derivative1(const Matrix31 &w) const
		// 利用角速度矩阵w和四元数, 计算导数(下一时刻的角速度矩阵)??
	Matrix41 derivative2(const Matrix31 &w) const
		// 利用角速度矩阵w和四元数, 计算导数(上一时刻的角速度矩阵)??
	static Quaternion expq(const Vector3<Type> &u)
		// 计算三维矢量的四元数指数??
	static Dcm<Type> inv_r_jacobian(const Vector3<Type> &u)
		// 计算四元数对数u的逆雅克比矩阵
	void invert()
		// 四元数反转
	Quaternion inversed() const
		// 返回反转四元数
	void canonicalize()
		// 四元数正则化
	Quaternion canonical() const
		// 返回四元数的正则形式
	void rotate(const AxisAngle<Type> &vec)
		// 四元数旋转
	Vector3<Type> rotateVector(const Vector3<Type> &vec) const
		// 返回描述从形参向量经过四元数描述的旋转旋转后的向量
	Vector3<Type> rotateVectorInverse(const Vector3<Type> &vec) const
		// 上述过程的逆过程
	Vector3<Type> imag() const
		// 四元数的虚分量
	Vector3<Type> dcm_z() const
		// 与姿态四元数/最后一个正交单位基向量相对应的物体z轴==等效旋转矩阵的最后一列
```

#### 2.1.6 filter_integration

```cpp
/**
 * @file filter.hpp 卡尔曼滤波
 */	 

/**
 *  P 预测协方差矩阵
 *  C 状态观测迭代矩阵
 *  R 状态协方差矩阵
 *  r 状态矩阵???
 */
template<typename Type, size_t M, size_t N>
int kalman_correct(
	const Matrix<Type, M, M> &P,
	const Matrix<Type, N, M> &C,
	const Matrix<Type, N, N> &R,
	const Matrix<Type, N, 1> &r,
	Matrix<Type, M, 1> &dx,
	Matrix<Type, M, M> &dP,
	Type &beta
)
{
	SquareMatrix<Type, N> S_I = SquareMatrix<Type, N>(C * P * C.T() + R).I();
	Matrix<Type, M, N> K = P * C.T() * S_I;
	// k-> kalman gain
	// K = PH.T(HPH.T + R)^-1
	dx = K * r;
	beta = Scalar<Type>(r.T() * S_I * r);
	
	// 预测协方差矩阵迭代增量
	dP = K * C * P * (-1);
	return 0;
}

/**
 * @file integration.hpp rk4-->四阶龙格库塔积分
 */	 

template<typename Type, size_t M, size_t N>
int integrate_rk4(
	Vector<Type, M> (*f)(Type, const Matrix<Type, M, 1> &x, const Matrix<Type, N, 1> &u),
	const Matrix<Type, M, 1> &y0,
	const Matrix<Type, N, 1> &u,
	Type t0,
	Type tf,
	Type h0,
	Matrix<Type, M, 1> &y1
)
{
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	Type t1 = t0;
	y1 = y0;
	Type h = h0;
	Vector<Type, M> k1, k2, k3, k4;

	if (tf < t0) { return -1; } // make sure t1 > t0

	while (t1 < tf) {
		if (t1 + h0 < tf) {
			h = h0;

		} else {
			h = tf - t1;
		}

		k1 = f(t1, y1, u);
		k2 = f(t1 + h / 2, y1 + k1 * h / 2, u);
		k3 = f(t1 + h / 2, y1 + k2 * h / 2, u);
		k4 = f(t1 + h, y1 + k3 * h, u);
		y1 += (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);
		t1 += h;
	}

	return 0;
}
```

#### 2.1.7 matrix_slice_scalar

```cpp

/**
 * @file Matrix.hpp 矩阵基础定义
 */
	/**
	 * class Matrix<Type, M, N>
	 */
	Type _data[M][N]
	====Constructors param:
		const Type data_[M * N]
		const Type data_[M][N]
		const Matrix &other
		const Slice<Type, M, N, P, Q> &in_slice
		const ConstSlice<Type, M, N, P, Q> &in_slice
	====basical function to access data:
		const Type &operator()(size_t i, size_t j) const 
		Type &operator()(size_t i, size_t j)
		Matrix<Type, M, N> &operator=(const Matrix<Type, M, N> &other)
		void copyTo(Type dst[M * N]) const
		void copyToColumnMajor(Type dst[M * N]) const
	====Matrix operation:
		Matrix<Type, M, P> operator*(const Matrix<Type, N, P> &other) const
			// A(M*N) * B(N*P)
		Matrix<Type, M, N> emult(const Matrix<Type, M, N> &other) const
			// element cal : A(M*N) * B(M*N)
		Matrix<Type, M, N> edivide(const Matrix<Type, M, N> &other) const
			// element cal : A(M*N) / B(M*N)
		Matrix<Type, M, N> operator+(const Matrix<Type, M, N> &other) const
			// A(M*N) + B(M*N)
		Matrix<Type, M, N> operator-(const Matrix<Type, M, N> &other) const
			// A(M*N) - B(M*N)
		Matrix<Type, M, N> operator-() const
			// A(M*N) * -1
		void operator+=(const Matrix<Type, M, N> &other)
			// A(M*N) = A(M*N) + B(M*N)
		void operator-=(const Matrix<Type, M, N> &other)
			// A(M*N) = A(M*N) - B(M*N)
		void operator*=(const Matrix<Type, N, P> &other)
			// A(M*P) = A(M*N) * B(N*P)
	====Scalar operation:
		Matrix<Type, M, N> operator*(Type scalar) const
			// A1(M*N) = A(M*N) * b (all element cal)
		inline Matrix<Type, M, N> operator/(Type scalar) const
			// A1(M*N) = A(M*N) * 1/b (all element cal)
		Matrix<Type, M, N> operator+(Type scalar) const
			// A1(M*N) = A(M*N) + b (all element cal)
		inline Matrix<Type, M, N> operator-(Type scalar) const
			// A1(M*N) = A(M*N) + -b (all element cal)
		void operator*=(Type scalar)
			// A(M*N) = A(M*N) * b (all element cal)
		void operator/=(Type scalar)
			// A(M*N) = A(M*N) * 1/b (all element cal)
		inline void operator+=(Type scalar)
			// A(M*N) = A(M*N) + b (all element cal)
		inline void operator-=(Type scalar)
			// A(M*N) = A(M*N) + -b (all element cal)
		bool operator==(const Matrix<Type, M, N> &other) const
			// if( A(M*N) == B(M*N) )
		bool operator!=(const Matrix<Type, M, N> &other) const
			// if( !(A(M*N) == B(M*N)) )
	====Functions:
		void write_string(char *buf, size_t n) const
			// print matrix element
		void print(float eps = 1e-9) const
			// print matrix element
		Matrix<Type, N, M> transpose() const
			// Matrix transpose(矩阵转置)
		inline Matrix<Type, N, M> T() const
			// Matrix transpose(矩阵转置)
		ConstSlice<Type, P, Q, M, N> slice(size_t x0, size_t y0) const
		Slice<Type, P, Q, M, N> slice(size_t x0, size_t y0)
			// 返回特定的矩阵切片
		ConstSlice<Type, 1, N, M, N> row(size_t i) const
		Slice<Type, 1, N, M, N> row(size_t i)
			// 返回特定行的矩阵切片
		ConstSlice<Type, M, 1, M, N> col(size_t j) const
		Slice<Type, M, 1, M, N> col(size_t j)
			// 返回特定列的矩阵切片
		void setRow(size_t i, const Matrix<Type, N, 1> &row_in)	
		void setRow(size_t i, Type val)
			// 设置某行的所有数据
		void setCol(size_t j, const Matrix<Type, M, 1> &column)
		void setCol(size_t j, Type val)
			// 设置某列的所有数据
		void setZero()
		inline void zero()
			// 矩阵置0
		void setAll(Type val) // 矩阵设置特定元素值
		inline void setOne() 	// 所有元素设置为1
		inline void setNaN()	// 所有元素设置为NAN
		void setIdentity()   //将矩阵斜对角的所有元素置1(特征元素??)
		inline void identity()	
		inline void swapRows(size_t a, size_t b)
			// 置换a行和b行的元素
		inline void swapCols(size_t a, size_t b)
			// 置换a列和b列的元素
		Matrix<Type, M, N> abs() const
			// 对矩阵所有元素取绝对值
		Type max() const
		Type min() const
			// 去矩阵所有元素的最值
		bool isAllNan() const   // 判断所有元素是否全是NAN
		bool isAllFinite() const  // 判断所有元素是否全是FINITE
	/**
	 * class Matrix<Type, M, N>
	 */
	
	==== other:
		Matrix<Type, M, N> zeros()   //取0矩阵
		Matrix<Type, M, N> ones()  //取1矩阵
		Matrix<float, M, N> nans() //取NAN矩阵
		Matrix<Type, M, N> operator*(Type scalar, const Matrix<Type, M, N> &other)
			// 重载matrix域的*为矩阵元素乘法
		bool isEqual(const Matrix<Type, M, N> &x, const Matrix<Type, M, N> &y, const Type eps = Type(1e-4f))
			// 判断两个矩阵是否相等---> 判断所有元素是否相等
	==== typeFunction::
		Type min(const Type x, const Type y)
		Type max(const Type x, const Type y)
			// 取最值
		Type constrain(const Type x, const Type lower_bound, const Type upper_bound)
			// 变量限幅
		Matrix<Type, M, N> min(const Matrix<Type, M, N> &x, const Type scalar_upper_bound)
		Matrix<Type, M, N> min(const Type scalar_upper_bound, const Matrix<Type, M, N> &x)
		Matrix<Type, M, N> min(const Matrix<Type, M, N> &x1, const Matrix<Type, M, N> &x2)	
		Matrix<Type, M, N> max(const Matrix<Type, M, N> &x, const Type scalar_lower_bound)
		Matrix<Type, M, N> max(const Type scalar_lower_bound, const Matrix<Type, M, N> &x)
		Matrix<Type, M, N> max(const Matrix<Type, M, N> &x1, const Matrix<Type, M, N> &x2)
		Matrix<Type, M, N> constrain(const Matrix<Type, M, N> &x,
			     const Type scalar_lower_bound,
			     const Type scalar_upper_bound)
		Matrix<Type, M, N> constrain(const Matrix<Type, M, N> &x,
			     const Matrix<Type, M, N> &x_lower_bound,
			     const Matrix<Type, M, N> &x_upper_bound)
			// 用矩阵所有元素来判断来构造一个新的最值矩阵
		OStream &operator<<(OStream &os, const matrix::Matrix<Type, M, N> &matrix)
			// 将矩阵输出到指定的输出流
		
/**
 * @file Slice.hpp 矩阵切片
 */
	/**
	 * class SliceT<MatrixT, Type, P, Q, M, N>
	 */
	size_t _x0, _y0;
	MatrixT *_data;
	==== Constructor params:
		SliceT(size_t x0, size_t y0, MatrixT *data)
		SliceT(const Self &other)
	==== basical access function:
		const Type &operator()(size_t i, size_t j) const
		Type &operator()(size_t i, size_t j)
			// 以切分的x0, y0为矩阵起点访问 第i行, i列的元素
		Self &operator=(const Self &other)
		Self &operator=(const SliceT<Matrix<Type, MM, NN>, Type, P, Q, MM, NN> &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator=(const SliceT<const Matrix<Type, MM, NN>, Type, P, Q, MM, NN> &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator=(const Matrix<Type, P, Q> &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator=(const Type &other)
			// 切分矩阵赋值
		SliceT<MatrixT, Type, 1, Q, M, N> &operator=(const Vector<Type, Q> &other)
			// 向量赋值
		SliceT<MatrixT, Type, P, Q, M, N> &operator+=(const SliceT<MatrixT, Type, P, Q, MM, NN> &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator+=(const Matrix<Type, P, Q> &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator+=(const Type &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator-=(const SliceT<MatrixT, Type, P, Q, MM, NN> &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator-=(const Matrix<Type, P, Q> &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator-=(const Type &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator*=(const Type &other)
		SliceT<MatrixT, Type, P, Q, M, N> &operator/=(const Type &other)
		Matrix<Type, P, Q> operator*(const Type &other) const
		Matrix<Type, P, Q> operator/(const Type &other) const
			// 四则运算
		const SliceT<MatrixT, Type, R, S, M, N> slice(size_t x0, size_t y0) const
		SliceT<MatrixT, Type, R, S, M, N> slice(size_t x0, size_t y0)
			// 在类基础之上取第二次切分
		void copyTo(Type dst[P * Q]) const
		void copyToColumnMajor(Type dst[P * Q]) const
			// 矩阵复制
		Vector < Type, P < Q ? P : Q > diag() const
			// 取对角向量(identity)
		Type norm_squared() const
		Type norm()
			// 求矩阵的模/模的平方
		bool longerThan(Type testVal)
			// 将矩阵的模与标量对比
		Type max()	
		Type min()
			// 取矩阵元素里的最值
		
/**
 * @file Scalar.hpp 标量定义
 */		
	/**
	 * class Scalar<Type>
	 */	
	 const Type _value;
	 
	 Scalar(const Matrix<Type, 1, 1> &other)
	 Scalar(Type other)	
	 bool operator==(const float other)	
	 operator const Type &()		   //类型转换重载
	 operator Matrix<Type, 1, 1>()
	 operator Vector<Type, 1>()
```

#### 2.1.8 squareMatrix_SparseVector

```cpp
/**
 * @file SquareMatrix.hpp 方矩阵定义
 */	 
	/**
	 * class SquareMatrix<typename Type, size_t  M> : public Matrix<Type, M, M>
	 */	
	explicit SquareMatrix(const Type data_[M][M])
	explicit SquareMatrix(const Type data_[M * M])
	SquareMatrix(const Matrix<Type, M, M> &other)
		// 简单构造
	SquareMatrix<Type, M> &operator=(const Matrix<Type, M, M> &other)
	SquareMatrix<Type, M> &operator=(const Slice<Type, M, M, P, Q> &in_slice)
		// 定义方阵的operator=	
	ConstSlice<Type, P, Q, M, M> slice(size_t x0, size_t y0) const
	Slice<Type, P, Q, M, M> slice(size_t x0, size_t y0)
		// 定义方阵的slice切片
	inline SquareMatrix<Type, M> I() const
	inline bool I(SquareMatrix<Type, M> &i) const
		// 求方阵的逆矩阵
	Vector<Type, M> diag() const
		// 取出方阵的对角向量
	Vector < Type, M *(M + 1) / 2 > upper_right_triangle() const
		// 获取行主向量格式右上三角形
	Type trace(size_t first) const
	Type trace() const
		// 求矩阵的迹
	void uncorrelateCovarianceBlock(size_t first)
		// 保存为协方差矩阵, 并将与矩阵其余部分相关的所有协方差元素归零
	void uncorrelateCovariance(size_t first)
		// 将所有非对角线元素归零并保留相应的对角线元素
	void uncorrelateCovarianceSetVariance(size_t first, const Vector<Type, Width> &vec)
		// 用向量来保存为协方差矩阵
	void uncorrelateCovarianceSetVariance(size_t first, Type val)
		// 用变量来保存为协方差矩阵
	void makeBlockSymmetric(size_t first)
		// 通过取两个相应的非对角值的平均值使块对角对称
	void makeRowColSymmetric(size_t first)	
		// 通过取两个相应的非对角线值的平均值来使行和列对称
	bool isBlockSymmetric(size_t first, const Type eps = Type(1e-8f))
		// 检查块对角线是否对称
	bool isRowColSymmetric(size_t first, const Type eps = Type(1e-8f))
		// 检查行和列是否对称
	void copyLowerToUpperTriangle()
	void copyUpperToLowerTriangle()
		// 保存/复制上/下三角矩阵
	/**
	 * class SquareMatrix<typename Type, size_t  M> : public Matrix<Type, M, M>
	 */		
	SquareMatrix<Type, M> eye()
		// 返回单位矩阵
	SquareMatrix<Type, M> diag(Vector<Type, M> d)
		// 将向量赋值为仅有的对角矩阵
	SquareMatrix<Type, M> expm(const Matrix<Type, M, M> &A, size_t order = 5)
		// 矩阵指数运算 e^(A) = I + A + A^2/2! + A^3/3! + ....
	bool inv(const SquareMatrix<Type, 1> &A, SquareMatrix<Type, 1> &inv, size_t rank = 1)
		// 求逆矩阵, 一阶矩阵通用
	bool inv(const SquareMatrix<Type, M> &A, SquareMatrix<Type, M> &inv, size_t rank = M)
		// 求逆矩阵, 通用矩阵阶数, LU分解求逆矩阵
	bool inv(const SquareMatrix<Type, 2> &A, SquareMatrix<Type, 2> &inv)
		// 求逆矩阵, 二阶矩阵通用
	bool inv(const SquareMatrix<Type, 3> &A, SquareMatrix<Type, 3> &inv)
		// 求逆矩阵, 三阶矩阵通用
	SquareMatrix<Type, M> inv(const SquareMatrix<Type, M> &A)
		// 求逆矩阵, 通用接口, 如果求逆失败则返回0矩阵
	SquareMatrix <Type, M> cholesky(const SquareMatrix<Type, M> &A)
		// cholesky分解
	SquareMatrix <Type, M> choleskyInv(const SquareMatrix<Type, M> &A)
		// cholesky求逆
	
/**
 * @file SparseVector.hpp 稀疏向量定义
 */	 
	/**
	 * class SparseVector<typename Type, size_t M, size_t... Idxs>
	 */		 
	 static constexpr size_t N = sizeof...(Idxs);
	 static constexpr size_t _indices[N] {Idxs...};
		// 非零元素的索引数组indices
	 Type _data[N] {};		// 值数组
	 
	 static constexpr bool duplicateIndices()
		// 判断索引数组是否存在相同的值
	 static constexpr size_t findMaxIndex()
		// 查找索引数组的最值
	 static constexpr int findCompressedIndex(size_t index)
		// 寻找指定索引在索引数组中的索引
	 constexpr size_t non_zeros() const      // 返回索引个数
	 constexpr size_t index(size_t i) const  // 返回索引位置中指定位置的索引
	 SparseVector(const matrix::Vector<Type, M> &data)
	 explicit SparseVector(const Type data[N])
		// 使用向量/数组来构造
	 template <size_t i>
	 inline Type at() const
	 template <size_t i>
	 inline Type &at()
		// 根据索引数组的索引来查找数据
	 inline Type atCompressedIndex(size_t i) const
	 inline Type &atCompressedIndex(size_t i)
		// 直接查询数据
	 void setZero()   // 设置为0
	 Type dot(const matrix::Vector<Type, M> &other)  
		//向量点乘(元素相乘)(根据索引数组的索引)
	 matrix::Vector<Type, M> operator+(const matrix::Vector<Type, M> &other)
		// 向量相加(根据索引数组的索引)
	 SparseVector &operator+=(Type t)
	 Type norm_squared() const
		// 向量的模平方
	 Type norm() const
	    // 向量的模
	 bool longerThan(Type testVal)
		// 对比向量的模大小
	/**
	 * class SparseVector<typename Type, size_t M, size_t... Idxs>
	 */		 
template<typename Type, size_t Q, size_t M, size_t ... Idxs>
matrix::Vector<Type, Q> operator*(const matrix::Matrix<Type, Q, M> &mat,
				  const matrix::SparseVector<Type, M, Idxs...> &vec)
	 // 稀疏矩阵和矩阵相乘

	// x.T * A * x
template<typename Type, size_t M, size_t ... Idxs>
Type quadraticForm(const matrix::SquareMatrix<Type, M> &A, const matrix::SparseVector<Type, M, Idxs...> &x)
```

#### 2.1.9 vectorN

```cpp
/**
 * @file Vector.hpp 一维向量定义
 */	 
	/**
	 * class Vector<typename Type, size_t M> : public Matrix<Type, M, 1>
	 */
	 // 构造
	 Vector()
	 Vector(const MatrixM1 &other)
	 explicit Vector(const Type data_[M])
	 Vector(const Slice<Type, M, 1, P, Q> &slice_in)
	 Vector(const Slice<Type, 1, M, P, Q> &slice_in)
	 Vector(const ConstSlice<Type, M, 1, P, Q> &slice_in)
	 Vector(const ConstSlice<Type, 1, M, P, Q> &slice_in)
	 
	 inline const Type &operator()(size_t i)
	 inline Type &operator()(size_t i)
		// 返回索引为i的元素
	 Type dot(const MatrixM1 &b)
	 inline Type operator*(const MatrixM1 &b)
	 inline Vector operator*(Type b)
	 Type norm()
	 inline Type length()
		// 向量的点积(向量的模) (一维点积 == 模)
	 Type norm_squared()
		// 向量的模平方
	 inline void normalize()
	 inline Vector normalized() const
	 Vector unit()
	 Vector unit_or_zero(const Type eps = Type(1e-5)) const
		// 向量单位化
	 bool longerThan(Type testVal) const
		// 对比向量的模和值的大小
	 Vector sqrt()
		// 向量开方
	 void print() 
	 static size_t size() //向量长度
	 OStream &operator<<(OStream &os, const matrix::Vector<Type, M> &vector)

/**
 * @file Vector2.hpp 二维向量定义
 */	 
	/**
	 * class Vector2<typename Type> : public Vector<Type, 2>
	 */	 
	 Vector2() = default;
	 Vector2(const Matrix21 &other)
	 explicit Vector2(const Type data_[2])
	 Vector2(Type x, Type y)
	 explicit Vector2(const Vector3 &other)
	 
	 Type cross(const Matrix21 &b) const
	 Type operator%(const Matrix21 &b) const
		// 二维向量叉乘
	 
/**
 * @file Vector3.hpp 三维向量定义
 */	 
	/**
	 * class Vector3<typename Type> : public Vector<Type, 3>
	 */	 
	 Vector3() = default;
	 Vector3(const Matrix31 &other)
	 explicit Vector3(const Type data_[3])
	 Vector3(Type x, Type y, Type z)
	 
	 Vector3 cross(const Matrix31 &b) const
		// 三维向量叉乘
	 inline Vector3 operator+(Vector3 other) const
	 inline Vector3 operator+(Type scalar) const
	 inline Vector3 operator-(Vector3 other) const
	 inline Vector3 operator-(Type scalar) const
	 inline Vector3 operator-() const
	 inline Vector3 operator*(Type scalar) const
	 inline Type operator*(Vector3 b) const
	 inline Vector3 operator%(const Matrix31 &b) const
		// 三维向量运算重载
	 ConstSlice<Type, 2, 1, 3, 1> xy()
	 Slice<Type, 2, 1, 3, 1> xy()
		// 返回xy范围的切片
	 Dcm<Type> hat() const  
		// 返回方向矢量所代表的的方向余弦矩阵
	
/**
 * @file Vector4.hpp 四维向量定义
 */	 
	/**
	 * class Vector4<typename Type> : public Vector<Type, 4>
	 */	 	 
	 Vector4() = default;
	 explicit Vector4(const Type data_[3])
	 Vector4(Type x1, Type x2, Type x3, Type x4)
	 Vector4(const Slice<Type, 4, 1, P, Q> &slice_in) : Vector<Type, 4>(slice_in)
	 Vector4(const Slice<Type, 1, 4, P, Q> &slice_in) : Vector<Type, 4>(slice_in)
```

### 2.2 module/attitude_estimator_q

```cpp
/* 1.14.0
	=========================    attitude_estimator_q_main    =========================
	
	0: 坐标系方向: 右手系 x正方向为机头, y正方向为机头右侧
	1: 初始化 init_attitude_q:
		以
			重力加速度
		和
			其重力加速度与磁场矢量的施密特正交化的向量
		和
			两者的叉乘
		作为dcm的基来初始化q
	2: 相比于传统mahony 主要加入了 磁偏角的误差补偿 和 gps查分加速度的误差补偿	
	3: 在mag参与mahony 互补融合中, 只让其加入了偏航角的融合部分, 不对pitch 和 roll造成影响
	  加入了陀螺仪的偏差矫正->
	  迭代模式:
	  corr = last_gyro 
			+ p加速度计 * (姿态重力向量×(加速度计重力向量 - gps差分加速度向量).normalized)
			+ p磁力计 * (载体坐标系磁航向 - 载体坐标系磁偏角) * 角速度模增益
			+ (corr * 陀螺仪偏差增益 * dt)

	  _q += _q.derivative1(corr) * dt;
	  _q.normalize();
*/
```

### 2.3 module/local_position_estimator

#### 2.3.1 sensor/gps.cpp

```cpp
#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t		REQ_GPS_INIT_COUNT = 10;
static const uint32_t		GPS_TIMEOUT = 1000000;	// 1.0 s

void BlockLocalPositionEstimator::gpsInit()
{
	// check for good gps signal
	uint8_t nSat = _sub_gps.get().satellites_used;
	float eph = _sub_gps.get().eph;
	float epv = _sub_gps.get().epv;
	uint8_t fix_type = _sub_gps.get().fix_type;

	if (
		nSat < 6 ||
		eph > _param_lpe_eph_max.get() ||
		epv > _param_lpe_epv_max.get() ||
		fix_type < 3
	) {
		_gpsStats.reset();
		return;
	}

	// measure
	Vector<double, n_y_gps> y;

	if (gpsMeasure(y) != OK) {
		_gpsStats.reset();
		return;
	}

	// if finished
	if (_gpsStats.getCount() > REQ_GPS_INIT_COUNT) {
		// get mean gps values
		double gpsLat = _gpsStats.getMean()(0);
		double gpsLon = _gpsStats.getMean()(1);
		float gpsAlt = _gpsStats.getMean()(2);

		_sensorTimeout &= ~SENSOR_GPS;
		_sensorFault &= ~SENSOR_GPS;
		_gpsStats.reset();

		if (!_receivedGps) {
			// this is the first time we have received gps
			_receivedGps = true;

			// note we subtract X_z which is in down directon so it is
			// an addition
			_gpsAltOrigin = gpsAlt + _x(X_z);

			// find lat, lon of current origin by subtracting x and y
			// if not using vision position since vision will
			// have it's own origin, not necessarily where vehicle starts
			if (!_map_ref.isInitialized()) {
				double gpsLatOrigin = 0;
				double gpsLonOrigin = 0;
				// reproject at current coordinates
				_map_ref.initReference(gpsLat, gpsLon);
				// find origin
				_map_ref.reproject(-_x(X_x), -_x(X_y), gpsLatOrigin, gpsLonOrigin);
				// reinit origin
				_map_ref.initReference(gpsLatOrigin, gpsLonOrigin);
				// set timestamp when origin was set to current time
				_time_origin = _timeStamp;

				// always override alt origin on first GPS to fix
				// possible baro offset in global altitude at init
				_altOrigin = _gpsAltOrigin;
				_altOriginInitialized = true;
				_altOriginGlobal = true;

				mavlink_log_info(&mavlink_log_pub, "[lpe] global origin init (gps) : lat %6.2f lon %6.2f alt %5.1f m",
						 gpsLatOrigin, gpsLonOrigin, double(_gpsAltOrigin));
			}

			PX4_INFO("[lpe] gps init "
				 "lat %6.2f lon %6.2f alt %5.1f m",
				 gpsLat,
				 gpsLon,
				 double(gpsAlt));
		}
	}
}

int BlockLocalPositionEstimator::gpsMeasure(Vector<double, n_y_gps> &y)
{
	// gps measurement
	y.setZero();
	y(0) = _sub_gps.get().latitude_deg;
	y(1) = _sub_gps.get().longitude_deg;
	y(2) = _sub_gps.get().altitude_msl_m;
	y(3) = (double)_sub_gps.get().vel_n_m_s;
	y(4) = (double)_sub_gps.get().vel_e_m_s;
	y(5) = (double)_sub_gps.get().vel_d_m_s;

	// increament sums for mean
	_gpsStats.update(y);
	_time_last_gps = _timeStamp;
	return OK;
}

void BlockLocalPositionEstimator::gpsCorrect()
{
	// measure
	Vector<double, n_y_gps> y_global;

	// 将GPS数据赋值给全球位置y_global
	if (gpsMeasure(y_global) != OK) { return; }

	// 将全球经纬度转换成本地XYZ坐标
	// gps measurement in local frame
	double lat = y_global(Y_gps_x);
	double lon = y_global(Y_gps_y);
	float alt = y_global(Y_gps_z);
	float px = 0;
	float py = 0;
	float pz = -(alt - _gpsAltOrigin);
	_map_ref.project(lat, lon, px, py);
	
	// 赋值本地位置
	Vector<float, n_y_gps> y;
	y.setZero();
	y(Y_gps_x) = px;
	y(Y_gps_y) = py;
	y(Y_gps_z) = pz;
	y(Y_gps_vx) = y_global(Y_gps_vx);
	y(Y_gps_vy) = y_global(Y_gps_vy);
	y(Y_gps_vz) = y_global(Y_gps_vz);

	// GPS观测矩阵，和状态向量相关的值之间就是简单的一比一关系
	// gps measurement matrix, measures position and velocity
	Matrix<float, n_y_gps, n_x> C;
	C.setZero();
	C(Y_gps_x, X_x) = 1;
	C(Y_gps_y, X_y) = 1;
	C(Y_gps_z, X_z) = 1;
	C(Y_gps_vx, X_vx) = 1;
	C(Y_gps_vy, X_vy) = 1;
	C(Y_gps_vz, X_vz) = 1;

	// GPS协方差矩阵
	// gps covariance matrix
	SquareMatrix<float, n_y_gps> R;
	R.setZero();

	// 如果设置了GPS协方差参数，使用设置的参数
	// default to parameter, use gps cov if provided
	float var_xy = _param_lpe_gps_xy.get() * _param_lpe_gps_xy.get();
	float var_z = _param_lpe_gps_z.get() * _param_lpe_gps_z.get();
	float var_vxy = _param_lpe_gps_vxy.get() * _param_lpe_gps_vxy.get();
	float var_vz = _param_lpe_gps_vz.get() * _param_lpe_gps_vz.get();

	// 如果实际GPS的精度因子大于参数值，使用精度因子的平方作为协方差
	// if field is not below minimum, set it to the value provided
	if (_sub_gps.get().eph > _param_lpe_gps_xy.get()) {
		var_xy = _sub_gps.get().eph * _sub_gps.get().eph;
	}

	if (_sub_gps.get().epv > _param_lpe_gps_z.get()) {
		var_z = _sub_gps.get().epv * _sub_gps.get().epv;
	}

	float gps_s_stddev =  _sub_gps.get().s_variance_m_s;

	if (gps_s_stddev > _param_lpe_gps_vxy.get()) {
		var_vxy = gps_s_stddev * gps_s_stddev;
	}

	if (gps_s_stddev > _param_lpe_gps_vz.get()) {
		var_vz = gps_s_stddev * gps_s_stddev;
	}

	// 赋值协方差矩阵
	R(0, 0) = var_xy;
	R(1, 1) = var_xy;
	R(2, 2) = var_z;
	R(3, 3) = var_vxy;
	R(4, 4) = var_vxy;
	R(5, 5) = var_vz;

	// 计算上一次的观测值x0
	// get delayed x
	uint8_t i_hist = 0;

	if (getDelayPeriods(_param_lpe_gps_delay.get(), &i_hist)  < 0) { return; }

	Vector<float, n_x> x0 = _xDelay.get(i_hist);

	// 计算残差
	// residual
	Vector<float, n_y_gps> r = y - C * x0;

	// 计算残差协方差
	// residual covariance
	Matrix<float, n_y_gps, n_y_gps> S = C * m_P * C.transpose() + R;

	// 发布
	// publish innovations
	_pub_innov.get().gps_hpos[0] = r(0);
	_pub_innov.get().gps_hpos[1] = r(1);
	_pub_innov.get().gps_vpos    = r(2);
	_pub_innov.get().gps_hvel[0] = r(3);
	_pub_innov.get().gps_hvel[1] = r(4);
	_pub_innov.get().gps_vvel    = r(5);

	// publish innovation variances
	_pub_innov_var.get().gps_hpos[0] = S(0, 0);
	_pub_innov_var.get().gps_hpos[1] = S(1, 1);
	_pub_innov_var.get().gps_vpos    = S(2, 2);
	_pub_innov_var.get().gps_hvel[0] = S(3, 3);
	_pub_innov_var.get().gps_hvel[1] = S(4, 4);
	_pub_innov_var.get().gps_vvel    = S(5, 5);

	// 残差协方差求逆
	// residual covariance, (inverse)
	Matrix<float, n_y_gps, n_y_gps> S_I = inv<float, n_y_gps>(S);

	// 故障检测
	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	// artificially increase beta threshhold to prevent fault during landing
	float beta_thresh = 1e2f;

	if (beta / BETA_TABLE[n_y_gps] > beta_thresh) {
		if (!(_sensorFault & SENSOR_GPS)) {
			mavlink_log_critical(&mavlink_log_pub, "[lpe] gps fault %3g %3g %3g %3g %3g %3g",
					     double(r(0) * r(0) / S_I(0, 0)),  double(r(1) * r(1) / S_I(1, 1)), double(r(2) * r(2) / S_I(2, 2)),
					     double(r(3) * r(3) / S_I(3, 3)),  double(r(4) * r(4) / S_I(4, 4)), double(r(5) * r(5) / S_I(5, 5)));
			_sensorFault |= SENSOR_GPS;
		}

	} else if (_sensorFault & SENSOR_GPS) {
		_sensorFault &= ~SENSOR_GPS;
		mavlink_log_info(&mavlink_log_pub, "[lpe] GPS OK");
	}

	//卡尔曼滤波观测修正
	//计算卡尔曼增益
	// kalman filter correction always for GPS
	Matrix<float, n_x, n_y_gps> K = m_P * C.transpose() * S_I;
	// 计算修正量
	Vector<float, n_x> dx = K * r;
	//修正状态向量
	_x += dx;
	// 协方差矩阵更新
	m_P -= K * C * m_P;
}

void BlockLocalPositionEstimator::gpsCheckTimeout()
{
	if (_timeStamp - _time_last_gps > GPS_TIMEOUT) {
		if (!(_sensorTimeout & SENSOR_GPS)) {
			_sensorTimeout |= SENSOR_GPS;
			_gpsStats.reset();
			mavlink_log_critical(&mavlink_log_pub, "[lpe] GPS timeout ");
		}
	}
}
```

#### 2.3.2 BlockLocalPositionEstimator.cpp

```cpp
#include "BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>
#include <cstdlib>

orb_advert_t mavlink_log_pub = nullptr;

// required standard deviation of estimate for estimator to publish data
static const uint32_t		EST_STDDEV_XY_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_Z_VALID = 2.0;	// 2.0 m
static const uint32_t		EST_STDDEV_TZ_VALID = 2.0;	// 2.0 m

static const float P_MAX = 1.0e6f;	// max allowed value in state covariance
static const float LAND_RATE = 10.0f;	// rate of land detector correction

static const char *msg_label = "[lpe] ";	// rate of land detector correction

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::INS0),

	// this block has no parent, and has name LPE
	SuperBlock(nullptr, "LPE"),

	// flow gyro
	_flow_gyro_x_high_pass(this, "FGYRO_HP"),
	_flow_gyro_y_high_pass(this, "FGYRO_HP"),

	// stats
	_baroStats(this, ""),
	_sonarStats(this, ""),
	_lidarStats(this, ""),
	_flowQStats(this, ""),
	_visionStats(this, ""),
	_mocapStats(this, ""),
	_gpsStats(this, ""),

	// low pass
	_xLowPass(this, "X_LP"),

	// use same lp constant for agl
	_aglLowPass(this, "X_LP"),

	// delay
	_xDelay(this, ""),
	_tDelay(this, ""),

	// misc
	_timeStamp(hrt_absolute_time()),
	_time_origin(0),
	_timeStampLastBaro(hrt_absolute_time()),
	_time_last_hist(0),
	_time_last_flow(0),
	_time_last_baro(0),
	_time_last_gps(0),
	_time_last_lidar(0),
	_time_last_sonar(0),
	_time_init_sonar(0),
	_time_last_vision_p(0),
	_time_last_mocap(0),
	_time_last_land(0),
	_time_last_target(0),

	// reference altitudes
	_altOrigin(0),
	_altOriginInitialized(false),
	_altOriginGlobal(false),
	_baroAltOrigin(0),
	_gpsAltOrigin(0),

	// status
	_receivedGps(false),
	_lastArmedState(false),

	// masks
	_sensorTimeout(UINT16_MAX),
	_sensorFault(0),
	_estimatorInitialized(0),

	// sensor update flags
	_flowUpdated(false),
	_gpsUpdated(false),
	_visionUpdated(false),
	_mocapUpdated(false),
	_lidarUpdated(false),
	_sonarUpdated(false),
	_landUpdated(false),
	_baroUpdated(false),

	// sensor validation flags
	_vision_xy_valid(false),
	_vision_z_valid(false),
	_mocap_xy_valid(false),
	_mocap_z_valid(false),

	// sensor std deviations
	_vision_eph(0.0),
	_vision_epv(0.0),
	_mocap_eph(0.0),
	_mocap_epv(0.0),

	// local to global coversion related variables
	_is_global_cov_init(false),
	_ref_lat(0.0),
	_ref_lon(0.0),
	_ref_alt(0.0)
{
	_sensors_sub.set_interval_ms(10); // main prediction loop, 100 hz (lockstep requires to run at full rate)

	// assign distance subs to array
	_dist_subs[0] = &_sub_dist0;
	_dist_subs[1] = &_sub_dist1;
	_dist_subs[2] = &_sub_dist2;
	_dist_subs[3] = &_sub_dist3;

	// initialize A, B,  P, x, u
	_x.setZero();
	_u.setZero();
	initSS();

	// print fusion settings to console
	PX4_INFO("fuse gps: %d, flow: %d, vis_pos: %d, "
		 "landing_target: %d, land: %d, pub_agl_z: %d, flow_gyro: %d, "
		 "baro: %d\n",
		 (_param_lpe_fusion.get() & FUSE_GPS) != 0,
		 (_param_lpe_fusion.get() & FUSE_FLOW) != 0,
		 (_param_lpe_fusion.get() & FUSE_VIS_POS) != 0,
		 (_param_lpe_fusion.get() & FUSE_LAND_TARGET) != 0,
		 (_param_lpe_fusion.get() & FUSE_LAND) != 0,
		 (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) != 0,
		 (_param_lpe_fusion.get() & FUSE_FLOW_GYRO_COMP) != 0,
		 (_param_lpe_fusion.get() & FUSE_BARO) != 0);
}

bool
BlockLocalPositionEstimator::init()
{
	if (!_sensors_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::dynamics(
	float t,
	const Vector<float, BlockLocalPositionEstimator::n_x> &x,
	const Vector<float, BlockLocalPositionEstimator::n_u> &u)
{
	return m_A * x + m_B * u;
}

void BlockLocalPositionEstimator::Run()
{
	if (should_exit()) {
		_sensors_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// 判断是否需要通过command命令设置全球位置的初始位置。
	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.update(&vehicle_command)) {
			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN) {
				const double latitude = vehicle_command.param5;
				const double longitude = vehicle_command.param6;
				const float altitude = vehicle_command.param7;

				_global_local_proj_ref.initReference(latitude, longitude, vehicle_command.timestamp);
				_global_local_alt0 = altitude;

				PX4_INFO("New NED origin (LLA): %3.10f, %3.10f, %4.3f\n", latitude, longitude, static_cast<double>(altitude));
			}
		}
	}

	//imu没有更新则返回
	sensor_combined_s imu;

	if (!_sensors_sub.update(&imu)) {
		return;
	}

	//计算两次运行的时间间隔dt
	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// set dt for all child blocks
	setDt(dt);

	// 未解锁时，寻找是否有测距传感器
	// auto-detect connected rangefinders while not armed
	_sub_armed.update();
	bool armedState = _sub_armed.get().armed;

	//不同的测据传感器发布的话题都是distance_sensor，采用的是一个多重发布的方式，
	//因此这里订阅时，也是遍历四个这个话题的四个接口，如果没有找到雷达或者声呐发布的消息，则进行寻
	if (!armedState && (_sub_lidar == nullptr || _sub_sonar == nullptr)) {
		// detect distance sensors
		for (size_t i = 0; i < N_DIST_SUBS; i++) {
			auto *s = _dist_subs[i];

			if (s == _sub_lidar || s == _sub_sonar) { continue; }

			if (s->update()) {

				if (s->get().timestamp == 0) { continue; }

				if (s->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_LASER &&
				    s->get().orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING &&
				    _sub_lidar == nullptr) {
					_sub_lidar = s;
					mavlink_log_info(&mavlink_log_pub, "%sDownward-facing Lidar detected with ID %zu", msg_label, i);

				} else if (s->get().type == distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND &&
					   s->get().orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING &&
					   _sub_sonar == nullptr) {
					_sub_sonar = s;
					mavlink_log_info(&mavlink_log_pub, "%sDownward-facing Sonar detected with ID %zu", msg_label, i);
				}
			}
		}
	}

	// reset pos, vel, and terrain on arming

	// XXX this will be re-enabled for indoor use cases using a
	// selection param, but is really not helping outdoors
	// right now.

	// if (!_lastArmedState && armedState) {

	//      // we just armed, we are at origin on the ground
	//      _x(X_x) = 0;
	//      _x(X_y) = 0;
	//      // reset Z or not? _x(X_z) = 0;

	//      // we aren't moving, all velocities are zero
	//      _x(X_vx) = 0;
	//      _x(X_vy) = 0;
	//      _x(X_vz) = 0;

	//      // assume we are on the ground, so terrain alt is local alt
	//      _x(X_tz) = _x(X_z);

	//      // reset lowpass filter as well
	//      _xLowPass.setState(_x);
	//      _aglLowPass.setState(0);
	// }

	_lastArmedState = armedState;

	// see which updates are available
	const bool paramsUpdated = _parameter_update_sub.updated();
	_baroUpdated = false;

	if ((_param_lpe_fusion.get() & FUSE_BARO) && _sub_airdata.update()) {
		if (_sub_airdata.get().timestamp != _timeStampLastBaro) {
			_baroUpdated = true;
			_timeStampLastBaro = _sub_airdata.get().timestamp;
		}
	}
	// 更新传感器可用标志
	_flowUpdated = (_param_lpe_fusion.get() & FUSE_FLOW) && _sub_flow.update();
	_gpsUpdated = (_param_lpe_fusion.get() & FUSE_GPS) && _sub_gps.update();
	_visionUpdated = (_param_lpe_fusion.get() & FUSE_VIS_POS) && _sub_visual_odom.update();
	_mocapUpdated = _sub_mocap_odom.update();
	_lidarUpdated = (_sub_lidar != nullptr) && _sub_lidar->update();
	_sonarUpdated = (_sub_sonar != nullptr) && _sub_sonar->update();
	_landUpdated = landed() && ((_timeStamp - _time_last_land) > 1.0e6f / LAND_RATE);// throttle rate
	bool targetPositionUpdated = _sub_landing_target_pose.update();

	// get new data
	_sub_att.update();
	_sub_angular_velocity.update();

	// update parameters
	if (paramsUpdated) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		SuperBlock::updateParams();
		ModuleParams::updateParams();
		updateSSParams();
	}

	// 判断vx vy数据是否符合性能标准, 如何符合, 则使能标志 EST_XY
	// is xy valid?
	bool vxy_stddev_ok = false;

	if (math::max(m_P(X_vx, X_vx), m_P(X_vy, X_vy)) < _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get()) {
		vxy_stddev_ok = true;
	}

	if (_estimatorInitialized & EST_XY) {
		// if valid and gps has timed out, set to not valid
		if (!vxy_stddev_ok && (_sensorTimeout & SENSOR_GPS)) {
			_estimatorInitialized &= ~EST_XY;
		}

	} else {
		if (vxy_stddev_ok) {
			//只要有一个模块初始化完成，并且vx和vy有效，都说明初始化完成
			if (!(_sensorTimeout & SENSOR_GPS)
			    || !(_sensorTimeout & SENSOR_FLOW)
			    || !(_sensorTimeout & SENSOR_VISION)
			    || !(_sensorTimeout & SENSOR_MOCAP)
			    || !(_sensorTimeout & SENSOR_LAND)
			    || !(_sensorTimeout & SENSOR_LAND_TARGET)
			   ) {
				_estimatorInitialized |= EST_XY;
			}
		}
	}

	//同判断xy有效数据相同
	// is z valid? 
	bool z_stddev_ok = sqrtf(m_P(X_z, X_z)) < _param_lpe_z_pub.get();

	if (_estimatorInitialized & EST_Z) {
		// if valid and baro has timed out, set to not valid
		if (!z_stddev_ok && (_sensorTimeout & SENSOR_BARO)) {
			_estimatorInitialized &= ~EST_Z;
		}

	} else {
		if (z_stddev_ok) {
			_estimatorInitialized |= EST_Z;
		}
	}

	// is terrain valid?
	bool tz_stddev_ok = sqrtf(m_P(X_tz, X_tz)) < _param_lpe_z_pub.get();

	if (_estimatorInitialized & EST_TZ) {
		if (!tz_stddev_ok) {
			_estimatorInitialized &= ~EST_TZ;
		}

	} else {
		if (tz_stddev_ok) {
			_estimatorInitialized |= EST_TZ;
		}
	}

	// 判断传感器是否超时
	// check timeouts
	checkTimeouts();

	// 如果GPS位置没有初始化, 则从参数LPE_LAT和LPE_LON里面获取初始位置
	// if we have no lat, lon initialize projection to LPE_LAT, LPE_LON parameters
	if (!_map_ref.isInitialized() && (_estimatorInitialized & EST_XY) && _param_lpe_fake_origin.get()) {
		_map_ref.initReference(
			(double)_param_lpe_lat.get(),
			(double)_param_lpe_lon.get());

		// set timestamp when origin was set to current time
		_time_origin = _timeStamp;

		mavlink_log_info(&mavlink_log_pub, "[lpe] global origin init (parameter) : lat %6.2f lon %6.2f alt %5.1f m",
				 double(_param_lpe_lat.get()), double(_param_lpe_lon.get()), double(_altOrigin));
	}

	// reinitialize x if necessary
	bool reinit_x = false;

	// 判断状态向量_x有无异常值, is_finite
	for (size_t i = 0; i < n_x; i++) {
		// should we do a reinit
		// of sensors here?
		// don't want it to take too long
		if (!PX4_ISFINITE(_x(i))) {
			reinit_x = true;
			mavlink_log_info(&mavlink_log_pub, "%sreinit x, x(%zu) not finite", msg_label, i);
			break;
		}
	}

	if (reinit_x) {
		for (size_t i = 0; i < n_x; i++) {
			_x(i) = 0;
		}

		mavlink_log_info(&mavlink_log_pub, "%sreinit x", msg_label);
	}

	// 判断状态协方差矩阵P是否需要重置，如果有值超限或者对角线元素存在负数则重置
	// force P symmetry and reinitialize P if necessary
	bool reinit_P = false;

	for (size_t i = 0; i < n_x; i++) {
		for (size_t j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(m_P(i, j))) {
				mavlink_log_info(&mavlink_log_pub,
						 "%sreinit P (%zu, %zu) not finite", msg_label, i, j);
				reinit_P = true;
			}

			if (i == j) {
				// make sure diagonal elements are positive
				if (m_P(i, i) <= 0) {
					mavlink_log_info(&mavlink_log_pub,
							 "%sreinit P (%zu, %zu) negative", msg_label, i, j);
					reinit_P = true;
				}

			} else {
				// copy elememnt from upper triangle to force
				// symmetry
				m_P(j, i) = m_P(i, j);
			}

			if (reinit_P) { break; }
		}

		if (reinit_P) { break; }
	}

	if (reinit_P) {
		initP();
	}

	// 状态预测步
	// do prediction
	predict(imu);

	// 利用传感器的观测值进行修正步，如果传感器数据超时，则重新初始化传感器，否则进行修正
	// sensor corrections/ initializations
	if (_gpsUpdated) {
		if (_sensorTimeout & SENSOR_GPS) {
			gpsInit();

		} else {
			gpsCorrect();
		}
	}

	if (_baroUpdated) {
		if (_sensorTimeout & SENSOR_BARO) {
			baroInit();

		} else {
			baroCorrect();
		}
	}

	if (_lidarUpdated) {
		if (_sensorTimeout & SENSOR_LIDAR) {
			lidarInit();

		} else {
			lidarCorrect();
		}
	}

	if (_sonarUpdated) {
		if (_sensorTimeout & SENSOR_SONAR) {
			sonarInit();

		} else {
			sonarCorrect();
		}
	}

	if (_flowUpdated) {
		if (_sensorTimeout & SENSOR_FLOW) {
			flowInit();

		} else {
			flowCorrect();
		}
	}

	if (_visionUpdated) {
		if (_sensorTimeout & SENSOR_VISION) {
			visionInit();

		} else {
			visionCorrect();
		}
	}

	if (_mocapUpdated) {
		if (_sensorTimeout & SENSOR_MOCAP) {
			mocapInit();

		} else {
			mocapCorrect();
		}
	}

	if (_landUpdated) {
		if (_sensorTimeout & SENSOR_LAND) {
			landInit();

		} else {
			landCorrect();
		}
	}

	if (targetPositionUpdated) {
		if (_sensorTimeout & SENSOR_LAND_TARGET) {
			landingTargetInit();

		} else {
			landingTargetCorrect();
		}
	}

	// 发布本地位置、估计器状态、协方差、状态标志等数据
	if (_altOriginInitialized) {
		// update all publications if possible
		publishLocalPos();
		publishOdom();
		publishEstimatorStatus();

		_pub_innov.get().timestamp_sample = _timeStamp;
		_pub_innov.get().timestamp = hrt_absolute_time();
		_pub_innov.update();

		_pub_innov_var.get().timestamp_sample = _timeStamp;
		_pub_innov_var.get().timestamp = hrt_absolute_time();
		_pub_innov_var.update();

		if ((_estimatorInitialized & EST_XY) && (_map_ref.isInitialized() || _param_lpe_fake_origin.get())) {
			publishGlobalPos();
		}
	}

	// 状态传播延时更新
	// propagate delayed state, no matter what
	// if state is frozen, delayed state still
	// needs to be propagated with frozen state
	float dt_hist = 1.0e-6f * (_timeStamp - _time_last_hist);

	if (_time_last_hist == 0 ||
	    (dt_hist > HIST_STEP)) {
		_tDelay.update(Scalar<uint64_t>(_timeStamp));
		_xDelay.update(_x);
		_time_last_hist = _timeStamp;
	}
}

void BlockLocalPositionEstimator::checkTimeouts()
{
	baroCheckTimeout();
	gpsCheckTimeout();
	lidarCheckTimeout();
	flowCheckTimeout();
	sonarCheckTimeout();
	visionCheckTimeout();
	mocapCheckTimeout();
	landCheckTimeout();
	landingTargetCheckTimeout();
}

bool BlockLocalPositionEstimator::landed()
{
	if (!(_param_lpe_fusion.get() & FUSE_LAND)) {
		return false;
	}

	_sub_land.update();

	bool disarmed_not_falling = (!_sub_armed.get().armed) && (!_sub_land.get().freefall);

	return _sub_land.get().landed || disarmed_not_falling;
}

void BlockLocalPositionEstimator::publishLocalPos()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(m_P(X_vx, X_vx) + m_P(X_vy, X_vy));
	float evv = sqrtf(m_P(X_vz, X_vz));
	float eph = sqrtf(m_P(X_x, X_x) + m_P(X_y, X_y));
	float epv = sqrtf(m_P(X_z, X_z));

	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (evh < _param_lpe_vxy_pub.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	// publish local position
	if (Vector3f(_x(X_x), _x(X_y), _x(X_z)).isAllFinite() &&
	    Vector3f(_x(X_vx), _x(X_vy), _x(X_vz)).isAllFinite()) {
		_pub_lpos.get().timestamp_sample = _timeStamp;

		_pub_lpos.get().xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().z_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().v_xy_valid = _estimatorInitialized & EST_XY;
		_pub_lpos.get().v_z_valid = _estimatorInitialized & EST_Z;

		_pub_lpos.get().x = xLP(X_x);	// north
		_pub_lpos.get().y = xLP(X_y);	// east

		if (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) {
			_pub_lpos.get().z = -_aglLowPass.getState();	// agl

		} else {
			_pub_lpos.get().z = xLP(X_z);	// down
		}

		const float heading = matrix::Eulerf(matrix::Quatf(_sub_att.get().q)).psi();
		_pub_lpos.get().heading = heading;
		_pub_lpos.get().heading_good_for_control = PX4_ISFINITE(heading);
		_pub_lpos.get().unaided_heading = NAN;

		_pub_lpos.get().vx = xLP(X_vx);		// north
		_pub_lpos.get().vy = xLP(X_vy);		// east
		_pub_lpos.get().vz = xLP(X_vz);		// down

		// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
		_pub_lpos.get().z_deriv = xLP(X_vz);

		_pub_lpos.get().ax = _u(U_ax);		// north
		_pub_lpos.get().ay = _u(U_ay);		// east
		_pub_lpos.get().az = _u(U_az);		// down

		_pub_lpos.get().xy_global = _estimatorInitialized & EST_XY;
		_pub_lpos.get().z_global = !(_sensorTimeout & SENSOR_BARO) && _altOriginGlobal;
		_pub_lpos.get().ref_timestamp = _time_origin;
		_pub_lpos.get().ref_lat = _map_ref.getProjectionReferenceLat();
		_pub_lpos.get().ref_lon = _map_ref.getProjectionReferenceLon();
		_pub_lpos.get().ref_alt = _altOrigin;
		_pub_lpos.get().dist_bottom = _aglLowPass.getState();
		// we estimate agl even when we don't have terrain info
		// if you are in terrain following mode this is important
		// so that if terrain estimation fails there isn't a
		// sudden altitude jump
		_pub_lpos.get().dist_bottom_valid = _estimatorInitialized & EST_Z;
		_pub_lpos.get().eph = eph;
		_pub_lpos.get().epv = epv;
		_pub_lpos.get().evh = evh;
		_pub_lpos.get().evv = evv;
		_pub_lpos.get().vxy_max = INFINITY;
		_pub_lpos.get().vz_max = INFINITY;
		_pub_lpos.get().hagl_min = INFINITY;
		_pub_lpos.get().hagl_max = INFINITY;
		_pub_lpos.get().timestamp = hrt_absolute_time();;
		_pub_lpos.update();
	}
}

void BlockLocalPositionEstimator::publishOdom()
{
	const Vector<float, n_x> &xLP = _xLowPass.getState();

	// publish vehicle odometry
	if (Vector3f(_x(X_x), _x(X_y), _x(X_z)).isAllFinite() &&
	    Vector3f(_x(X_vx), _x(X_vy), _x(X_vz)).isAllFinite()) {
		_pub_odom.get().timestamp_sample = _timeStamp;
		_pub_odom.get().pose_frame = vehicle_odometry_s::POSE_FRAME_NED;

		// position
		_pub_odom.get().position[0] = xLP(X_x);	// north
		_pub_odom.get().position[1] = xLP(X_y);	// east

		if (_param_lpe_fusion.get() & FUSE_PUB_AGL_Z) {
			_pub_odom.get().position[2] = -_aglLowPass.getState();	// agl

		} else {
			_pub_odom.get().position[2] = xLP(X_z);	// down
		}

		// orientation
		matrix::Quatf q = matrix::Quatf(_sub_att.get().q);
		q.copyTo(_pub_odom.get().q);

		// linear velocity
		_pub_odom.get().velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
		_pub_odom.get().velocity[0] = xLP(X_vx);		// vel north
		_pub_odom.get().velocity[1] = xLP(X_vy);		// vel east
		_pub_odom.get().velocity[2] = xLP(X_vz);		// vel down

		// angular velocity
		_pub_odom.get().angular_velocity[0] = NAN;
		_pub_odom.get().angular_velocity[1] = NAN;
		_pub_odom.get().angular_velocity[2] = NAN;

		// get the covariance matrix size
		const size_t POS_URT_SIZE = sizeof(_pub_odom.get().position_variance) / sizeof(_pub_odom.get().position_variance[0]);
		const size_t VEL_URT_SIZE = sizeof(_pub_odom.get().velocity_variance) / sizeof(_pub_odom.get().velocity_variance[0]);

		// initially set pose covariances to 0
		for (size_t i = 0; i < POS_URT_SIZE; i++) {
			_pub_odom.get().position_variance[i] = NAN;
		}

		// set the position variances
		_pub_odom.get().position_variance[0] = m_P(X_vx, X_vx);
		_pub_odom.get().position_variance[1] = m_P(X_vy, X_vy);
		_pub_odom.get().position_variance[2] = m_P(X_vz, X_vz);

		// unknown orientation covariances
		// TODO: add orientation covariance to vehicle_attitude
		_pub_odom.get().orientation_variance[0] = NAN;
		_pub_odom.get().orientation_variance[1] = NAN;
		_pub_odom.get().orientation_variance[2] = NAN;

		// initially set velocity covariances to 0
		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			_pub_odom.get().velocity_variance[i] = NAN;
		}

		// set the linear velocity variances
		_pub_odom.get().velocity_variance[0] = m_P(X_vx, X_vx);
		_pub_odom.get().velocity_variance[1] = m_P(X_vy, X_vy);
		_pub_odom.get().velocity_variance[2] = m_P(X_vz, X_vz);

		_pub_odom.get().timestamp = hrt_absolute_time();
		_pub_odom.update();
	}
}

void BlockLocalPositionEstimator::publishEstimatorStatus()
{
	_pub_est_states.get().timestamp_sample = _timeStamp;

	for (size_t i = 0; i < n_x; i++) {
		_pub_est_states.get().states[i] = _x(i);
	}

	// matching EKF2 covariances indexing
	// quaternion - not determined, as it is a position estimator
	_pub_est_states.get().covariances[0] = NAN;
	_pub_est_states.get().covariances[1] = NAN;
	_pub_est_states.get().covariances[2] = NAN;
	_pub_est_states.get().covariances[3] = NAN;
	// linear velocity
	_pub_est_states.get().covariances[4] = m_P(X_vx, X_vx);
	_pub_est_states.get().covariances[5] = m_P(X_vy, X_vy);
	_pub_est_states.get().covariances[6] = m_P(X_vz, X_vz);
	// position
	_pub_est_states.get().covariances[7] = m_P(X_x, X_x);
	_pub_est_states.get().covariances[8] = m_P(X_y, X_y);
	_pub_est_states.get().covariances[9] = m_P(X_z, X_z);
	// gyro bias - not determined
	_pub_est_states.get().covariances[10] = NAN;
	_pub_est_states.get().covariances[11] = NAN;
	_pub_est_states.get().covariances[12] = NAN;
	// accel bias
	_pub_est_states.get().covariances[13] = m_P(X_bx, X_bx);
	_pub_est_states.get().covariances[14] = m_P(X_by, X_by);
	_pub_est_states.get().covariances[15] = m_P(X_bz, X_bz);

	// mag - not determined
	for (size_t i = 16; i <= 21; i++) {
		_pub_est_states.get().covariances[i] = NAN;
	}

	// replacing the hor wind cov with terrain altitude covariance
	_pub_est_states.get().covariances[22] = m_P(X_tz, X_tz);

	_pub_est_states.get().n_states = n_x;
	_pub_est_states.get().timestamp = hrt_absolute_time();
	_pub_est_states.update();

	// estimator_status
	_pub_est_status.get().timestamp_sample = _timeStamp;
	_pub_est_status.get().health_flags = _sensorFault;
	_pub_est_status.get().timeout_flags = _sensorTimeout;
	_pub_est_status.get().pos_horiz_accuracy = _pub_gpos.get().eph;
	_pub_est_status.get().pos_vert_accuracy = _pub_gpos.get().epv;

	_pub_est_status.get().timestamp = hrt_absolute_time();
	_pub_est_status.update();
}

void BlockLocalPositionEstimator::publishGlobalPos()
{
	// publish global position
	double lat = 0;
	double lon = 0;
	const Vector<float, n_x> &xLP = _xLowPass.getState();
	_map_ref.reproject(xLP(X_x), xLP(X_y), lat, lon);
	float alt = -xLP(X_z) + _altOrigin;

	// lie about eph/epv to allow visual odometry only navigation when velocity est. good
	float evh = sqrtf(m_P(X_vx, X_vx) + m_P(X_vy, X_vy));
	float eph = sqrtf(m_P(X_x, X_x) + m_P(X_y, X_y));
	float epv = sqrtf(m_P(X_z, X_z));

	float eph_thresh = 3.0f;
	float epv_thresh = 3.0f;

	if (evh < _param_lpe_vxy_pub.get()) {
		if (eph > eph_thresh) {
			eph = eph_thresh;
		}

		if (epv > epv_thresh) {
			epv = epv_thresh;
		}
	}

	if (PX4_ISFINITE(lat) && PX4_ISFINITE(lon) && PX4_ISFINITE(alt) &&
	    Vector3f(xLP(X_vx), xLP(X_vy), xLP(X_vz)).isAllFinite()) {
		_pub_gpos.get().timestamp_sample = _timeStamp;
		_pub_gpos.get().lat = lat;
		_pub_gpos.get().lon = lon;
		_pub_gpos.get().alt = alt;
		_pub_gpos.get().eph = eph;
		_pub_gpos.get().epv = epv;
		_pub_gpos.get().terrain_alt = _altOrigin - xLP(X_tz);
		_pub_gpos.get().terrain_alt_valid = _estimatorInitialized & EST_TZ;
		_pub_gpos.get().dead_reckoning = !(_estimatorInitialized & EST_XY);
		_pub_gpos.get().timestamp = hrt_absolute_time();
		_pub_gpos.update();
	}
}

void BlockLocalPositionEstimator::initP()
{
	// 根据参数初始化协方差矩阵P
	m_P.setZero();
	// initialize to twice valid condition
	m_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	m_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	m_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	m_P(X_vx, X_vx) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	m_P(X_vy, X_vy) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	// use vxy thresh for vz init as well
	m_P(X_vz, X_vz) = 2 * _param_lpe_vxy_pub.get() * _param_lpe_vxy_pub.get();
	// initialize bias uncertainty to small values to keep them stable
	m_P(X_bx, X_bx) = 1e-6;
	m_P(X_by, X_by) = 1e-6;
	m_P(X_bz, X_bz) = 1e-6;
	m_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
}

void BlockLocalPositionEstimator::initSS()
{
	/* 初始化A、B、Q、R */
	initP();

	// dynamics matrix
	m_A.setZero();
	// 这里只设置了A的其中三个，其余的在下面的updateSSStates()函数中
	// derivative of position is velocity
	m_A(X_x, X_vx) = 1;
	m_A(X_y, X_vy) = 1;
	m_A(X_z, X_vz) = 1;

	// input matrix
	m_B.setZero();
	m_B(X_vx, U_ax) = 1;
	m_B(X_vy, U_ay) = 1;
	m_B(X_vz, U_az) = 1;

	// update components that depend on current state
	updateSSStates();
	updateSSParams();
}

void BlockLocalPositionEstimator::updateSSStates()
{
	// 速度的微分就是加速度计的加速度信息减去偏差
	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	m_A(X_vx, X_bx) = -_R_att(0, 0);
	m_A(X_vx, X_by) = -_R_att(0, 1);
	m_A(X_vx, X_bz) = -_R_att(0, 2);

	m_A(X_vy, X_bx) = -_R_att(1, 0);
	m_A(X_vy, X_by) = -_R_att(1, 1);
	m_A(X_vy, X_bz) = -_R_att(1, 2);

	m_A(X_vz, X_bx) = -_R_att(2, 0);
	m_A(X_vz, X_by) = -_R_att(2, 1);
	m_A(X_vz, X_bz) = -_R_att(2, 2);
}

void BlockLocalPositionEstimator::updateSSParams()
{
	// 更新R矩阵
	// input noise covariance matrix
	m_R.setZero();
	m_R(U_ax, U_ax) = _param_lpe_acc_xy.get() * _param_lpe_acc_xy.get();
	m_R(U_ay, U_ay) = _param_lpe_acc_xy.get() * _param_lpe_acc_xy.get();
	m_R(U_az, U_az) = _param_lpe_acc_z.get() * _param_lpe_acc_z.get();

	// 更新Q矩阵
	// process noise power matrix
	m_Q.setZero();
	float pn_p_sq = _param_lpe_pn_p.get() * _param_lpe_pn_p.get();
	float pn_v_sq = _param_lpe_pn_v.get() * _param_lpe_pn_v.get();
	m_Q(X_x, X_x) = pn_p_sq;
	m_Q(X_y, X_y) = pn_p_sq;
	m_Q(X_z, X_z) = pn_p_sq;
	m_Q(X_vx, X_vx) = pn_v_sq;
	m_Q(X_vy, X_vy) = pn_v_sq;
	m_Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	float pn_b_sq = _param_lpe_pn_b.get() * _param_lpe_pn_b.get();
	m_Q(X_bx, X_bx) = pn_b_sq;
	m_Q(X_by, X_by) = pn_b_sq;
	m_Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	float pn_t_noise_density =
		_param_lpe_pn_t.get() +
		(_param_lpe_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
	m_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
}

void BlockLocalPositionEstimator::predict(const sensor_combined_s &imu)
{
	// 将姿态四元数转换为旋转矩阵
	// get acceleration
	_R_att = matrix::Dcm<float>(matrix::Quatf(_sub_att.get().q));
	// 获取加速度
	Vector3f a(imu.accelerometer_m_s2);
	// 将加速度数据从机体坐标系转换到地理坐标系
	// 无人机采用的是NED坐标系，Z轴正方向是指向地面的，当无人机静止时，
	// 加速度值是负的重力加速度，所以需要加上重力加速度抵消重力加速度的影响，得到运动加速度
	// note, bias is removed in dynamics function
	_u = _R_att * a;
	_u(U_az) += CONSTANTS_ONE_G;	// add g

	// 更新系统状态空间转移矩阵，即A矩阵
	// update state space based on new states
	updateSSStates();

	// 连续时间卡尔曼滤波，利用四阶龙格库塔进行状态预测
	// 状态方程如下 (m_A * _x + m_B * _u)
	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = getDt();
	Vector<float, n_x> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	// 根据数据有效位判断 判断是否进行特定数据的融合
	// don't integrate position if no valid xy data
	if (!(_estimatorInitialized & EST_XY))  {
		dx(X_x) = 0;
		dx(X_vx) = 0;
		dx(X_y) = 0;
		dx(X_vy) = 0;
	}

	// don't integrate z if no valid z data
	if (!(_estimatorInitialized & EST_Z))  {
		dx(X_z) = 0;
	}

	// don't integrate tz if no valid tz data
	if (!(_estimatorInitialized & EST_TZ))  {
		dx(X_tz) = 0;
	}

	// 漂移量抗饱和, 先计算出bx、by、bz，然后判断如果大于BIAS_MAX，
	// 则将dx中的bx、by、bz进行限幅，bx = BIAS_MAX * bx / std::abs(bx)求出的是BIAS_MAX，符号和bx 相同，
	// 将 bx - _x(X_bx)赋值为偏移量，这样在后面进行状态更新时再加上_x(X_bx)，更新后的偏移量即为最大偏移量BIAS_MAX
	// saturate bias
	float bx = dx(X_bx) + _x(X_bx);
	float by = dx(X_by) + _x(X_by);
	float bz = dx(X_bz) + _x(X_bz);

	if (std::abs(bx) > BIAS_MAX) {
		bx = BIAS_MAX * bx / std::abs(bx);
		dx(X_bx) = bx - _x(X_bx);
	}

	if (std::abs(by) > BIAS_MAX) {
		by = BIAS_MAX * by / std::abs(by);
		dx(X_by) = by - _x(X_by);
	}

	if (std::abs(bz) > BIAS_MAX) {
		bz = BIAS_MAX * bz / std::abs(bz);
		dx(X_bz) = bz - _x(X_bz);
	}

	// 更新状态向量预测值
	// propagate
	_x += dx;
	
	// 下面是p的一阶微分方程，在解这个方程的时候，用的是最简单的欧拉法
	// 根据卡尔曼滤波公式预测协方差矩阵m_P
	Matrix<float, n_x, n_x> dP = (m_A * m_P + m_P * m_A.transpose() +
				      m_B * m_R * m_B.transpose() + m_Q) * getDt();

	// 如果本身的p已经大于了P_MAX，则不会再累加dp
	// covariance propagation logic
	for (size_t i = 0; i < n_x; i++) {
		if (m_P(i, i) > P_MAX) {
			// if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for (size_t j = 0; j < n_x; j++) {
				dP(i, j) = 0;
				dP(j, i) = 0;
			}
		}
	}

	m_P += dP;
	
	//低通滤波
	_xLowPass.update(_x);
	_aglLowPass.update(agl());
}

int BlockLocalPositionEstimator::getDelayPeriods(float delay, uint8_t *periods)
{
	float t_delay = 0;
	uint8_t i_hist = 0;

	for (i_hist = 1; i_hist < HIST_LEN; i_hist++) {
		t_delay = 1.0e-6f * (_timeStamp - _tDelay.get(i_hist)(0, 0));

		if (t_delay > delay) {
			break;
		}
	}

	*periods = i_hist;

	if (t_delay > DELAY_MAX) {
		mavlink_log_info(&mavlink_log_pub, "%sdelayed data old: %8.4f", msg_label, double(t_delay));
		return -1;
	}

	return OK;
}

int
BlockLocalPositionEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
BlockLocalPositionEstimator::task_spawn(int argc, char *argv[])
{
	BlockLocalPositionEstimator *instance = new BlockLocalPositionEstimator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
BlockLocalPositionEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude and position estimator using an Extended Kalman Filter.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("local_position_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int local_position_estimator_main(int argc, char *argv[])
{
	return BlockLocalPositionEstimator::main(argc, argv);
}
```

#### 2.3.3 BlockLocalPositionEstimator.hpp

```cpp
#pragma once

// 解析: https://www.freesion.com/article/5586340498/
// 移植: https://blog.csdn.net/iamqianrenzhan/article/details/74088650
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <lib/controllib/blocks.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/Matrix.hpp>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_odometry.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_innovations.h>

using namespace matrix;
using namespace control;
using namespace time_literals;

static const float DELAY_MAX = 0.5f;	// seconds
static const float HIST_STEP = 0.05f;	// 20 hz
static const float BIAS_MAX = 1e-1f;
static const size_t HIST_LEN = 10;	// DELAY_MAX / HIST_STEP;
static const size_t N_DIST_SUBS = 4;

// 统计学中的卡方检验：卡方检验就是统计样本的实际观测值与理论推断值之间的偏离程度，
// 实际观测值与理论推断值之间的偏离程度就决定卡方值的大小，卡方值越大，越不符合；卡方值越小，偏差越小，越趋于符合，
// 若两个值完全相等时，卡方值就为0，表明理论值完全符合
// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
static const float BETA_TABLE[7] = {0,
				    8.82050518214,
				    12.094592431,
				    13.9876612368,
				    16.0875642296,
				    17.8797700658,
				    19.6465647819,
				   };

class BlockLocalPositionEstimator : public ModuleBase<BlockLocalPositionEstimator>, public ModuleParams,
	public px4::WorkItem, public control::SuperBlock
{
// dynamics:
//
//	x(+) = A * x(-) + B * u(+) 系统状态方程
//	y_i = C_i*x 观测方程
//
// kalman filter
//
//	E[xx'] = P 估计量的误差的协方差
//	E[uu'] = W 系统噪声
//	E[y_iy_i'] = R_i 系统噪声协方差矩阵
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
//      ax, ay, az (acceleration NED)
//
// states:
//      px, py, pz , ( position NED, m)
//      vx, vy, vz ( vel NED, m/s),
//      bx, by, bz ( accel bias, m/s^2)
//      tz (terrain altitude, ASL, m)
//
// measurements:
//
//      sonar: pz (measured d*cos(phi)*cos(theta))
//
//      baro: pz
//
//      flow: vx, vy (flow is in body x, y frame)
//
//      gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
//      lidar: pz (actual measured d*cos(phi)*cos(theta))
//
//      vision: px, py, pz, vx, vy, vz
//
//      mocap: px, py, pz
//
//      land (detects when landed)): pz (always measures agl = 0)
//
public:

	BlockLocalPositionEstimator();
	~BlockLocalPositionEstimator() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	// constants
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_vx = 0, Y_flow_vy, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {Y_land_vx = 0, Y_land_vy, Y_land_agl, n_y_land};
	enum {Y_target_x = 0, Y_target_y, n_y_target};
	enum {
		FUSE_GPS = 1 << 0,
		FUSE_FLOW = 1 << 1,
		FUSE_VIS_POS = 1 << 2,
		FUSE_LAND_TARGET = 1 << 3,
		FUSE_LAND = 1 << 4,
		FUSE_PUB_AGL_Z = 1 << 5,
		FUSE_FLOW_GYRO_COMP = 1 << 6,
		FUSE_BARO = 1 << 7
	};

	enum sensor_t {
		SENSOR_BARO = 1 << 0,
		SENSOR_GPS = 1 << 1,
		SENSOR_LIDAR = 1 << 2,
		SENSOR_FLOW = 1 << 3,
		SENSOR_SONAR = 1 << 4,
		SENSOR_VISION = 1 << 5,
		SENSOR_MOCAP = 1 << 6,
		SENSOR_LAND = 1 << 7,
		SENSOR_LAND_TARGET = 1 << 8,
	};

	enum estimate_t {
		EST_XY = 1 << 0,
		EST_Z = 1 << 1,
		EST_TZ = 1 << 2,
	};

	void Run() override;

	// methods
	// ----------------------------
	//
	//动态方程，形式为：x‘ = _A * x + _B * u，这是一个一阶微分方程，也就是
	//描述系统状态空间的状态方程，属于现代控制理论中的。
	//区分kf中的 x_(k) = A*x(k-1) + B*u_(k-1)。
	//本程序中的状态估计用的是这个一阶微分方程结合龙哥库塔，而不是用的kf中的第一个方程。因为
	//这是一个连续系统
	Vector<float, n_x> dynamics(
		float t,
		const Vector<float, n_x> &x,
		const Vector<float, n_u> &u);
	void initP();			//初始化状态协方差矩阵P
	void initSS();			//这个函数包括了下面的两个函数，执行这个函数的同时也就执行了下面的两个
	void updateSSStates();		//设置A
	void updateSSParams();		//设置R、Q

	// predict the next state
	void predict(const sensor_combined_s &imu);

	// lidar
	int  lidarMeasure(Vector<float, n_y_lidar> &y);
	void lidarCorrect();
	void lidarInit();
	void lidarCheckTimeout();

	// sonar
	int  sonarMeasure(Vector<float, n_y_sonar> &y);
	void sonarCorrect();
	void sonarInit();
	void sonarCheckTimeout();

	// baro
	int  baroMeasure(Vector<float, n_y_baro> &y);
	void baroCorrect();
	void baroInit();
	void baroCheckTimeout();

	// gps
	int  gpsMeasure(Vector<double, n_y_gps> &y);
	void gpsCorrect();
	void gpsInit();
	void gpsCheckTimeout();

	// flow
	int  flowMeasure(Vector<float, n_y_flow> &y);
	void flowCorrect();
	void flowInit();
	void flowCheckTimeout();

	// vision
	int  visionMeasure(Vector<float, n_y_vision> &y);
	void visionCorrect();
	void visionInit();
	void visionCheckTimeout();

	// mocap
	int  mocapMeasure(Vector<float, n_y_mocap> &y);
	void mocapCorrect();
	void mocapInit();
	void mocapCheckTimeout();

	// land
	int  landMeasure(Vector<float, n_y_land> &y);
	void landCorrect();
	void landInit();
	void landCheckTimeout();

	// landing target
	int  landingTargetMeasure(Vector<float, n_y_target> &y);
	void landingTargetCorrect();
	void landingTargetInit();
	void landingTargetCheckTimeout();

	// timeouts
	void checkTimeouts();

	// misc
	inline float agl()
	{
		return _x(X_tz) - _x(X_z);
	}
	bool landed();
	int getDelayPeriods(float delay, uint8_t *periods);

	// publications
	void publishLocalPos();
	void publishGlobalPos();
	void publishOdom();
	void publishEstimatorStatus();
	void publishEk2fTimestamps();

	// attributes
	// ----------------------------

	// subscriptions
	uORB::SubscriptionCallbackWorkItem _sensors_sub{this, ORB_ID(sensor_combined)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::SubscriptionData<actuator_armed_s> _sub_armed{ORB_ID(actuator_armed)};
	uORB::SubscriptionData<vehicle_land_detected_s> _sub_land{ORB_ID(vehicle_land_detected)};
	uORB::SubscriptionData<vehicle_attitude_s> _sub_att{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<vehicle_angular_velocity_s> _sub_angular_velocity{ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionData<vehicle_optical_flow_s> _sub_flow{ORB_ID(vehicle_optical_flow)};
	uORB::SubscriptionData<sensor_gps_s> _sub_gps{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionData<vehicle_odometry_s> _sub_visual_odom{ORB_ID(vehicle_visual_odometry)};
	uORB::SubscriptionData<vehicle_odometry_s> _sub_mocap_odom{ORB_ID(vehicle_mocap_odometry)};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist0{ORB_ID(distance_sensor), 0};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist1{ORB_ID(distance_sensor), 1};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist2{ORB_ID(distance_sensor), 2};
	uORB::SubscriptionData<distance_sensor_s> _sub_dist3{ORB_ID(distance_sensor), 3};
	uORB::SubscriptionData<distance_sensor_s> *_dist_subs[N_DIST_SUBS] {};
	uORB::SubscriptionData<distance_sensor_s> *_sub_lidar{nullptr};
	uORB::SubscriptionData<distance_sensor_s> *_sub_sonar{nullptr};
	uORB::SubscriptionData<landing_target_pose_s> _sub_landing_target_pose{ORB_ID(landing_target_pose)};
	uORB::SubscriptionData<vehicle_air_data_s> _sub_airdata{ORB_ID(vehicle_air_data)};

	// publications
	uORB::PublicationData<vehicle_local_position_s> _pub_lpos{ORB_ID(vehicle_local_position)};
	uORB::PublicationData<vehicle_global_position_s> _pub_gpos{ORB_ID(vehicle_global_position)};
	uORB::PublicationData<vehicle_odometry_s> _pub_odom{ORB_ID(vehicle_odometry)};
	uORB::PublicationData<estimator_states_s> _pub_est_states{ORB_ID(estimator_states)};
	uORB::PublicationData<estimator_status_s> _pub_est_status{ORB_ID(estimator_status)};
	uORB::PublicationData<estimator_innovations_s> _pub_innov{ORB_ID(estimator_innovations)};
	uORB::PublicationData<estimator_innovations_s> _pub_innov_var{ORB_ID(estimator_innovation_variances)};

	// map projection
	MapProjection _map_ref;

	MapProjection _global_local_proj_ref{};
	float _global_local_alt0{NAN};

	// target mode paramters from landing_target_estimator module
	enum TargetMode {
		Target_Moving = 0,
		Target_Stationary = 1
	};

	// flow gyro filter
	BlockHighPass _flow_gyro_x_high_pass;
	BlockHighPass _flow_gyro_y_high_pass;

	// stats
	BlockStats<float, n_y_baro> _baroStats;
	BlockStats<float, n_y_sonar> _sonarStats;
	BlockStats<float, n_y_lidar> _lidarStats;
	BlockStats<float, 1> _flowQStats;
	BlockStats<float, n_y_vision> _visionStats;
	BlockStats<float, n_y_mocap> _mocapStats;
	BlockStats<double, n_y_gps> _gpsStats;
	uint16_t _landCount;

	// low pass
	BlockLowPassVector<float, n_x> _xLowPass;
	BlockLowPass _aglLowPass;

	// delay blocks
	BlockDelay<float, n_x, 1, HIST_LEN> _xDelay;
	BlockDelay<uint64_t, 1, 1, HIST_LEN> _tDelay;

	// misc
	uint64_t _timeStamp;
	uint64_t _time_origin;
	uint64_t _timeStampLastBaro;
	uint64_t _time_last_hist;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_init_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_mocap;
	uint64_t _time_last_land;
	uint64_t _time_last_target;

	// reference altitudes
	float _altOrigin;			//原点的海拔
	bool _altOriginInitialized;	//原点海拔初始化
	bool _altOriginGlobal; // true when the altitude of the origin is defined wrt a global reference frame
	float _baroAltOrigin;		//原点的气压高度
	float _gpsAltOrigin;		//原点的gps高度

	// status
	bool _receivedGps;
	bool _lastArmedState;

	// masks
	uint16_t _sensorTimeout;
	uint16_t _sensorFault;
	uint8_t _estimatorInitialized;

	// sensor update flags
	bool _flowUpdated;
	bool _gpsUpdated;
	bool _visionUpdated;
	bool _mocapUpdated;
	bool _lidarUpdated;
	bool _sonarUpdated;
	bool _landUpdated;
	bool _baroUpdated;

	// sensor validation flags
	bool _vision_xy_valid;
	bool _vision_z_valid;
	bool _mocap_xy_valid;
	bool _mocap_z_valid;

	// sensor std deviations
	float _vision_eph;
	float _vision_epv;
	float _mocap_eph;
	float _mocap_epv;

	// local to global coversion related variables
	bool _is_global_cov_init;
	double _ref_lat;
	double _ref_lon;
	float _ref_alt;

	// state space
	Vector<float, n_x>  _x;	// state vector  状态向量
	Vector<float, n_u>  _u;	// input vector   系统输入量
	Matrix<float, n_x, n_x> m_P;	// state covariance matrix  状态协方差矩阵

	matrix::Dcm<float> _R_att;

	Matrix<float, n_x, n_x>  m_A;	// dynamics matrix  动态矩阵，也叫系统矩阵
	Matrix<float, n_x, n_u>  m_B;	// input matrix     输入矩阵
	Matrix<float, n_u, n_u>  m_R;	// input covariance 输入的噪声协方差矩阵
	Matrix<float, n_x, n_x>  m_Q;	// process noise covariance  过程噪声的协方差矩阵


	DEFINE_PARAMETERS(
		// general parameters
		(ParamInt<px4::params::LPE_FUSION>) _param_lpe_fusion,
		(ParamFloat<px4::params::LPE_VXY_PUB>) _param_lpe_vxy_pub,
		(ParamFloat<px4::params::LPE_Z_PUB>) _param_lpe_z_pub,

		// sonar parameters
		(ParamFloat<px4::params::LPE_SNR_Z>) _param_lpe_snr_z,
		(ParamFloat<px4::params::LPE_SNR_OFF_Z>) _param_lpe_snr_off_z,

		// lidar parameters
		(ParamFloat<px4::params::LPE_LDR_Z>) _param_lpe_ldr_z,
		(ParamFloat<px4::params::LPE_LDR_OFF_Z>) _param_lpe_ldr_off_z,

		// accel parameters
		(ParamFloat<px4::params::LPE_ACC_XY>) _param_lpe_acc_xy,
		(ParamFloat<px4::params::LPE_ACC_Z>) _param_lpe_acc_z,

		// baro parameters
		(ParamFloat<px4::params::LPE_BAR_Z>) _param_lpe_bar_z,

		// gps parameters
		(ParamFloat<px4::params::LPE_GPS_DELAY>) _param_lpe_gps_delay,
		(ParamFloat<px4::params::LPE_GPS_XY>) _param_lpe_gps_xy,
		(ParamFloat<px4::params::LPE_GPS_Z>) _param_lpe_gps_z,
		(ParamFloat<px4::params::LPE_GPS_VXY>) _param_lpe_gps_vxy,
		(ParamFloat<px4::params::LPE_GPS_VZ>) _param_lpe_gps_vz,
		(ParamFloat<px4::params::LPE_EPH_MAX>) _param_lpe_eph_max,
		(ParamFloat<px4::params::LPE_EPV_MAX>) _param_lpe_epv_max,

		// vision parameters
		(ParamFloat<px4::params::LPE_VIS_XY>) _param_lpe_vis_xy,
		(ParamFloat<px4::params::LPE_VIS_Z>) _param_lpe_vis_z,
		(ParamFloat<px4::params::LPE_VIS_DELAY>) _param_lpe_vis_delay,

		// mocap parameters
		(ParamFloat<px4::params::LPE_VIC_P>) _param_lpe_vic_p,

		// flow parameters
		(ParamFloat<px4::params::LPE_FLW_OFF_Z>) _param_lpe_flw_off_z,
		(ParamFloat<px4::params::LPE_FLW_SCALE>) _param_lpe_flw_scale,
		(ParamInt<px4::params::LPE_FLW_QMIN>) _param_lpe_flw_qmin,
		(ParamFloat<px4::params::LPE_FLW_R>) _param_lpe_flw_r,
		(ParamFloat<px4::params::LPE_FLW_RR>) _param_lpe_flw_rr,

		// land parameters
		(ParamFloat<px4::params::LPE_LAND_Z>) _param_lpe_land_z,
		(ParamFloat<px4::params::LPE_LAND_VXY>) _param_lpe_land_vxy,

		// process noise
		(ParamFloat<px4::params::LPE_PN_P>) _param_lpe_pn_p,
		(ParamFloat<px4::params::LPE_PN_V>) _param_lpe_pn_v,
		(ParamFloat<px4::params::LPE_PN_B>) _param_lpe_pn_b,
		(ParamFloat<px4::params::LPE_PN_T>) _param_lpe_pn_t,
		(ParamFloat<px4::params::LPE_T_MAX_GRADE>) _param_lpe_t_max_grade,

		(ParamFloat<px4::params::LPE_LT_COV>) _param_lpe_lt_cov,
		(ParamInt<px4::params::LTEST_MODE>) _param_ltest_mode,

		// init origin
		(ParamInt<px4::params::LPE_FAKE_ORIGIN>) _param_lpe_fake_origin,
		(ParamFloat<px4::params::LPE_LAT>) _param_lpe_lat,
		(ParamFloat<px4::params::LPE_LON>) _param_lpe_lon
	)

};

/*
kalman filter:
	预测步 x(k) = A * x(k-1) + B * u(k-1)
		  p(k) = A * p(k-1) * AT + Q
	x: 后验状态估计值
	p: 后验估计协方差
	A: 状态转移矩阵
	B: 外部输入转换为状态的矩阵
	Q: 系统过程协方差矩阵, 表示状态转换矩阵与实际过程之间的误差

	更新步: K = p(k) * HT / (H * p(k) * HT + R)
			x(k) = x(k) + K * (Z - H * x(k))
			p(k) = (I - K * H) * p(k)
	H: 状态变量到观测量的转换矩阵
		表示将状态和观测连接起来的关系，卡尔曼滤波里为线性关系
	Z: 测量值
	K: 卡尔曼增益
	R: 测量噪声协方差矩阵
	(Z - H * x(k)): 实际观测和预测观测的残差, 和卡尔曼增益一起修正先验（预测），得到后验。

local_position_estimator:

	系统状态方程: x(+) = A * x(-) + B * u(+)
	观测方程: y_i = C_i*x
	x 为状态向量
	A 为状态转移矩阵
	u 为状态控制向量
	B 为控制变量到观测状态矩阵
	w 为控制系统噪声, 服从高斯分布 w ~ N(0, Q)

DCM:
					[ 1-2(q2^2+q3^2)			2(q1q2-q0q3)			2(q1q3+q0q2) ]
C(b_R)  =			[		2(q1q2+q0q3)		1-2(q1^2+q3^2)			2(q2q3-q0q1) ]
					[		2(q1q3-q0q2)			2(q2q3+q0q1)     1-2(q1^2+q2^2)  ]

状态向量 _x:
	[x, y, z, vx, vy, vz, bx, by, bz, tz]^T
控制向量 _u:
	[ax, ay, az]^T
协方差矩阵 m_P(初始化)
		x 		y 		z  		vx 		vy  	vz  	bx 		by  	bz 		tz
	x   [2*2, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	y   [0, 	2*2, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	z   [0, 	0, 		2*2, 	0, 		0, 		0, 		0, 		0, 		0, 		0]
	vx  [0, 	0, 		0, 		2*0.3^2,0, 		0, 		0, 		0, 		0, 		0]
	vy  [0, 	0, 		0, 		0, 		2*0.3^2,0, 		0, 		0, 		0, 		0]
	vz  [0, 	0, 		0, 		0, 		0, 		2*0.3^2,0, 		0, 		0, 		0]
	bx  [0, 	0, 		0, 		0, 		0, 		0, 		1e-6, 	0, 		0, 		0]
	by  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		1e-6, 	0, 		0]
	bz  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		1e-6, 	0]
	tz  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		2*2]

过程噪声w的协方差矩阵 m_Q
		x 		y 		z  		vx 		vy  	vz  	bx 		by  	bz 		tz
	x   [0.1^2, 0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	y   [0, 	0.1^2, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	z   [0, 	0, 		0.1^2, 	0, 		0, 		0, 		0, 		0, 		0, 		0]
	vx  [0, 	0, 		0, 		0.1^2,	0, 		0, 		0, 		0, 		0, 		0]
	vy  [0, 	0, 		0, 		0, 		0.1^2,	0, 		0, 		0, 		0, 		0]
	vz  [0, 	0, 		0, 		0, 		0, 		0.1^2,	0, 		0, 		0, 		0]
	bx  [0, 	0, 		0, 		0, 		0, 		0, 		0.001^2,0, 		0, 		0]
	by  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0.001^2,0, 		0]
	bz  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0.001^2,0]
	tz  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		ph^2]

	ph(k+1) = ph(k) + 1/100*sqrt(vx^2 + vy^2) ph(0) = 0.001
	0.1, 0.001为位置, 速度, 加速度等的传播噪声密度

状态转移矩阵 m_A:
		x 		y 		z  		vx 		vy  	vz  	bx 		by  	bz 		tz
	x   [0, 	0, 		0, 		1, 		0, 		0, 		0, 		0, 		0, 		0]
	y   [0, 	0, 		0, 		0, 		1, 		0, 		0, 		0, 		0, 		0]
	z   [0, 	0, 		0, 		0, 		0, 		1, 		0, 		0, 		0, 		0]
	vx  [0, 	0, 		0, 		0, 		0, 		0, 		-dcm, 	-dcm, 	-dcm, 	0]
	vy  [0, 	0, 		0, 		0, 		0, 		0, 		-dcm, 	-dcm, 	-dcm, 	0]
	vz  [0, 	0, 		0, 		0, 		0, 		0, 		-dcm, 	-dcm, 	-dcm, 	0]
	bx  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	by  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	bz  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	tz  [0, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]

输入矩阵 m_B
	x 	[0,		0,		0]
	y 	[0,		0,		0]
	z 	[0,		0,		0]
	vx 	[1,		0,		0]
	vy 	[0,		1,		0]
	vz 	[0,		0,		1]
	bx 	[0,		0,		0]
	by 	[0,		0,		0]
	bz 	[0,		0,		0]
	tz 	[0,		0,		0]
		ax 		ay 		az

输入u的噪声协方差矩阵 m_R
	ax 	[0.012^2,		0,				0]
	ay 	[0,				0.012^2,		0]
	az 	[0,				0,				0.02^2]
		ax 		ay 		az	
	0.012, 0.02为加速度计xy,z 向的噪声密度

GPS测量矩阵C:
			x 		y 		z  		vx 		vy  	vz  	bx 		by  	bz 		tz
	gps_x   [1, 	0, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	gps_y   [0, 	1, 		0, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	gps_z   [0, 	0, 		1, 		0, 		0, 		0, 		0, 		0, 		0, 		0]
	gps_vx  [0, 	0, 		0, 		1, 		0, 		0, 		0, 		0, 		0, 		0]
	gps_vy  [0, 	0, 		0, 		0, 		1, 		0, 		0, 		0, 		0, 		0]
	gps_vz  [0, 	0, 		0, 		0, 		0, 		1, 		0, 		0, 		0, 		0]

GPS测量矩阵R:
		x 		y 		z  		vx 		vy  	vz
	x   [1, 	0, 		0, 		0, 		0, 		0]
	y   [0, 	1, 		0, 		0, 		0, 		0]
	z   [0, 	0, 		3^2, 	0, 		0, 		0]
	vx  [0, 	0, 		0, 		0.25^2, 0, 		0]
	vy  [0, 	0, 		0, 		0, 		0.25^2, 0]
	vz  [0, 	0, 		0, 		0, 		0, 		0.25^2]
	1, 3, 0.25 分别为GPS各向的标准偏差


update实际过程:
	预测步 predict:
				获取姿态矩阵 _R_att
				提取加速度a, 并将其旋转到地球坐标系上, 获得_u
				_u在z轴上加上重力补偿
				根据_R_att更新m_A状态转移矩阵
				更新 m_Q, m_R, m_B矩阵(只有m_Q矩阵的(tz, tz)是迭代的)
				根据系统状态方程(xk = A*xk-1 + B*u)的龙格库塔解法, 计算出dx增量
				更新状态值_x
				根据一阶微分方程计算出系统协方差矩阵增量
					dP = (A * P + P * AT + B * R * BT + Q) * getDt();
				更新系统协方差 m_P
				将_x输入低通滤波器

	更新步 correct (GPS部分):

				更新测量值 y[x, y, z, vx, vy, vz]
				更新测量矩阵C (相关单位阵)
				更新测量协方差矩阵R:
					判断直接使用参数定值还是实时的eph, epv的平方
				获得上一次的观测值 y_last
				计算残差 r = y - C * y_last;
				计算残差协方差 S = C * m_P * CT + R;
				发布相关算法统计量话题
				残差协方差S求逆 -> S_I
					算出故障检测参数值: beta =  (rT * (S_I * r))(0, 0);
					判断是否符合性能要求

				计算卡尔曼增益 K = m_P * CT * S_I;
				计算后验估计增量 dx = K * r;
				修正后验估计 _x += dx
				更新协方差矩阵 m_P -= K * C * m_P

*/

```

### 2.4 null

