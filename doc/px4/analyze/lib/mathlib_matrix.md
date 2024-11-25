## filter

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

## Functions_SearchMin

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

## Utilities_TrajMath

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

## Welford

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

## axisangle_dcm_dcm2_euler_quaternion

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

## filter_integration

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

## matrix_slice_scalar

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

## squareMatrix_SparseVector

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

## vector1_2_3_4

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

