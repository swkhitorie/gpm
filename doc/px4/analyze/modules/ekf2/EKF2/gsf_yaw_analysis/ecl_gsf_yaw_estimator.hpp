/**
	https://zhuanlan.zhihu.com/p/602518216
	https://zhuanlan.zhihu.com/p/428920320
	https://zhuanlan.zhihu.com/p/359222413
	https://blog.csdn.net/lovely_yoshino/article/details/127844900
	https://github.com/ericzzj1989/matlab_px4_msf/blob/master/Analysis%20of%20PX4%20ECL.pdf
	innovation (Z - HX) 测量残差
	https://www.coder.work/article/1951073 约瑟夫形式??
	该算法专用于INS/GPS组合导航的 yaw航向确定, 因为磁场环境随着环境变化为变化, 
	同时rtk层本高, 所以使用该算法来解决问题

	引入EKFGSF_yaw的目的，就是为应对磁场异常情况对yaw进行重置、
	或直接使用EKFGSF_yaw计算yaw并作为主滤波器航向初值

	EKFGSF_yaw核心就是使用gps速度共同来确定yaw航向
	ECL 项目自2018年起进行的初步算法工作，以确定是否有可能仅使用 IMU 和 GPS 速度数据快速确定偏航角(yaw)，
	而不依赖机体动力学的假设。最后确定使用多个 EKF 状态的高斯和滤波器(Gaussian Sum Filter, GSF)算法，
	即EKF-GSF，因其为应用提供了最佳的性能/计算成本权衡
*/


/**
	5组 3参数状态的ekf
	struct _ekf_gsf_struct{
		matrix::Vector3f X; 				// 北向速度, 东向速度, 偏航角
		matrix::SquareMatrix<float, 3> P; 		// 协方差
		matrix::SquareMatrix<float, 2> S_inverse;	// 残差协方差矩阵的逆
		float S_det_inverse; 				// 残差协方差矩阵行列式的逆
		matrix::Vector2f innov; 			// 北向速度, 动向速度的残差
	} _ekf_gsf[N_MODELS_EKFGSF]{};

	X: [v_n, v_e, yaw]^T 	-> 北向速度, 东向速度, 偏航角
	X初始状态 v_n = v_e = 0, 
		yaw偏航角度估算开始时的角度间隔相等 -pi~pi之间的均匀分布 [-4/5pi, -2/5pi, 0, 2/5pi, 4/5pi]
	P初始状态: [vel_accuracy^(1/2)	0	0]
	   		  [0	vel_accuracy^(1/2)	0]
	          [0	0		(2pi/5)^(1/2)]
	================================================================================================

	5组 使用互补滤波器的 AHRS解
		预测yaw和向前,向右加速度
		空速(测量或估计)用于固定翼飞行期间的向心加速度修正

	struct _ahrs_ekf_gsf_struct{
		Dcmf R;			// 旋转矩阵, 可将矢量从 体坐标系 旋转到 地球坐标系
		Vector3f gyro_bias;	// 四元数计算中学习和使用的陀螺偏置
		bool aligned;		// AHRS是否已对齐
		float vel_NE[2];	// 上次GPS测量的NE速度矢量（m/s）
		bool fuse_gps;		// 当GPS应该融合在该帧上时为true
		float accel_dt;		// 生成_simple_accel_FR数据时使用的时间步长（秒）
	} _ahrs_ekf_gsf[N_MODELS_EKFGSF]{};

	Gauss Sum Filter高斯和滤波器???
*/


void EKFGSF_yaw::update(const imuSample& imu_sample,
			bool run_EKF,			// 当飞行或移动适合偏航估计时设置为true
			float airspeed,			// 用于向心加速度补偿的真实空速-不需要时设置为0
			const Vector3f &imu_gyro_bias)  // 估计速率陀螺偏差 (rad/sec)
{
	// ***拷贝数据到本类中***
	// 获取角度增量, 速度增量, 角速度周期, 速度周期
	// 获取ekf运行标志, 空速数据
	_delta_ang = imu_sample.delta_ang;
	_delta_vel = imu_sample.delta_vel;
	_delta_ang_dt = imu_sample.delta_ang_dt;
	_delta_vel_dt = imu_sample.delta_vel_dt;
	_run_ekf_gsf = run_EKF;
	_true_airspeed = airspeed;

	// ***计算加速度***
	// _tilt_gain: 互补滤波器从倾斜误差到陀螺校正的增益（1秒）
	// 为了减少振动的影响，使用时间常数为AHRS倾斜校正时间常数的1/10的LPF进行滤波
	// 计算互补滤波参数 min(10 * 速度周期 * 倾斜增益(0.2f), 1.0f)
	// 根据速度增量和速度周期计算加速度并由低通滤波输出
	const float filter_coef = fminf(10.0f * _delta_vel_dt * _tilt_gain, 1.0f);
	const Vector3f accel = _delta_vel / fmaxf(_delta_vel_dt, 0.001f);
	_ahrs_accel = _ahrs_accel * (1.0f - filter_coef) + accel * filter_coef;

	// 是否进行了AHRS倾斜对齐
	if (!_ahrs_ekf_gsf_tilt_aligned) {
		// 检查加速度是否过大，以减少由于车辆移动而产生较大初始侧倾/俯仰误差的可能性
		const float accel_norm_sq = accel.norm_squared();
		const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
		const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
		// 若车辆正确对齐与地面坐标系上
		const bool ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit));
		if (ok_to_align) {
			// 三态EKF模型执行初始化
			initialiseEKFGSF();
			// 进行AHRS倾斜对齐
			ahrsAlignTilt();
			_ahrs_ekf_gsf_tilt_aligned = true;
		}
		return;
	}

	// 计算 AHRS互补滤波器 加速度的模
	_ahrs_accel_norm = _ahrs_accel.norm();

	// AHRS 预测步
	_ahrs_accel_fusion_gain = ahrsCalcAccelGain();
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		predictEKF(model_index);
	}

	// 三态EKF模型仅在飞行时运行，以避免因操作员操作和GPS干扰而导致的估计值损坏
	if (_run_ekf_gsf && _vel_data_updated) {
		// 融合是否开始
		if (!_ekf_gsf_vel_fuse_started) {
			// 三态EKF模型执行初始化
			initialiseEKFGSF();
			// 进行AHRS航向对齐
			ahrsAlignYaw();
			// 初始化主滤波器的陀螺偏置估计，因为重力矢量可能存在较大的未校正速率陀螺偏置误差
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				_ahrs_ekf_gsf[model_index].gyro_bias = imu_gyro_bias;
			}
			_ekf_gsf_vel_fuse_started = true;
		} else {
			bool bad_update = false;
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				// EKF更新步
				if (!updateEKF(model_index)) {
					bad_update = true;
				}
			}

			// 如果结果质量达到要求
			if (!bad_update) {
				float total_weight = 0.0f;
				// 假设正态分布，计算每个模型的权重
				const float min_weight = 1E-5f;
				uint8_t n_weight_clips = 0;
				for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
					// 权重经过高斯和滤波器出来的密度-> 残差越小, 权重越大
					_model_weights(model_index) = gaussianDensity(model_index) * _model_weights(model_index);
					if (_model_weights(model_index) < min_weight) {
						// 如果权重过小, 计数器增加
						n_weight_clips++;
						_model_weights(model_index) = min_weight;
					}
					total_weight += _model_weights(model_index);
				}

				// 归一化加权函数
				if (n_weight_clips < N_MODELS_EKFGSF) {
					// 如果至少有几个模型满足要求
					// 将所有权重归一化
					_model_weights /= total_weight;
				} else {
					// 由于残差差异过大，所有权重都已崩溃，因此重置过滤器
					initialiseEKFGSF();
				}

				//强制使用最小权重值。这是在最初的开发过程中添加的，但后来没有需要，
				// 所以如果我们在没有任何加权函数问题的情况下通过测试，可以删除这段代码和相应的_weight_min。
				if (_weight_min > FLT_EPSILON) {
					float correction_sum = 0.0f; // 通过应用限制增加的重量总和
					bool change_mask[N_MODELS_EKFGSF] = {}; // 当该模型的权重增加时为true
					float unmodified_weights_sum = 0.0f; // 未修改权重之和
					for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
						if (_model_weights(model_index) < _weight_min) {
							correction_sum += _weight_min - _model_weights(model_index);
							_model_weights(model_index) = _weight_min;
							change_mask[model_index] = true;
						} else {
							unmodified_weights_sum += _model_weights(model_index);
						}
					}

					// 重新缩放未修改的权重以使总和为1
					const float scale_factor = (unmodified_weights_sum - correction_sum - _weight_min) / (unmodified_weights_sum - _weight_min);
					for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
						if (!change_mask[model_index]) {
							_model_weights(model_index) = _weight_min + scale_factor * (_model_weights(model_index) - _weight_min);
						}
					}
				}
			}
		}
	} else if (_ekf_gsf_vel_fuse_started && !_run_ekf_gsf) {
		// 等待再次飞行
		_ekf_gsf_vel_fuse_started = false;
	}

	// 计算复合偏航矢量作为每个模型的状态的加权平均值
	// 为了避免角度包裹的问题，在求和之前，将偏航状态转换为长度等于加权值的矢量。
	Vector2f yaw_vector;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		yaw_vector(0) += _model_weights(model_index) * cosf(_ekf_gsf[model_index].X(2));
		yaw_vector(1) += _model_weights(model_index) * sinf(_ekf_gsf[model_index].X(2));
	}
	_gsf_yaw = atan2f(yaw_vector(1),yaw_vector(0));

	// 根据每个模型方差的加权平均值计算偏航状态的综合方差，具有较大残差的模型加权较小
	_gsf_yaw_variance = 0.0f;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		const float yaw_delta = wrap_pi(_ekf_gsf[model_index].X(2) - _gsf_yaw);
		_gsf_yaw_variance += _model_weights(model_index) * (_ekf_gsf[model_index].P(2,2) + yaw_delta * yaw_delta);
	}

	// 防止多次使用相同的速度数据
	_vel_data_updated = false;
}




/**
	************ Gauss高斯和滤波器 ************
	一维高斯函数:
	其中 δ为x的方差, μ为x的均值
	f(x) = [1 / (((2pi)^1/2) * δ)] * e^((-1/2) * ((x-μ)^2) / 2δ^2)
*/
float EKFGSF_yaw::gaussianDensity(const uint8_t model_index) const
{
	// 计算马式距离(Mahalanobis distance) https://en.wikipedia.org/wiki/Mahalanobis_distance
	// 马氏距离为 标准分数平方的多变量推广 z = (x-μ) / δ
	// DM(马氏距离) = [(x-μ)^T * S^-1 * (x-μ)] ^ 1/2 μ->平均值 S->正定协方差矩阵
	// calculate transpose(innovation) * inv(S) * innovation
	const float normDist = _ekf_gsf[model_index].innov.dot(_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	// 再计算2D正态分布(Multivariate Normal Distribution)的密度 https://online.stat.psu.edu/stat505/book/export/html/636
	// Density = (1/2pi) * det(S^-1)^1/2 * exp(-DM^2 / 2)密度
	// 即残差越小, 显然则计算出来的2D正态分布密度越高
	return _m_2pi_inv * sqrtf(_ekf_gsf[model_index].S_det_inverse) * expf(-0.5f * normDist);
}



/**
	初始化5个ekf估计模型结构体, 并将其状态设置为初始状态,
	其中yaw初始值为5个在-pi~pi的步长为2pi/5的均匀分布值
	并设置高斯滤波器的权重为均匀的1/5
*/
void EKFGSF_yaw::initialiseEKFGSF()
{
	_gsf_yaw = 0.0f;
	_ekf_gsf_vel_fuse_started = false;
	_gsf_yaw_variance = _m_pi2 * _m_pi2;
	_model_weights.setAll(1.0f / (float)N_MODELS_EKFGSF);

	// 初始化5个ekf估计模型结构体
	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	const float yaw_increment = 2.0f * _m_pi / (float)N_MODELS_EKFGSF;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		// evenly space initial yaw estimates in the region between +-Pi
		_ekf_gsf[model_index].X(2) = -_m_pi + (0.5f * yaw_increment) + ((float)model_index * yaw_increment);

		// take velocity states and corresponding variance from last measurement
		_ekf_gsf[model_index].X(0) = _vel_NE(0);
		_ekf_gsf[model_index].X(1) = _vel_NE(1);
		_ekf_gsf[model_index].P(0,0) = sq(_vel_accuracy);
		_ekf_gsf[model_index].P(1,1) = _ekf_gsf[model_index].P(0,0);

		// use half yaw interval for yaw uncertainty
		_ekf_gsf[model_index].P(2,2) = sq(0.5f * yaw_increment);
	}
}




/**
	************ AHRS 倾斜对齐 ************
	根据速度增量来设置倾斜对齐矩阵(旋转矩阵)
		**假设:偏航为0/加速度全由重力驱动
		计算机体坐标系的归一重力方向向量Db
		计算地球坐标系的北轴单位向量, 旋转为机体坐标系并与Db正交得到Nb
		Nb和Db叉乘, 得到一对坐标基
		将这对坐标基作为旋转矩阵赋值到5个ekf模型中的R
*/
void EKFGSF_yaw::ahrsAlignTilt()
{
	//旋转矩阵是直接从加速度测量中构建的，对于所以所有的模型只需要计算一次。假设是：
	// 1) 偏航角为零——当速度融合开始时，稍后对每个模型的偏航进行校准。
	// 2) 车辆没有加速，因此所有测得的加速度都是由于重力造成的

	// 计算地球坐标系旋转到身体坐标系的重力方向单位向量
	// Db = -△V / ||△V||
	const Vector3f down_in_bf = -_delta_vel.normalized();
 
	// 计算地球坐标系北轴单位向量，旋转到身体坐标系中，与 “down_in_bf” Db 正交
	// Nb = [1, 0, 0]^T - Db · ([1, 0, 0]Db) (施密特正交)
	// Nb = Nb / ||Nb|| 
	const Vector3f i_vec_bf(1.0f,0.0f,0.0f);
	Vector3f north_in_bf = i_vec_bf - down_in_bf * (i_vec_bf.dot(down_in_bf));
	north_in_bf.normalize();

	// 计算地球坐标系旋转到身体坐标系中的东轴单位向量，垂直于“down_in_bf” Db 和“north_in_bf” Nb
	// Eb = Db × Nb
	const Vector3f east_in_bf = down_in_bf % north_in_bf;

	// 从地球帧到身体帧的旋转矩阵中的每一列表示旋转到身体帧中的相应地球帧单位向量的投影，
	// 例如“north_in_bf”将是第一列。我们需要从体帧到地帧的旋转矩阵，
	// 因此旋转到体帧的地帧单位向量被复制到相应的行中。
	Dcmf R;
	R.setRow(0, north_in_bf);
	R.setRow(1, east_in_bf);
	R.setRow(2, down_in_bf);

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		_ahrs_ekf_gsf[model_index].R = R;
	}
}




/**
	************ AHRS 偏航对齐 ************
		给5个3参数状态ekf进行偏航角对齐
		根据模型最新的偏航角更新欧拉角向量, 进而生成新的姿态矩阵
			即 使用3参数状态的ekf自己估计的yaw来更新对齐自己维护的姿态矩阵R
*/
void EKFGSF_yaw::ahrsAlignYaw()
{
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		if (fabsf(_ahrs_ekf_gsf[model_index].R(2, 0)) < fabsf(_ahrs_ekf_gsf[model_index].R(2, 1))) {
			// 获取姿态角, 321 Tait Bryan旋转序列
			Eulerf euler_init(_ahrs_ekf_gsf[model_index].R);

			// set the yaw angle
			euler_init(2) = wrap_pi(_ekf_gsf[model_index].X(2));

			// update the rotation matrix
			_ahrs_ekf_gsf[model_index].R = Dcmf(euler_init);

		} else {
			// 获取姿态角, 312 Tait Bryan旋转序列
			const Vector3f rot312(wrap_pi(_ekf_gsf[model_index].X(2)),  // yaw
					      asinf(_ahrs_ekf_gsf[model_index].R(2, 1)),  // roll
					      atan2f(-_ahrs_ekf_gsf[model_index].R(2, 0),
						      _ahrs_ekf_gsf[model_index].R(2, 2)));  // pitch

			//计算旋转矩阵
			_ahrs_ekf_gsf[model_index].R = taitBryan312ToRotMat(rot312);

		}
		_ahrs_ekf_gsf[model_index].aligned = true;
	}
}




/**
	************ 计算加速度补偿增益(有空速计的固定翼专用) ************
	根据空速数据是否存在以及加速度是否大于重力加速度来调整增益函数
	根据增益函数以及倾斜增益来获取加速度补偿增益
	增益函数来自于: https://www.desmos.com/calculator/dbqbxvnwfg?lang=zh-CN
*/
float EKFGSF_yaw::ahrsCalcAccelGain() const
{
	// Calculate the acceleration fusion gain using a continuous function that is unity at 1g and zero
	// at the min and max g value. Allow for more acceleration when flying as a fixed wing vehicle using centripetal
	// acceleration correction as higher and more sustained g will be experienced.
	// Use a quadratic instead of linear function to prevent vibration around 1g reducing the tilt correction effectiveness.
	// see https://www.desmos.com/calculator/dbqbxvnwfg

	float attenuation = 2.f;
	const bool centripetal_accel_compensation_enabled = (_true_airspeed > FLT_EPSILON);

	if (centripetal_accel_compensation_enabled
	    && _ahrs_accel_norm > CONSTANTS_ONE_G) {
		attenuation = 1.f;
	}

	const float delta_accel_g = (_ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G;
	return _tilt_gain * sq(1.f - math::min(attenuation * fabsf(delta_accel_g), 1.f));
}




/**
	************ AHRS互补滤波器预测 ************
		计算角速度(根据外部ekf模型的bias)
		补偿旋转矩阵(根据倾斜补偿和角速度补偿...)
*/
void EKFGSF_yaw::ahrsPredict(const uint8_t model_index)
{
	// 使用简单互补滤波器生成所选模型的姿态解
	// 计算角速度(补偿gyro的bias)
	const Vector3f ang_rate = _delta_ang / fmaxf(_delta_ang_dt, 0.001f) - _ahrs_ekf_gsf[model_index].gyro_bias;

	// 计算旋转矩阵R_to_body(to body)
	const Dcmf R_to_body = _ahrs_ekf_gsf[model_index].R.transpose();
	// 计算体坐标系的重力方向向量(R_to_body.col(2))
	const Vector3f gravity_direction_bf = R_to_body.col(2);

	// 在固定翼飞行过程中，假设转弯协调且X轴向前，补偿向心加速度
	// 使用加速度数据进行角速率校正，并在加速度幅度远离1g时减少校正（减少车辆起步和移动时的漂移）
	// 计算倾斜矫正参数 tilt_correction = 机体加速度 × 机体重力加速度 (近似为角度) * 
	//									加速度补偿增益 / 机体加速度模 (加速度离重力加速度越远, 
	//																矫正参数越小)
	Vector3f tilt_correction;
	if (_ahrs_accel_fusion_gain > 0.0f) {

		Vector3f accel = _ahrs_accel;

		if (_true_airspeed > FLT_EPSILON) {
			const Vector3f centripetal_accel_bf = Vector3f(0.0f, _true_airspeed * ang_rate(2), - _true_airspeed * ang_rate(1));

			// 向心加速度的正确测量加速度
			accel -= centripetal_accel_bf;
		}

		tilt_correction = (gravity_direction_bf % accel) * _ahrs_accel_fusion_gain / _ahrs_accel_norm;

	}

	// 计算角速度偏差(gyro bias)
	constexpr float gyro_bias_limit = 0.05f;
	const float spinRate = ang_rate.length();
	if (spinRate < 0.175f) {
		// 当角速度模小于一定值之后: 角速度偏差 = 倾斜矫正参数 * (角速度偏差增益(0.04f) * 角度增量周期)
		_ahrs_ekf_gsf[model_index].gyro_bias -= tilt_correction * (_gyro_bias_gain * _delta_ang_dt);
		_ahrs_ekf_gsf[model_index].gyro_bias = matrix::constrain(_ahrs_ekf_gsf[model_index].gyro_bias, -gyro_bias_limit, gyro_bias_limit);
	}

	// 校准角度增量 校正角度增量 = 角度增量 + (倾斜矫正参数 - 角速度偏差) * 角度增量周期
	const Vector3f delta_angle_corrected = _delta_ang + (tilt_correction - _ahrs_ekf_gsf[model_index].gyro_bias) * _delta_ang_dt;

	// 将增量角度应用于旋转矩阵
	// ekf.R = ahrsPredictRotMat(ekf.R, 校正角度增量)
	_ahrs_ekf_gsf[model_index].R = ahrsPredictRotMat(_ahrs_ekf_gsf[model_index].R, delta_angle_corrected);

}




/**
	************ AHRS 旋转矩阵补偿 ************
		根据角度增量来补偿旋转矩阵
*/
Matrix3f EKFGSF_yaw::ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g)
{
	Matrix3f ret = R;
	ret(0,0) += R(0,1) * g(2) - R(0,2) * g(1);
	ret(0,1) += R(0,2) * g(0) - R(0,0) * g(2);
	ret(0,2) += R(0,0) * g(1) - R(0,1) * g(0);
	ret(1,0) += R(1,1) * g(2) - R(1,2) * g(1);
	ret(1,1) += R(1,2) * g(0) - R(1,0) * g(2);
	ret(1,2) += R(1,0) * g(1) - R(1,1) * g(0);
	ret(2,0) += R(2,1) * g(2) - R(2,2) * g(1);
	ret(2,1) += R(2,2) * g(0) - R(2,0) * g(2);
	ret(2,2) += R(2,0) * g(1) - R(2,1) * g(0);

	// 重新对行数据进行正则化
	for (uint8_t r = 0; r < 3; r++) {
		const float rowLengthSq = ret.row(r).norm_squared();
		if (rowLengthSq > FLT_EPSILON) {
			// 利用行长度接近1.0的优势，使用线性近似进行逆sqrt
			const float rowLengthInv = 1.5f - 0.5f * rowLengthSq;
			ret(r,0) *= rowLengthInv;
			ret(r,1) *= rowLengthInv;
			ret(r,2) *= rowLengthInv;
		}
    }

	return ret;
}


/**
	************ EKF预测步 ************

*/
void EKFGSF_yaw::predictEKF(const uint8_t model_index)
{
	// generate an attitude reference using IMU data
	ahrsPredict(model_index);

	// we don't start running the EKF part of the algorithm until there are regular velocity observations
	if (!_ekf_gsf_vel_fuse_started) {
		return;
	}

	// Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
	if (fabsf(_ahrs_ekf_gsf[model_index].R(2, 0)) < fabsf(_ahrs_ekf_gsf[model_index].R(2, 1))) {
		// use 321 Tait-Bryan rotation to define yaw state
		_ekf_gsf[model_index].X(2) = atan2f(_ahrs_ekf_gsf[model_index].R(1, 0), _ahrs_ekf_gsf[model_index].R(0, 0));
	} else {
		// use 312 Tait-Bryan rotation to define yaw state
		_ekf_gsf[model_index].X(2) = atan2f(-_ahrs_ekf_gsf[model_index].R(0, 1), _ahrs_ekf_gsf[model_index].R(1, 1)); // first rotation (yaw)
	}

	// calculate delta velocity in a horizontal front-right frame
	const Vector3f del_vel_NED = _ahrs_ekf_gsf[model_index].R * _delta_vel;
	const float dvx =   del_vel_NED(0) * cosf(_ekf_gsf[model_index].X(2)) + del_vel_NED(1) * sinf(_ekf_gsf[model_index].X(2));
	const float dvy = - del_vel_NED(0) * sinf(_ekf_gsf[model_index].X(2)) + del_vel_NED(1) * cosf(_ekf_gsf[model_index].X(2));

	// sum delta velocities in earth frame:
	_ekf_gsf[model_index].X(0) += del_vel_NED(0);
	_ekf_gsf[model_index].X(1) += del_vel_NED(1);

	// predict covariance - equations generated using EKF/python/gsf_ekf_yaw_estimator/main.py

	// Local short variable name copies required for readability
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P22 = _ekf_gsf[model_index].P(2,2);
	const float &psi = _ekf_gsf[model_index].X(2);

	// Use fixed values for delta velocity and delta angle process noise variances
	const float dvxVar = sq(_accel_noise * _delta_vel_dt); // variance of forward delta velocity - (m/s)^2
	const float dvyVar = dvxVar; // variance of right delta velocity - (m/s)^2
	const float dazVar = sq(_gyro_noise * _delta_ang_dt); // variance of yaw delta angle - rad^2

	const float S0 = cosf(psi);
	const float S1 = ecl::powf(S0, 2);
	const float S2 = sinf(psi);
	const float S3 = ecl::powf(S2, 2);
	const float S4 = S0*dvy + S2*dvx;
	const float S5 = P02 - P22*S4;
	const float S6 = S0*dvx - S2*dvy;
	const float S7 = S0*S2;
	const float S8 = P01 + S7*dvxVar - S7*dvyVar;
	const float S9 = P12 + P22*S6;

	_ekf_gsf[model_index].P(0,0) = P00 - P02*S4 + S1*dvxVar + S3*dvyVar - S4*S5;
	_ekf_gsf[model_index].P(0,1) = -P12*S4 + S5*S6 + S8;
	_ekf_gsf[model_index].P(1,1) = P11 + P12*S6 + S1*dvyVar + S3*dvxVar + S6*S9;
	_ekf_gsf[model_index].P(0,2) = S5;
	_ekf_gsf[model_index].P(1,2) = S9;
	_ekf_gsf[model_index].P(2,2) = P22 + dazVar;

	// covariance matrix is symmetrical, so copy upper half to lower half
	_ekf_gsf[model_index].P(1,0) = _ekf_gsf[model_index].P(0,1);
	_ekf_gsf[model_index].P(2,0) = _ekf_gsf[model_index].P(0,2);
	_ekf_gsf[model_index].P(2,1) = _ekf_gsf[model_index].P(1,2);

	// constrain variances
	const float min_var = 1e-6f;
	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index,index) = fmaxf(_ekf_gsf[model_index].P(index,index),min_var);
	}
}





/**
	************ EKF更新步 ************
		使用速度测量更新指定模型索引的EKF状态和协方差
*/
bool EKFGSF_yaw::updateEKF(const uint8_t model_index)
{
	// set observation variance from accuracy estimate supplied by GPS and apply a sanity check minimum
	const float velObsVar = sq(fmaxf(_vel_accuracy, 0.5f));

	// calculate velocity observation innovations
	_ekf_gsf[model_index].innov(0) = _ekf_gsf[model_index].X(0) - _vel_NE(0);
	_ekf_gsf[model_index].innov(1) = _ekf_gsf[model_index].X(1) - _vel_NE(1);

	// Use temporary variables for covariance elements to reduce verbosity of auto-code expressions
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P22 = _ekf_gsf[model_index].P(2,2);

	// optimized auto generated code
	const float t0 = ecl::powf(P01, 2);
	const float t1 = -t0;
	const float t2 = P00*P11 + P00*velObsVar + P11*velObsVar + t1 + ecl::powf(velObsVar, 2);
	if (fabsf(t2) < 1e-6f) {
		return false;
	}
	const float t3 = 1.0F/t2;
	const float t4 = P11 + velObsVar;
	const float t5 = P01*t3;
	const float t6 = -t5;
	const float t7 = P00 + velObsVar;
	const float t8 = P00*t4 + t1;
	const float t9 = t5*velObsVar;
	const float t10 = P11*t7;
	const float t11 = t1 + t10;
	const float t12 = P01*P12;
	const float t13 = P02*t4;
	const float t14 = P01*P02;
	const float t15 = P12*t7;
	const float t16 = t0*velObsVar;
	const float t17 = ecl::powf(t2, -2);
	const float t18 = t4*velObsVar + t8;
	const float t19 = t17*t18;
	const float t20 = t17*(t16 + t7*t8);
	const float t21 = t0 - t10;
	const float t22 = t17*t21;
	const float t23 = t14 - t15;
	const float t24 = P01*t23;
	const float t25 = t12 - t13;
	const float t26 = t16 - t21*t4;
	const float t27 = t17*t26;
	const float t28 = t11 + t7*velObsVar;
	const float t30 = t17*t28;
	const float t31 = P01*t25;
	const float t32 = t23*t4 + t31;
	const float t33 = t17*t32;
	const float t35 = t24 + t25*t7;
	const float t36 = t17*t35;

	_ekf_gsf[model_index].S_det_inverse = t3;

	_ekf_gsf[model_index].S_inverse(0,0) = t3*t4;
	_ekf_gsf[model_index].S_inverse(0,1) = t6;
	_ekf_gsf[model_index].S_inverse(1,1) = t3*t7;
	_ekf_gsf[model_index].S_inverse(1,0) = _ekf_gsf[model_index].S_inverse(0,1);

	matrix::Matrix<float, 3, 2> K;
	K(0,0) = t3*t8;
	K(1,0) = t9;
	K(2,0) = t3*(-t12 + t13);
	K(0,1) = t9;
	K(1,1) = t11*t3;
	K(2,1) = t3*(-t14 + t15);

	_ekf_gsf[model_index].P(0,0) = P00 - t16*t19 - t20*t8;
	_ekf_gsf[model_index].P(0,1) = P01*(t18*t22 - t20*velObsVar + 1);
	_ekf_gsf[model_index].P(1,1) = P11 - t16*t30 + t22*t26;
	_ekf_gsf[model_index].P(0,2) = P02 + t19*t24 + t20*t25;
	_ekf_gsf[model_index].P(1,2) = P12 + t23*t27 + t30*t31;
	_ekf_gsf[model_index].P(2,2) = P22 - t23*t33 - t25*t36;
	_ekf_gsf[model_index].P(1,0) = _ekf_gsf[model_index].P(0,1);
	_ekf_gsf[model_index].P(2,0) = _ekf_gsf[model_index].P(0,2);
	_ekf_gsf[model_index].P(2,1) = _ekf_gsf[model_index].P(1,2);

	// constrain variances
	const float min_var = 1e-6f;
	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index,index) = fmaxf(_ekf_gsf[model_index].P(index,index),min_var);
	}

	// test ratio = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
	const float test_ratio = _ekf_gsf[model_index].innov * (_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	// Perform a chi-square innovation consistency test and calculate a compression scale factor
	// that limits the magnitude of innovations to 5-sigma
	// If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
	// This protects from large measurement spikes
	const float innov_comp_scale_factor = test_ratio > 25.f ? sqrtf(25.0f / test_ratio) : 1.f;

	// Correct the state vector and capture the change in yaw angle
	const float oldYaw = _ekf_gsf[model_index].X(2);

	_ekf_gsf[model_index].X -= (K * _ekf_gsf[model_index].innov) * innov_comp_scale_factor;

	const float yawDelta = _ekf_gsf[model_index].X(2) - oldYaw;

	// apply the change in yaw angle to the AHRS
	// take advantage of sparseness in the yaw rotation matrix
	const float cosYaw = cosf(yawDelta);
	const float sinYaw = sinf(yawDelta);
	const float R_prev00 = _ahrs_ekf_gsf[model_index].R(0, 0);
	const float R_prev01 = _ahrs_ekf_gsf[model_index].R(0, 1);
	const float R_prev02 = _ahrs_ekf_gsf[model_index].R(0, 2);

	_ahrs_ekf_gsf[model_index].R(0, 0) = R_prev00 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 0) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 1) = R_prev01 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 1) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 2) = R_prev02 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 2) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(1, 0) = R_prev00 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 0) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 1) = R_prev01 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 1) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 2) = R_prev02 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 2) * cosYaw;

	return true;
}

bool EKFGSF_yaw::getYawData(float *yaw, float *yaw_variance)
{
	if(_ekf_gsf_vel_fuse_started) {
		*yaw = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;
		return true;
	}
	return false;
}

void EKFGSF_yaw::setVelocity(const Vector2f &velocity, float accuracy)
{
	_vel_NE = velocity;
	_vel_accuracy = accuracy;
	_vel_data_updated = true;
}