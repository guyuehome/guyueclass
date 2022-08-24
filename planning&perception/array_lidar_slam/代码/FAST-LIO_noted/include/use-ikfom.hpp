#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; // S2 流形
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

// 定义的ieskf状态空间
MTK_BUILD_MANIFOLD(state_ikfom,
				   ((vect3, pos))((SO3, rot))((SO3, offset_R_L_I))((vect3, offset_T_L_I))((vect3, vel))((vect3, bg))((vect3, ba)) // S2流形,grav为负值
				   ((S2, grav)));

// 定义的输入状态
MTK_BUILD_MANIFOLD(input_ikfom,
				   ((vect3, acc))((vect3, gyro)));

//定义的协方差噪声格式
//角速度(3),加速度(3),角速度偏置(3),加速度偏置(3)
MTK_BUILD_MANIFOLD(process_noise_ikfom,
				   ((vect3, ng))((vect3, na))((vect3, nbg))((vect3, nba)));

// 噪声协方差初始化
MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);	  // 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001);	  // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001); // 0.001 0.05 0.0001/out 0.01
	return cov;
}

// double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia
// vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);
// fast_lio2论文公式(2), 起始这里的f就是将imu的积分方程组成矩阵形式然后再去计算
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero(); // 将imu积分方程矩阵初始化为0,这里的24个对应了速度(3)，角速度(3),外参偏置T(3),外参偏置R(3)，加速度(3),角速度偏置(3),加速度偏置(3),位置(3)，与论文公式不一致
	vect3 omega;
	in.gyro.boxminus(omega, s.bg); // 得到imu的角速度
	// 加速度转到世界坐标系
	vect3 a_inertial = s.rot * (in.acc - s.ba);
	for (int i = 0; i < 3; i++)
	{
		res(i) = s.vel[i];						 //更新的速度
		res(i + 3) = omega[i];					 //更新的角速度
		res(i + 12) = a_inertial[i] + s.grav[i]; //更新的加速度
	}
	return res;
}
// 对应fast_lio2论文公式(7)
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
	//当中的23个对应了status的维度计算，为pos(3), rot(3),offset_R_L_I(3),offset_T_L_I(3), vel(3), bg(3), ba(3), grav(2);这一块和fast-lio不同需要注意
	Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity(); //一开始是一个R3的单位阵，代表速度转移
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba); //拿到加速度
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);												  //拿到角速度
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix() * MTK::hat(acc_); //这里的-s.rot.toRotationMatrix()是因为论文中的矩阵是逆时针旋转的
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();				  // 将角度转到存入的矩阵中（应该与上面颠倒了）
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	s.S2_Mx(grav_matrix, vec, 21);									//将vec的2*1矩阵转为grav_matrix的3*2矩阵
	cov.template block<3, 2>(12, 21) = grav_matrix;					//存入到矩阵中
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); //角速度存入
	return cov;
}
// 对应fast_lio2论文公式(7)
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();   //加速度
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity(); //角速度
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity(); //角速度偏置
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity(); //加速度偏置
	return cov;
}

vect3 SO3ToEuler(const SO3 &orient) //将SO3转为Euler角
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose(); //将SO3转为4*1的矩阵
	// scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3] * q_data[3];
	double sqx = q_data[0] * q_data[0];
	double sqy = q_data[1] * q_data[1];
	double sqz = q_data[2] * q_data[2];
	double unit = sqx + sqy + sqz + sqw; //如果归一为1，否则为校正因子
	double test = q_data[3] * q_data[1] - q_data[2] * q_data[0];

	if (test > 0.49999 * unit)
	{ // 在北极点

		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI / 2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999 * unit)
	{ // 在南极点
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI / 2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	// 否则正常计算
	_ang << std::atan2(2 * q_data[0] * q_data[3] + 2 * q_data[1] * q_data[2], -sqx - sqy + sqz + sqw),
		std::asin(2 * test / unit),
		std::atan2(2 * q_data[2] * q_data[3] + 2 * q_data[1] * q_data[0], sqx - sqy - sqz + sqw); //转为弧度
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};							  //转为角度
	vect3 euler_ang(temp, 3);
	// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif