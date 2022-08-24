/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ESEKFOM_EKF_HPP
#define ESEKFOM_EKF_HPP

#include <vector>
#include <cstdlib>

#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "../mtk/types/vect.hpp"
#include "../mtk/types/SOn.hpp"
#include "../mtk/types/S2.hpp"
#include "../mtk/startIdx.hpp"
#include "../mtk/build_manifold.hpp"
#include "util.hpp"

//#define USE_sparse

namespace esekfom
{

	using namespace Eigen;

	// used for iterated error state EKF update
	// for the aim to calculate  measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function.
	// applied for measurement as a manifold.
	template <typename S, typename M, int measurement_noise_dof = M::DOF>
	struct share_datastruct
	{
		bool valid;
		bool converge;
		M z;
		Eigen::Matrix<typename S::scalar, M::DOF, measurement_noise_dof> h_v;
		Eigen::Matrix<typename S::scalar, M::DOF, S::DOF> h_x;
		Eigen::Matrix<typename S::scalar, measurement_noise_dof, measurement_noise_dof> R;
	};

	// used for iterated error state EKF update
	// for the aim to calculate  measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function.
	// applied for measurement as an Eigen matrix whose dimension is changing
	template <typename T>
	struct dyn_share_datastruct
	{
		bool valid;
		bool converge;
		Eigen::Matrix<T, Eigen::Dynamic, 1> z;
		Eigen::Matrix<T, Eigen::Dynamic, 1> h;
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_v;
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R;
	};

	// used for iterated error state EKF update
	// for the aim to calculate  measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function.
	// applied for measurement as a dynamic manifold whose dimension or type is changing
	template <typename T>
	struct dyn_runtime_share_datastruct
	{
		bool valid;
		bool converge;
		// Z z;
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_v;
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
		Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R;
	};
	//状态量，噪声维度，输入状态量这三个参数输入
	template <typename state, int process_noise_dof, typename input = state, typename measurement = state, int measurement_noise_dof = 0>
	class esekf
	{

		typedef esekf self;
		enum
		{
			n = state::DOF,		 //状态量自由度，一般代表x的维度
			m = state::DIM,		 //状态量自由度，一般代表res的维度
			l = measurement::DOF //测量噪声维度
		};

	public:
		typedef typename state::scalar scalar_type;
		typedef Matrix<scalar_type, n, n> cov;
		typedef Matrix<scalar_type, m, n> cov_;
		typedef SparseMatrix<scalar_type> spMt;
		typedef Matrix<scalar_type, n, 1> vectorized_state;
		typedef Matrix<scalar_type, m, 1> flatted_state;
		typedef flatted_state processModel(state &, const input &); //声明一个函数指针
		typedef Eigen::Matrix<scalar_type, m, n> processMatrix1(state &, const input &);
		typedef Eigen::Matrix<scalar_type, m, process_noise_dof> processMatrix2(state &, const input &);
		typedef Eigen::Matrix<scalar_type, process_noise_dof, process_noise_dof> processnoisecovariance;
		typedef measurement measurementModel(state &, bool &);
		typedef measurement measurementModel_share(state &, share_datastruct<state, measurement, measurement_noise_dof> &);
		typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, 1> measurementModel_dyn(state &, bool &);
		// typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, 1> measurementModel_dyn_share(state &,  dyn_share_datastruct<scalar_type> &);
		typedef void measurementModel_dyn_share(state &, dyn_share_datastruct<scalar_type> &);
		typedef Eigen::Matrix<scalar_type, l, n> measurementMatrix1(state &, bool &);
		typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, n> measurementMatrix1_dyn(state &, bool &);
		typedef Eigen::Matrix<scalar_type, l, measurement_noise_dof> measurementMatrix2(state &, bool &);
		typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> measurementMatrix2_dyn(state &, bool &);
		typedef Eigen::Matrix<scalar_type, measurement_noise_dof, measurement_noise_dof> measurementnoisecovariance;
		typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> measurementnoisecovariance_dyn;

		esekf(const state &x = state(),
			  const cov &P = cov::Identity()) : x_(x), P_(P) // esekf初始化，主要是初始化了x_和P_
		{
#ifdef USE_sparse
			SparseMatrix<scalar_type> ref(n, n);
			ref.setIdentity();
			l_ = ref;
			f_x_2 = ref;
			f_x_1 = ref;
#endif
		};

		// receive system-specific models and their differentions.
		// for measurement as a manifold.
		void init(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in, measurementModel h_in, measurementMatrix1 h_x_in, measurementMatrix2 h_v_in, int maximum_iteration, scalar_type limit_vector[n])
		{
			f = f_in;
			f_x = f_x_in;
			f_w = f_w_in;
			h = h_in;
			h_x = h_x_in;
			h_v = h_v_in;

			maximum_iter = maximum_iteration;
			for (int i = 0; i < n; i++)
			{
				limit[i] = limit_vector[i];
			}

			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
		}

		// receive system-specific models and their differentions.
		// for measurement as an Eigen matrix whose dimention is chaing.
		void init_dyn(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in, measurementModel_dyn h_in, measurementMatrix1_dyn h_x_in, measurementMatrix2_dyn h_v_in, int maximum_iteration, scalar_type limit_vector[n])
		{
			f = f_in;
			f_x = f_x_in;
			f_w = f_w_in;
			h_dyn = h_in;
			h_x_dyn = h_x_in;
			h_v_dyn = h_v_in;

			maximum_iter = maximum_iteration;
			for (int i = 0; i < n; i++)
			{
				limit[i] = limit_vector[i];
			}

			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
		}

		// receive system-specific models and their differentions.
		// for measurement as a dynamic manifold whose dimension or type is changing.
		void init_dyn_runtime(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in, measurementMatrix1_dyn h_x_in, measurementMatrix2_dyn h_v_in, int maximum_iteration, scalar_type limit_vector[n])
		{
			f = f_in;
			f_x = f_x_in;
			f_w = f_w_in;
			h_x_dyn = h_x_in;
			h_v_dyn = h_v_in;

			maximum_iter = maximum_iteration;
			for (int i = 0; i < n; i++)
			{
				limit[i] = limit_vector[i];
			}

			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
		}

		// receive system-specific models and their differentions
		// for measurement as a manifold.
		// calculate  measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function (h_share_in).
		void init_share(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in, measurementModel_share h_share_in, int maximum_iteration, scalar_type limit_vector[n])
		{
			f = f_in;
			f_x = f_x_in;
			f_w = f_w_in;
			h_share = h_share_in;

			maximum_iter = maximum_iteration;
			for (int i = 0; i < n; i++)
			{
				limit[i] = limit_vector[i];
			}

			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
		}

		//接收系统特定的模型及其差异
		//作为特征矩阵的测量，其维数是变化的。
		//通过一个函数(h_dyn_share_in)完成了测量(z)，估计测量(h)，偏微分矩阵(h_x, h_v)和噪声协方差(R)的同时计算。
		void init_dyn_share(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in, measurementModel_dyn_share h_dyn_share_in, int maximum_iteration, scalar_type limit_vector[n])
		{
			f = f_in;
			f_x = f_x_in;
			f_w = f_w_in;
			h_dyn_share = h_dyn_share_in;

			maximum_iter = maximum_iteration;
			for (int i = 0; i < n; i++)
			{
				limit[i] = limit_vector[i];
			}

			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
		}

		// receive system-specific models and their differentions
		// for measurement as a dynamic manifold whose dimension  or type is changing.
		// calculate  measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function (h_dyn_share_in).
		// for any scenarios where it is needed
		void init_dyn_runtime_share(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in, int maximum_iteration, scalar_type limit_vector[n])
		{
			f = f_in;
			f_x = f_x_in;
			f_w = f_w_in;

			maximum_iter = maximum_iteration;
			for (int i = 0; i < n; i++)
			{
				limit[i] = limit_vector[i];
			}

			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
		}

		// 迭代误差状态EKF传播
		void predict(double &dt, processnoisecovariance &Q, const input &i_in) //这里的参数均从use-ikfom中传入
		{
			// f函数对应use-ikfom.hpp中的 get_f函数，对应fast_lio2论文公式(2)
			flatted_state f_ = f(x_, i_in); // m*1
			// 对应use-ikfom.hpp中的 df_dx函数
			// 对应fast_lio2论文公式(7)
			cov_ f_x_ = f_x(x_, i_in); // m*n
			cov f_x_final;			   // n*n
			// 对应use-ikfom.hpp中的 df_dw函数
			// 对应fast_lio2论文公式(7)
			Matrix<scalar_type, m, process_noise_dof> f_w_ = f_w(x_, i_in);
			Matrix<scalar_type, n, process_noise_dof> f_w_final;
			state x_before = x_; //保存x_的值，用于后面的更新

			// 对应fast_lio2论文公式(2)
			x_.oplus(f_, dt); //这个是公式2的整个函数，用于更新x的

			F_x1 = cov::Identity(); //状态转移矩阵
			// 更新f_x和f_w
			for (std::vector<std::pair<std::pair<int, int>, int>>::iterator it = x_.vect_state.begin(); it != x_.vect_state.end(); it++)
			{
				int idx = (*it).first.first;  //状态变量的索引
				int dim = (*it).first.second; //状态变量的维数
				int dof = (*it).second;		  //状态变量的自由度
				for (int i = 0; i < n; i++)
				{
					for (int j = 0; j < dof; j++)
					{
						f_x_final(idx + j, i) = f_x_(dim + j, i); //更新f_x_final，形成n*n阵，用于更新
					}
				}
				for (int i = 0; i < process_noise_dof; i++)
				{
					for (int j = 0; j < dof; j++)
					{
						f_w_final(idx + j, i) = f_w_(dim + j, i); //更新f_w_final，形成n*dof，用于更新
					}
				}
			}
			Matrix<scalar_type, 3, 3> res_temp_SO3;
			MTK::vect<3, scalar_type> seg_SO3;
			for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
			{
				int idx = (*it).first;	//状态变量的索引
				int dim = (*it).second; //状态变量的维数
				for (int i = 0; i < 3; i++)
				{
					seg_SO3(i) = -1 * f_(dim + i) * dt; //拿到S03更新值
				}
				MTK::SO3<scalar_type> res;													//残差计算
				res.w() = MTK::exp<scalar_type, 3>(res.vec(), seg_SO3, scalar_type(1 / 2)); //残差计算
#ifdef USE_sparse
				res_temp_SO3 = res.toRotationMatrix();
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						f_x_1.coeffRef(idx + i, idx + j) = res_temp_SO3(i, j);
					}
				}
#else
				F_x1.template block<3, 3>(idx, idx) = res.toRotationMatrix(); //更新f_x_1
#endif
				res_temp_SO3 = MTK::A_matrix(seg_SO3); //转为矩阵形式
				for (int i = 0; i < n; i++)
				{
					f_x_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_x_.template block<3, 1>(dim, i)); // F矩阵
				}
				for (int i = 0; i < process_noise_dof; i++)
				{
					f_w_final.template block<3, 1>(idx, i) = res_temp_SO3 * (f_w_.template block<3, 1>(dim, i));
				}
			}

			Matrix<scalar_type, 2, 3> res_temp_S2;
			Matrix<scalar_type, 2, 2> res_temp_S2_;
			MTK::vect<3, scalar_type> seg_S2;
			for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
			{
				int idx = (*it).first;
				int dim = (*it).second;
				for (int i = 0; i < 3; i++)
				{
					seg_S2(i) = f_(dim + i) * dt;
				}
				MTK::vect<2, scalar_type> vec = MTK::vect<2, scalar_type>::Zero();
				MTK::SO3<scalar_type> res;
				res.w() = MTK::exp<scalar_type, 3>(res.vec(), seg_S2, scalar_type(1 / 2));
				Eigen::Matrix<scalar_type, 2, 3> Nx;
				Eigen::Matrix<scalar_type, 3, 2> Mx;
				x_.S2_Nx_yy(Nx, idx);
				x_before.S2_Mx(Mx, vec, idx);
#ifdef USE_sparse
				res_temp_S2_ = Nx * res.toRotationMatrix() * Mx;
				for (int i = 0; i < 2; i++)
				{
					for (int j = 0; j < 2; j++)
					{
						f_x_1.coeffRef(idx + i, idx + j) = res_temp_S2_(i, j);
					}
				}
#else
				F_x1.template block<2, 2>(idx, idx) = Nx * res.toRotationMatrix() * Mx;
#endif

				Eigen::Matrix<scalar_type, 3, 3> x_before_hat;
				x_before.S2_hat(x_before_hat, idx);
				res_temp_S2 = -Nx * res.toRotationMatrix() * x_before_hat * MTK::A_matrix(seg_S2).transpose();

				for (int i = 0; i < n; i++)
				{
					f_x_final.template block<2, 1>(idx, i) = res_temp_S2 * (f_x_.template block<3, 1>(dim, i));
				}
				for (int i = 0; i < process_noise_dof; i++)
				{
					f_w_final.template block<2, 1>(idx, i) = res_temp_S2 * (f_w_.template block<3, 1>(dim, i));
				}
			}

#ifdef USE_sparse
			f_x_1.makeCompressed();
			spMt f_x2 = f_x_final.sparseView();
			spMt f_w1 = f_w_final.sparseView();
			spMt xp = f_x_1 + f_x2 * dt;
			P_ = xp * P_ * xp.transpose() + (f_w1 * dt) * Q * (f_w1 * dt).transpose();
#else
			F_x1 += f_x_final * dt;
			P_ = (F_x1)*P_ * (F_x1).transpose() + (dt * f_w_final) * Q * (dt * f_w_final).transpose();
#endif
		}

		// iterated error state EKF update for measurement as a manifold.
		void update_iterated(measurement &z, measurementnoisecovariance &R)
		{

			if (!(is_same<typename measurement::scalar, scalar_type>()))
			{
				std::cerr << "the scalar type of measurment must be the same as the state" << std::endl;
				std::exit(100);
			}
			int t = 0;
			bool converg = true;
			bool valid = true;
			state x_propagated = x_;
			cov P_propagated = P_;

			for (int i = -1; i < maximum_iter; i++)
			{
				vectorized_state dx, dx_new;
				x_.boxminus(dx, x_propagated);
				dx_new = dx;
#ifdef USE_sparse
				spMt h_x_ = h_x(x_, valid).sparseView();
				spMt h_v_ = h_v(x_, valid).sparseView();
				spMt R_ = R.sparseView();
#else
				Matrix<scalar_type, l, n> h_x_ = h_x(x_, valid);
				Matrix<scalar_type, l, Eigen::Dynamic> h_v_ = h_v(x_, valid);
#endif
				if (!valid)
				{
					continue;
				}

				P_ = P_propagated;

				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 3; i++)
					{
						seg_SO3(i) = dx(idx + i);
					}

					res_temp_SO3 = A_matrix(seg_SO3).transpose();
					dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx.template block<3, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}

				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 2; i++)
					{
						seg_S2(i) = dx(idx + i);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx;
					dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx.template block<2, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}

				Matrix<scalar_type, n, l> K_;
				if (n > l)
				{
#ifdef USE_sparse
					Matrix<scalar_type, l, l> K_temp = h_x_ * P_ * h_x_.transpose();
					spMt R_temp = h_v_ * R_ * h_v_.transpose();
					K_temp += R_temp;
					K_ = P_ * h_x_.transpose() * K_temp.inverse();
#else
					K_ = P_ * h_x_.transpose() * (h_x_ * P_ * h_x_.transpose() + h_v_ * R * h_v_.transpose()).inverse();
#endif
				}
				else
				{
#ifdef USE_sparse
					measurementnoisecovariance b = measurementnoisecovariance::Identity();
					Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(R_);
					measurementnoisecovariance R_in_temp = solver.solve(b);
					spMt R_in = R_in_temp.sparseView();
					spMt K_temp = h_x_.transpose() * R_in * h_x_;
					cov P_temp = P_.inverse();
					P_temp += K_temp;
					K_ = P_temp.inverse() * h_x_.transpose() * R_in;
#else
					measurementnoisecovariance R_in = (h_v_ * R * h_v_.transpose()).inverse();
					K_ = (h_x_.transpose() * R_in * h_x_ + P_.inverse()).inverse() * h_x_.transpose() * R_in;
#endif
				}
				Matrix<scalar_type, l, 1> innovation;
				z.boxminus(innovation, h(x_, valid));
				cov K_x = K_ * h_x_;
				Matrix<scalar_type, n, 1> dx_ = K_ * innovation + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
				state x_before = x_;
				x_.boxplus(dx_);

				converg = true;
				for (int i = 0; i < n; i++)
				{
					if (std::fabs(dx_[i]) > limit[i])
					{
						converg = false;
						break;
					}
				}

				if (converg)
					t++;

				if (t > 1 || i == maximum_iter - 1)
				{
					L_ = P_;

					Matrix<scalar_type, 3, 3> res_temp_SO3;
					MTK::vect<3, scalar_type> seg_SO3;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
					{
						int idx = (*it).first;
						for (int i = 0; i < 3; i++)
						{
							seg_SO3(i) = dx_(i + idx);
						}
						res_temp_SO3 = A_matrix(seg_SO3).transpose();
						for (int i = 0; i < n; i++)
						{
							L_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
						}
						if (n > l)
						{
							for (int i = 0; i < l; i++)
							{
								K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_.template block<3, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<3, 1>(idx, i) = res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 3>(i, idx) = (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
							P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						}
					}

					Matrix<scalar_type, 2, 2> res_temp_S2;
					MTK::vect<2, scalar_type> seg_S2;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
					{
						int idx = (*it).first;

						for (int i = 0; i < 2; i++)
						{
							seg_S2(i) = dx_(i + idx);
						}

						Eigen::Matrix<scalar_type, 2, 3> Nx;
						Eigen::Matrix<scalar_type, 3, 2> Mx;
						x_.S2_Nx_yy(Nx, idx);
						x_propagated.S2_Mx(Mx, seg_S2, idx);
						res_temp_S2 = Nx * Mx;

						for (int i = 0; i < n; i++)
						{
							L_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
						}
						if (n > l)
						{
							for (int i = 0; i < l; i++)
							{
								K_.template block<2, 1>(idx, i) = res_temp_S2 * (K_.template block<2, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<2, 1>(idx, i) = res_temp_S2 * (K_x.template block<2, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 2>(i, idx) = (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
							P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						}
					}
					if (n > l)
					{
						P_ = L_ - K_ * h_x_ * P_;
					}
					else
					{
						P_ = L_ - K_x * P_;
					}
					return;
				}
			}
		}

		// iterated error state EKF update for measurement as a manifold.
		// calculate measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function.
		void update_iterated_share()
		{

			if (!(is_same<typename measurement::scalar, scalar_type>()))
			{
				std::cerr << "the scalar type of measurment must be the same as the state" << std::endl;
				std::exit(100);
			}

			int t = 0;
			share_datastruct<state, measurement, measurement_noise_dof> _share;
			_share.valid = true;
			_share.converge = true;
			state x_propagated = x_;
			cov P_propagated = P_;

			for (int i = -1; i < maximum_iter; i++)
			{
				vectorized_state dx, dx_new;
				x_.boxminus(dx, x_propagated);
				dx_new = dx;
				measurement h = h_share(x_, _share);
				measurement z = _share.z;
				measurementnoisecovariance R = _share.R;
#ifdef USE_sparse
				spMt h_x_ = _share.h_x.sparseView();
				spMt h_v_ = _share.h_v.sparseView();
				spMt R_ = _share.R.sparseView();
#else
				Matrix<scalar_type, l, n> h_x_ = _share.h_x;
				Matrix<scalar_type, l, Eigen::Dynamic> h_v_ = _share.h_v;
#endif
				if (!_share.valid)
				{
					continue;
				}

				P_ = P_propagated;

				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 3; i++)
					{
						seg_SO3(i) = dx(idx + i);
					}

					res_temp_SO3 = A_matrix(seg_SO3).transpose();
					dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx.template block<3, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}

				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 2; i++)
					{
						seg_S2(i) = dx(idx + i);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx;
					dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx.template block<2, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}

				Matrix<scalar_type, n, l> K_;
				if (n > l)
				{
#ifdef USE_sparse
					Matrix<scalar_type, l, l> K_temp = h_x_ * P_ * h_x_.transpose();
					spMt R_temp = h_v_ * R_ * h_v_.transpose();
					K_temp += R_temp;
					K_ = P_ * h_x_.transpose() * K_temp.inverse();
#else
					K_ = P_ * h_x_.transpose() * (h_x_ * P_ * h_x_.transpose() + h_v_ * R * h_v_.transpose()).inverse();
#endif
				}
				else
				{
#ifdef USE_sparse
					measurementnoisecovariance b = measurementnoisecovariance::Identity();
					Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(R_);
					measurementnoisecovariance R_in_temp = solver.solve(b);
					spMt R_in = R_in_temp.sparseView();
					spMt K_temp = h_x_.transpose() * R_in * h_x_;
					cov P_temp = P_.inverse();
					P_temp += K_temp;
					K_ = P_temp.inverse() * h_x_.transpose() * R_in;
#else
					measurementnoisecovariance R_in = (h_v_ * R * h_v_.transpose()).inverse();
					K_ = (h_x_.transpose() * R_in * h_x_ + P_.inverse()).inverse() * h_x_.transpose() * R_in;
#endif
				}
				Matrix<scalar_type, l, 1> innovation;
				z.boxminus(innovation, h);
				cov K_x = K_ * h_x_;
				Matrix<scalar_type, n, 1> dx_ = K_ * innovation + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
				state x_before = x_;
				x_.boxplus(dx_);

				_share.converge = true;
				for (int i = 0; i < n; i++)
				{
					if (std::fabs(dx_[i]) > limit[i])
					{
						_share.converge = false;
						break;
					}
				}

				if (_share.converge)
					t++;

				if (t > 1 || i == maximum_iter - 1)
				{
					L_ = P_;

					Matrix<scalar_type, 3, 3> res_temp_SO3;
					MTK::vect<3, scalar_type> seg_SO3;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
					{
						int idx = (*it).first;
						for (int i = 0; i < 3; i++)
						{
							seg_SO3(i) = dx_(i + idx);
						}
						res_temp_SO3 = A_matrix(seg_SO3).transpose();
						for (int i = 0; i < n; i++)
						{
							L_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
						}
						if (n > l)
						{
							for (int i = 0; i < l; i++)
							{
								K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_.template block<3, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<3, 1>(idx, i) = res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 3>(i, idx) = (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
							P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						}
					}

					Matrix<scalar_type, 2, 2> res_temp_S2;
					MTK::vect<2, scalar_type> seg_S2;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
					{
						int idx = (*it).first;

						for (int i = 0; i < 2; i++)
						{
							seg_S2(i) = dx_(i + idx);
						}

						Eigen::Matrix<scalar_type, 2, 3> Nx;
						Eigen::Matrix<scalar_type, 3, 2> Mx;
						x_.S2_Nx_yy(Nx, idx);
						x_propagated.S2_Mx(Mx, seg_S2, idx);
						res_temp_S2 = Nx * Mx;

						for (int i = 0; i < n; i++)
						{
							L_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
						}
						if (n > l)
						{
							for (int i = 0; i < l; i++)
							{
								K_.template block<2, 1>(idx, i) = res_temp_S2 * (K_.template block<2, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<2, 1>(idx, i) = res_temp_S2 * (K_x.template block<2, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 2>(i, idx) = (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
							P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						}
					}
					if (n > l)
					{
						P_ = L_ - K_ * h_x_ * P_;
					}
					else
					{
						P_ = L_ - K_x * P_;
					}
					return;
				}
			}
		}

		// iterated error state EKF update for measurement as an Eigen matrix whose dimension is changing.
		void update_iterated_dyn(Eigen::Matrix<scalar_type, Eigen::Dynamic, 1> z, measurementnoisecovariance_dyn R)
		{

			int t = 0;
			bool valid = true;
			bool converg = true;
			state x_propagated = x_;
			cov P_propagated = P_;
			int dof_Measurement;
			int dof_Measurement_noise = R.rows();
			for (int i = -1; i < maximum_iter; i++)
			{
				valid = true;
#ifdef USE_sparse
				spMt h_x_ = h_x_dyn(x_, valid).sparseView();
				spMt h_v_ = h_v_dyn(x_, valid).sparseView();
				spMt R_ = R.sparseView();
#else
				Matrix<scalar_type, Eigen::Dynamic, n> h_x_ = h_x_dyn(x_, valid);
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_v_ = h_v_dyn(x_, valid);
#endif
				Matrix<scalar_type, Eigen::Dynamic, 1> h_ = h_dyn(x_, valid);
				dof_Measurement = h_.rows();
				vectorized_state dx, dx_new;
				x_.boxminus(dx, x_propagated);
				dx_new = dx;
				if (!valid)
				{
					continue;
				}

				P_ = P_propagated;
				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 3; i++)
					{
						seg_SO3(i) = dx(idx + i);
					}

					res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
					dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}

				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 2; i++)
					{
						seg_S2(i) = dx(idx + i);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx;
					dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx_new.template block<2, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}

				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_;
				if (n > dof_Measurement)
				{
#ifdef USE_sparse
					Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_temp = h_x_ * P_ * h_x_.transpose();
					spMt R_temp = h_v_ * R_ * h_v_.transpose();
					K_temp += R_temp;
					K_ = P_ * h_x_.transpose() * K_temp.inverse();
#else
					K_ = P_ * h_x_.transpose() * (h_x_ * P_ * h_x_.transpose() + h_v_ * R * h_v_.transpose()).inverse();
#endif
				}
				else
				{
#ifdef USE_sparse
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> b = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Identity(dof_Measurement_noise, dof_Measurement_noise);
					Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(R_);
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in_temp = solver.solve(b);
					spMt R_in = R_in_temp.sparseView();
					spMt K_temp = h_x_.transpose() * R_in * h_x_;
					cov P_temp = P_.inverse();
					P_temp += K_temp;
					K_ = P_temp.inverse() * h_x_.transpose() * R_in;
#else
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in = (h_v_ * R * h_v_.transpose()).inverse();
					K_ = (h_x_.transpose() * R_in * h_x_ + P_.inverse()).inverse() * h_x_.transpose() * R_in;
#endif
				}
				cov K_x = K_ * h_x_;
				Matrix<scalar_type, n, 1> dx_ = K_ * (z - h_) + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
				state x_before = x_;
				x_.boxplus(dx_);
				converg = true;
				for (int i = 0; i < n; i++)
				{
					if (std::fabs(dx_[i]) > limit[i])
					{
						converg = false;
						break;
					}
				}
				if (converg)
					t++;
				if (t > 1 || i == maximum_iter - 1)
				{
					L_ = P_;
					std::cout << "iteration time:" << t << "," << i << std::endl;

					Matrix<scalar_type, 3, 3> res_temp_SO3;
					MTK::vect<3, scalar_type> seg_SO3;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
					{
						int idx = (*it).first;
						for (int i = 0; i < 3; i++)
						{
							seg_SO3(i) = dx_(i + idx);
						}
						res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
						for (int i = 0; i < n; i++)
						{
							L_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_.template block<3, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<3, 1>(idx, i) = res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 3>(i, idx) = (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
							P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						}
					}

					Matrix<scalar_type, 2, 2> res_temp_S2;
					MTK::vect<2, scalar_type> seg_S2;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
					{
						int idx = (*it).first;

						for (int i = 0; i < 2; i++)
						{
							seg_S2(i) = dx_(i + idx);
						}

						Eigen::Matrix<scalar_type, 2, 3> Nx;
						Eigen::Matrix<scalar_type, 3, 2> Mx;
						x_.S2_Nx_yy(Nx, idx);
						x_propagated.S2_Mx(Mx, seg_S2, idx);
						res_temp_S2 = Nx * Mx;

						for (int i = 0; i < n; i++)
						{
							L_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<2, 1>(idx, i) = res_temp_S2 * (K_.template block<2, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<2, 1>(idx, i) = res_temp_S2 * (K_x.template block<2, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 2>(i, idx) = (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
							P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						}
					}
					if (n > dof_Measurement)
					{
						P_ = L_ - K_ * h_x_ * P_;
					}
					else
					{
						P_ = L_ - K_x * P_;
					}
					return;
				}
			}
		}
		// iterated error state EKF update for measurement as an Eigen matrix whose dimension is changing.
		// calculate measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function.
		void update_iterated_dyn_share()
		{

			int t = 0;
			dyn_share_datastruct<scalar_type> dyn_share;
			dyn_share.valid = true;
			dyn_share.converge = true;
			state x_propagated = x_;
			cov P_propagated = P_;
			int dof_Measurement;
			int dof_Measurement_noise;
			for (int i = -1; i < maximum_iter; i++)
			{
				dyn_share.valid = true;
				h_dyn_share(x_, dyn_share);
				// Matrix<scalar_type, Eigen::Dynamic, 1> h = h_dyn_share (x_,  dyn_share);
				Matrix<scalar_type, Eigen::Dynamic, 1> z = dyn_share.z;
				Matrix<scalar_type, Eigen::Dynamic, 1> h = dyn_share.h;
#ifdef USE_sparse
				spMt h_x = dyn_share.h_x.sparseView();
				spMt h_v = dyn_share.h_v.sparseView();
				spMt R_ = dyn_share.R.sparseView();
#else
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R = dyn_share.R;
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x = dyn_share.h_x;
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_v = dyn_share.h_v;
#endif
				dof_Measurement = h_x.rows();
				dof_Measurement_noise = dyn_share.R.rows();
				vectorized_state dx, dx_new;
				x_.boxminus(dx, x_propagated);
				dx_new = dx;
				if (!(dyn_share.valid))
				{
					continue;
				}

				P_ = P_propagated;
				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 3; i++)
					{
						seg_SO3(i) = dx(idx + i);
					}

					res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
					dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}

				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 2; i++)
					{
						seg_S2(i) = dx(idx + i);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx;
					dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx_new.template block<2, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}

				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_;
				if (n > dof_Measurement)
				{
#ifdef USE_sparse
					Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_temp = h_x * P_ * h_x.transpose();
					spMt R_temp = h_v * R_ * h_v.transpose();
					K_temp += R_temp;
					K_ = P_ * h_x.transpose() * K_temp.inverse();
#else
					K_ = P_ * h_x.transpose() * (h_x * P_ * h_x.transpose() + h_v * R * h_v.transpose()).inverse();
#endif
				}
				else
				{
#ifdef USE_sparse
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> b = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Identity(dof_Measurement_noise, dof_Measurement_noise);
					Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(R_);
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in_temp = solver.solve(b);
					spMt R_in = R_in_temp.sparseView();
					spMt K_temp = h_x.transpose() * R_in * h_x;
					cov P_temp = P_.inverse();
					P_temp += K_temp;
					K_ = P_temp.inverse() * h_x.transpose() * R_in;
#else
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in = (h_v * R * h_v.transpose()).inverse();
					K_ = (h_x.transpose() * R_in * h_x + P_.inverse()).inverse() * h_x.transpose() * R_in;
#endif
				}

				cov K_x = K_ * h_x;
				Matrix<scalar_type, n, 1> dx_ = K_ * (z - h) + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
				state x_before = x_;
				x_.boxplus(dx_);
				dyn_share.converge = true;
				for (int i = 0; i < n; i++)
				{
					if (std::fabs(dx_[i]) > limit[i])
					{
						dyn_share.converge = false;
						break;
					}
				}
				if (dyn_share.converge)
					t++;
				if (t > 1 || i == maximum_iter - 1)
				{
					L_ = P_;
					std::cout << "iteration time:" << t << "," << i << std::endl;

					Matrix<scalar_type, 3, 3> res_temp_SO3;
					MTK::vect<3, scalar_type> seg_SO3;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
					{
						int idx = (*it).first;
						for (int i = 0; i < 3; i++)
						{
							seg_SO3(i) = dx_(i + idx);
						}
						res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
						for (int i = 0; i < int(n); i++)
						{
							L_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_.template block<3, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<3, 1>(idx, i) = res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 3>(i, idx) = (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
							P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						}
					}

					Matrix<scalar_type, 2, 2> res_temp_S2;
					MTK::vect<2, scalar_type> seg_S2;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
					{
						int idx = (*it).first;

						for (int i = 0; i < 2; i++)
						{
							seg_S2(i) = dx_(i + idx);
						}

						Eigen::Matrix<scalar_type, 2, 3> Nx;
						Eigen::Matrix<scalar_type, 3, 2> Mx;
						x_.S2_Nx_yy(Nx, idx);
						x_propagated.S2_Mx(Mx, seg_S2, idx);
						res_temp_S2 = Nx * Mx;

						for (int i = 0; i < n; i++)
						{
							L_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<2, 1>(idx, i) = res_temp_S2 * (K_.template block<2, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<2, 1>(idx, i) = res_temp_S2 * (K_x.template block<2, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 2>(i, idx) = (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
							P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						}
					}
					if (n > dof_Measurement)
					{
						P_ = L_ - K_ * h_x * P_;
					}
					else
					{
						P_ = L_ - K_x * P_;
					}
					return;
				}
			}
		}

		// iterated error state EKF update for measurement as a dynamic manifold, whose dimension or type is changing.
		// the measurement and the measurement model are received in a dynamic manner.
		template <typename measurement_runtime, typename measurementModel_runtime>
		void update_iterated_dyn_runtime(measurement_runtime z, measurementnoisecovariance_dyn R, measurementModel_runtime h_runtime)
		{

			int t = 0;
			bool valid = true;
			bool converg = true;
			state x_propagated = x_;
			cov P_propagated = P_;
			int dof_Measurement;
			int dof_Measurement_noise;
			for (int i = -1; i < maximum_iter; i++)
			{
				valid = true;
#ifdef USE_sparse
				spMt h_x_ = h_x_dyn(x_, valid).sparseView();
				spMt h_v_ = h_v_dyn(x_, valid).sparseView();
				spMt R_ = R.sparseView();
#else
				Matrix<scalar_type, Eigen::Dynamic, n> h_x_ = h_x_dyn(x_, valid);
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_v_ = h_v_dyn(x_, valid);
#endif
				measurement_runtime h_ = h_runtime(x_, valid);
				dof_Measurement = measurement_runtime::DOF;
				dof_Measurement_noise = R.rows();
				vectorized_state dx, dx_new;
				x_.boxminus(dx, x_propagated);
				dx_new = dx;
				if (!valid)
				{
					continue;
				}

				P_ = P_propagated;
				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 3; i++)
					{
						seg_SO3(i) = dx(idx + i);
					}

					res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
					dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}

				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 2; i++)
					{
						seg_S2(i) = dx(idx + i);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx;
					dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx_new.template block<2, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}

				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_;
				if (n > dof_Measurement)
				{
#ifdef USE_sparse
					Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_temp = h_x_ * P_ * h_x_.transpose();
					spMt R_temp = h_v_ * R_ * h_v_.transpose();
					K_temp += R_temp;
					K_ = P_ * h_x_.transpose() * K_temp.inverse();
#else
					K_ = P_ * h_x_.transpose() * (h_x_ * P_ * h_x_.transpose() + h_v_ * R * h_v_.transpose()).inverse();
#endif
				}
				else
				{
#ifdef USE_sparse
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> b = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Identity(dof_Measurement_noise, dof_Measurement_noise);
					Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(R_);
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in_temp = solver.solve(b);
					spMt R_in = R_in_temp.sparseView();
					spMt K_temp = h_x_.transpose() * R_in * h_x_;
					cov P_temp = P_.inverse();
					P_temp += K_temp;
					K_ = P_temp.inverse() * h_x_.transpose() * R_in;
#else
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in = (h_v_ * R * h_v_.transpose()).inverse();
					K_ = (h_x_.transpose() * R_in * h_x_ + P_.inverse()).inverse() * h_x_.transpose() * R_in;
#endif
				}
				cov K_x = K_ * h_x_;
				Eigen::Matrix<scalar_type, measurement_runtime::DOF, 1> innovation;
				z.boxminus(innovation, h_);
				Matrix<scalar_type, n, 1> dx_ = K_ * innovation + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
				state x_before = x_;
				x_.boxplus(dx_);
				converg = true;
				for (int i = 0; i < n; i++)
				{
					if (std::fabs(dx_[i]) > limit[i])
					{
						converg = false;
						break;
					}
				}
				if (converg)
					t++;
				if (t > 1 || i == maximum_iter - 1)
				{
					L_ = P_;
					std::cout << "iteration time:" << t << "," << i << std::endl;

					Matrix<scalar_type, 3, 3> res_temp_SO3;
					MTK::vect<3, scalar_type> seg_SO3;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
					{
						int idx = (*it).first;
						for (int i = 0; i < 3; i++)
						{
							seg_SO3(i) = dx_(i + idx);
						}
						res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
						for (int i = 0; i < n; i++)
						{
							L_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_.template block<3, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<3, 1>(idx, i) = res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 3>(i, idx) = (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
							P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						}
					}

					Matrix<scalar_type, 2, 2> res_temp_S2;
					MTK::vect<2, scalar_type> seg_S2;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
					{
						int idx = (*it).first;

						for (int i = 0; i < 2; i++)
						{
							seg_S2(i) = dx_(i + idx);
						}

						Eigen::Matrix<scalar_type, 2, 3> Nx;
						Eigen::Matrix<scalar_type, 3, 2> Mx;
						x_.S2_Nx_yy(Nx, idx);
						x_propagated.S2_Mx(Mx, seg_S2, idx);
						res_temp_S2 = Nx * Mx;

						for (int i = 0; i < n; i++)
						{
							L_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<2, 1>(idx, i) = res_temp_S2 * (K_.template block<2, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<2, 1>(idx, i) = res_temp_S2 * (K_x.template block<2, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 2>(i, idx) = (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
							P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						}
					}
					if (n > dof_Measurement)
					{
						P_ = L_ - K_ * h_x_ * P_;
					}
					else
					{
						P_ = L_ - K_x * P_;
					}
					return;
				}
			}
		}

		// iterated error state EKF update for measurement as a dynamic manifold, whose dimension or type is changing.
		// the measurement and the measurement model are received in a dynamic manner.
		// calculate measurement (z), estimate measurement (h), partial differention matrices (h_x, h_v) and the noise covariance (R) at the same time, by only one function.
		template <typename measurement_runtime, typename measurementModel_dyn_runtime_share>
		void update_iterated_dyn_runtime_share(measurement_runtime z, measurementModel_dyn_runtime_share h)
		{

			int t = 0;
			dyn_runtime_share_datastruct<scalar_type> dyn_share;
			dyn_share.valid = true;
			dyn_share.converge = true;
			state x_propagated = x_;
			cov P_propagated = P_;
			int dof_Measurement;
			int dof_Measurement_noise;
			for (int i = -1; i < maximum_iter; i++)
			{
				dyn_share.valid = true;
				measurement_runtime h_ = h(x_, dyn_share);
				// measurement_runtime z = dyn_share.z;
#ifdef USE_sparse
				spMt h_x = dyn_share.h_x.sparseView();
				spMt h_v = dyn_share.h_v.sparseView();
				spMt R_ = dyn_share.R.sparseView();
#else
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R = dyn_share.R;
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x = dyn_share.h_x;
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_v = dyn_share.h_v;
#endif
				dof_Measurement = measurement_runtime::DOF;
				dof_Measurement_noise = dyn_share.R.rows();
				vectorized_state dx, dx_new;
				x_.boxminus(dx, x_propagated);
				dx_new = dx;
				if (!(dyn_share.valid))
				{
					continue;
				}

				P_ = P_propagated;
				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 3; i++)
					{
						seg_SO3(i) = dx(idx + i);
					}

					res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
					dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}

				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 2; i++)
					{
						seg_S2(i) = dx(idx + i);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx;
					dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx_new.template block<2, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}

				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_;
				if (n > dof_Measurement)
				{
#ifdef USE_sparse
					Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_temp = h_x * P_ * h_x.transpose();
					spMt R_temp = h_v * R_ * h_v.transpose();
					K_temp += R_temp;
					K_ = P_ * h_x.transpose() * K_temp.inverse();
#else
					K_ = P_ * h_x.transpose() * (h_x * P_ * h_x.transpose() + h_v * R * h_v.transpose()).inverse();
#endif
				}
				else
				{
#ifdef USE_sparse
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> b = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Identity(dof_Measurement_noise, dof_Measurement_noise);
					Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(R_);
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in_temp = solver.solve(b);
					spMt R_in = R_in_temp.sparseView();
					spMt K_temp = h_x.transpose() * R_in * h_x;
					cov P_temp = P_.inverse();
					P_temp += K_temp;
					K_ = P_temp.inverse() * h_x.transpose() * R_in;
#else
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in = (h_v * R * h_v.transpose()).inverse();
					K_ = (h_x.transpose() * R_in * h_x + P_.inverse()).inverse() * h_x.transpose() * R_in;
#endif
				}
				cov K_x = K_ * h_x;
				Eigen::Matrix<scalar_type, measurement_runtime::DOF, 1> innovation;
				z.boxminus(innovation, h_);
				Matrix<scalar_type, n, 1> dx_ = K_ * innovation + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
				state x_before = x_;
				x_.boxplus(dx_);
				dyn_share.converge = true;
				for (int i = 0; i < n; i++)
				{
					if (std::fabs(dx_[i]) > limit[i])
					{
						dyn_share.converge = false;
						break;
					}
				}
				if (dyn_share.converge)
					t++;
				if (t > 1 || i == maximum_iter - 1)
				{
					L_ = P_;
					std::cout << "iteration time:" << t << "," << i << std::endl;

					Matrix<scalar_type, 3, 3> res_temp_SO3;
					MTK::vect<3, scalar_type> seg_SO3;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
					{
						int idx = (*it).first;
						for (int i = 0; i < 3; i++)
						{
							seg_SO3(i) = dx_(i + idx);
						}
						res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
						for (int i = 0; i < int(n); i++)
						{
							L_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_.template block<3, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<3, 1>(idx, i) = res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 3>(i, idx) = (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
							P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						}
					}

					Matrix<scalar_type, 2, 2> res_temp_S2;
					MTK::vect<2, scalar_type> seg_S2;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
					{
						int idx = (*it).first;

						for (int i = 0; i < 2; i++)
						{
							seg_S2(i) = dx_(i + idx);
						}

						Eigen::Matrix<scalar_type, 2, 3> Nx;
						Eigen::Matrix<scalar_type, 3, 2> Mx;
						x_.S2_Nx_yy(Nx, idx);
						x_propagated.S2_Mx(Mx, seg_S2, idx);
						res_temp_S2 = Nx * Mx;

						for (int i = 0; i < n; i++)
						{
							L_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
						}
						if (n > dof_Measurement)
						{
							for (int i = 0; i < dof_Measurement; i++)
							{
								K_.template block<2, 1>(idx, i) = res_temp_S2 * (K_.template block<2, 1>(idx, i));
							}
						}
						else
						{
							for (int i = 0; i < n; i++)
							{
								K_x.template block<2, 1>(idx, i) = res_temp_S2 * (K_x.template block<2, 1>(idx, i));
							}
						}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 2>(i, idx) = (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
							P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						}
					}
					if (n > dof_Measurement)
					{
						P_ = L_ - K_ * h_x * P_;
					}
					else
					{
						P_ = L_ - K_x * P_;
					}
					return;
				}
			}
		}

		// 迭代错误状态EKF更新修改为一个特定的系统
		void update_iterated_dyn_share_modified(double R, double &solve_time)
		{

			dyn_share_datastruct<scalar_type> dyn_share;
			dyn_share.valid = true;
			dyn_share.converge = true;
			int t = 0;
			//获取上一次的状态和协方差矩阵
			state x_propagated = x_;
			cov P_propagated = P_;
			int dof_Measurement;

			Matrix<scalar_type, n, 1> K_h;
			Matrix<scalar_type, n, n> K_x;

			vectorized_state dx_new = vectorized_state::Zero();
			// 最多进行maximum_iter次迭代优化
			for (int i = -1; i < maximum_iter; i++)
			{
				dyn_share.valid = true;
				// 计算测量模型方程的雅克比，也就是点面残差的导数 H(代码里是h_x)
				h_dyn_share(x_, dyn_share);
				// Matrix<scalar_type, Eigen::Dynamic, 1> h = h_dyn_share(x_, dyn_share);
#ifdef USE_sparse
				spMt h_x_ = dyn_share.h_x.sparseView();
#else
				// 获取测量模型的雅克比d(pos, rot, 0, 0)/dx
				Eigen::Matrix<scalar_type, Eigen::Dynamic, 12> h_x_ = dyn_share.h_x;
#endif
				double solve_start = omp_get_wtime();
				dof_Measurement = h_x_.rows(); // 观测方程个数m
				vectorized_state dx;		   // 定义误差状态
				x_.boxminus(dx, x_propagated); //获取误差dx
				dx_new = dx;				   // 用于迭代的误差状态

				if (!dyn_share.valid)
				{
					continue;
				}
				// 预测得到的误差状态协方差矩阵
				//协方差矩阵在迭代过程中不会代入下一次迭代，直到最后一次退出时更新，在迭代过程中更新的只是先验
				P_ = P_propagated;
				// 这一大段都在求协方差的先验更新，大致上是P=(J^-1)*P*(J^-T)如论文式16~18
				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 3; i++)
					{
						seg_SO3(i) = dx(idx + i);
					}

					res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
					dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}

				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
				{
					int idx = (*it).first;
					int dim = (*it).second;
					for (int i = 0; i < 2; i++)
					{
						seg_S2(i) = dx(idx + i);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx;
					dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx_new.template block<2, 1>(idx, 0);
					for (int i = 0; i < n; i++)
					{
						P_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
					}
					for (int i = 0; i < n; i++)
					{
						P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}
				// Matrix<scalar_type, n, Eigen::Dynamic> K_;
				// Matrix<scalar_type, n, 1> K_h;
				// Matrix<scalar_type, n, n> K_x;

				/*
				if(n > dof_Measurement)
				{
					K_= P_ * h_x_.transpose() * (h_x_ * P_ * h_x_.transpose()/R + Eigen::Matrix<double, Dynamic, Dynamic>::Identity(dof_Measurement, dof_Measurement)).inverse()/R;
				}
				else
				{
					K_= (h_x_.transpose() * h_x_ + (P_/R).inverse()).inverse()*h_x_.transpose();
				}
				*/
				// 状态维度 n > 测量维度 dof_Measurement
				// 如果状态量维度大于观测方程 n > m，不满秩
				if (n > dof_Measurement)
				{
					//#ifdef USE_sparse
					// Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_temp = h_x * P_ * h_x.transpose();
					// spMt R_temp = h_v * R_ * h_v.transpose();
					// K_temp += R_temp;
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
					//每一次迭代将重新计算增益K，即论文式18
					h_x_cur.topLeftCorner(dof_Measurement, 12) = h_x_;
					/*
					h_x_cur.col(0) = h_x_.col(0);
					h_x_cur.col(1) = h_x_.col(1);
					h_x_cur.col(2) = h_x_.col(2);
					h_x_cur.col(3) = h_x_.col(3);
					h_x_cur.col(4) = h_x_.col(4);
					h_x_cur.col(5) = h_x_.col(5);
					h_x_cur.col(6) = h_x_.col(6);
					h_x_cur.col(7) = h_x_.col(7);
					h_x_cur.col(8) = h_x_.col(8);
					h_x_cur.col(9) = h_x_.col(9);
					h_x_cur.col(10) = h_x_.col(10);
					h_x_cur.col(11) = h_x_.col(11);
					*/
					// 重新计算增益矩阵K
					Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_ = P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() / R + Eigen::Matrix<double, Dynamic, Dynamic>::Identity(dof_Measurement, dof_Measurement)).inverse() / R;
					K_h = K_ * dyn_share.h;
					K_x = K_ * h_x_cur;
					//#else
					//	K_= P_ * h_x.transpose() * (h_x * P_ * h_x.transpose() + h_v * R * h_v.transpose()).inverse();
					//#endif
				}
				else
				{
					//避免求逆矩阵，K按稀疏矩阵分解的方法如论文式20
#ifdef USE_sparse
					// Eigen::Matrix<scalar_type, n, n> b = Eigen::Matrix<scalar_type, n, n>::Identity();
					// Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver;
					spMt A = h_x_.transpose() * h_x_;
					cov P_temp = (P_ / R).inverse();
					P_temp.template block<12, 12>(0, 0) += A;
					P_temp = P_temp.inverse();
					/*
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
					h_x_cur.col(0) = h_x_.col(0);
					h_x_cur.col(1) = h_x_.col(1);
					h_x_cur.col(2) = h_x_.col(2);
					h_x_cur.col(3) = h_x_.col(3);
					h_x_cur.col(4) = h_x_.col(4);
					h_x_cur.col(5) = h_x_.col(5);
					h_x_cur.col(6) = h_x_.col(6);
					h_x_cur.col(7) = h_x_.col(7);
					h_x_cur.col(8) = h_x_.col(8);
					h_x_cur.col(9) = h_x_.col(9);
					h_x_cur.col(10) = h_x_.col(10);
					h_x_cur.col(11) = h_x_.col(11);
					*/
					K_ = P_temp.template block<n, 12>(0, 0) * h_x_.transpose();
					K_x = cov::Zero();
					K_x.template block<n, 12>(0, 0) = P_inv.template block<n, 12>(0, 0) * HTH;
					/*
					solver.compute(R_);
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in_temp = solver.solve(b);
					spMt R_in =R_in_temp.sparseView();
					spMt K_temp = h_x.transpose() * R_in * h_x;
					cov P_temp = P_.inverse();
					P_temp += K_temp;
					K_ = P_temp.inverse() * h_x.transpose() * R_in;
					*/
#else
					cov P_temp = (P_ / R).inverse();
					// Eigen::Matrix<scalar_type, 12, Eigen::Dynamic> h_T = h_x_.transpose();
					Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_;
					P_temp.template block<12, 12>(0, 0) += HTH;
					/*
					Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
					//std::cout << "line 1767" << std::endl;
					h_x_cur.col(0) = h_x_.col(0);
					h_x_cur.col(1) = h_x_.col(1);
					h_x_cur.col(2) = h_x_.col(2);
					h_x_cur.col(3) = h_x_.col(3);
					h_x_cur.col(4) = h_x_.col(4);
					h_x_cur.col(5) = h_x_.col(5);
					h_x_cur.col(6) = h_x_.col(6);
					h_x_cur.col(7) = h_x_.col(7);
					h_x_cur.col(8) = h_x_.col(8);
					h_x_cur.col(9) = h_x_.col(9);
					h_x_cur.col(10) = h_x_.col(10);
					h_x_cur.col(11) = h_x_.col(11);
					*/
					cov P_inv = P_temp.inverse();
					// std::cout << "line 1781" << std::endl;
					K_h = P_inv.template block<n, 12>(0, 0) * h_x_.transpose() * dyn_share.h; // (H_T_H + P^-1)^-1 * H^T * h(残差) = K * h
					// std::cout << "line 1780" << std::endl;
					// cov HTH_cur = cov::Zero();
					// HTH_cur. template block<12, 12>(0, 0) = HTH;
					K_x.setZero(); // = cov::Zero();

					K_x.template block<n, 12>(0, 0) = P_inv.template block<n, 12>(0, 0) * HTH; //(H_T_H + P^-1)^-1 * H_T_H = KH

					// K_= (h_x_.transpose() * h_x_ + (P_/R).inverse()).inverse()*h_x_.transpose();
#endif
				}

				// K_x = K_ * h_x_;
				//由于是误差迭代KF，得到的是误差的最优估计！
				Matrix<scalar_type, n, 1> dx_ = K_h + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new; //误差增量后验 K*h + (K*H - I) dx

				state x_before = x_; // 加上校正后的误差状态dx_
				x_.boxplus(dx_);	 //根据计算得到的误差增量后验，更新状态量
				// 判断迭代是否发散
				dyn_share.converge = true;
				//判断已收敛的条件是误差的估计值小于阈值
				for (int i = 0; i < n; i++)
				{
					if (std::fabs(dx_[i]) > limit[i])
					{
						dyn_share.converge = false;
						break;
					}
				}
				if (dyn_share.converge)
					t++;

				if (!t && i == maximum_iter - 2)
				{
					dyn_share.converge = true;
				}
				// 迭代完成后更新误差状态协方差矩阵
				//结束迭代后，更新协方差矩阵的后验值，大致上是P=(I-K*H)*P，如论文式19
				if (t > 1 || i == maximum_iter - 1)
				{
					L_ = P_;
					// std::cout << "iteration time" << t << "," << i << std::endl;
					Matrix<scalar_type, 3, 3> res_temp_SO3;
					MTK::vect<3, scalar_type> seg_SO3;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++)
					{
						int idx = (*it).first;
						for (int i = 0; i < 3; i++)
						{
							seg_SO3(i) = dx_(i + idx);
						}
						res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
						for (int i = 0; i < n; i++)
						{
							L_.template block<3, 1>(idx, i) = res_temp_SO3 * (P_.template block<3, 1>(idx, i));
						}
						// if(n > dof_Measurement)
						// {
						// 	for(int i = 0; i < dof_Measurement; i++){
						// 		K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_. template block<3, 1>(idx, i));
						// 	}
						// }
						// else
						// {
						for (int i = 0; i < 12; i++)
						{
							K_x.template block<3, 1>(idx, i) = res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
						}
						//}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 3>(i, idx) = (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
							P_.template block<1, 3>(i, idx) = (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						}
					}

					Matrix<scalar_type, 2, 2> res_temp_S2;
					MTK::vect<2, scalar_type> seg_S2;
					for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++)
					{
						int idx = (*it).first;

						for (int i = 0; i < 2; i++)
						{
							seg_S2(i) = dx_(i + idx);
						}

						Eigen::Matrix<scalar_type, 2, 3> Nx;
						Eigen::Matrix<scalar_type, 3, 2> Mx;
						x_.S2_Nx_yy(Nx, idx);
						x_propagated.S2_Mx(Mx, seg_S2, idx);
						res_temp_S2 = Nx * Mx;
						for (int i = 0; i < n; i++)
						{
							L_.template block<2, 1>(idx, i) = res_temp_S2 * (P_.template block<2, 1>(idx, i));
						}
						// if(n > dof_Measurement)
						// {
						// 	for(int i = 0; i < dof_Measurement; i++){
						// 		K_. template block<2, 1>(idx, i) = res_temp_S2 * (K_. template block<2, 1>(idx, i));
						// 	}
						// }
						// else
						// {
						for (int i = 0; i < 12; i++)
						{
							K_x.template block<2, 1>(idx, i) = res_temp_S2 * (K_x.template block<2, 1>(idx, i));
						}
						//}
						for (int i = 0; i < n; i++)
						{
							L_.template block<1, 2>(i, idx) = (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
							P_.template block<1, 2>(i, idx) = (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						}
					}

					// if(n > dof_Measurement)
					// {
					// 	Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
					// 	h_x_cur.topLeftCorner(dof_Measurement, 12) = h_x_;
					// 	/*
					// 	h_x_cur.col(0) = h_x_.col(0);
					// 	h_x_cur.col(1) = h_x_.col(1);
					// 	h_x_cur.col(2) = h_x_.col(2);
					// 	h_x_cur.col(3) = h_x_.col(3);
					// 	h_x_cur.col(4) = h_x_.col(4);
					// 	h_x_cur.col(5) = h_x_.col(5);
					// 	h_x_cur.col(6) = h_x_.col(6);
					// 	h_x_cur.col(7) = h_x_.col(7);
					// 	h_x_cur.col(8) = h_x_.col(8);
					// 	h_x_cur.col(9) = h_x_.col(9);
					// 	h_x_cur.col(10) = h_x_.col(10);
					// 	h_x_cur.col(11) = h_x_.col(11);
					// 	*/
					// 	P_ = L_ - K_*h_x_cur * P_;
					// }
					// else
					//{
					P_ = L_ - K_x.template block<n, 12>(0, 0) * P_.template block<12, n>(0, 0);
					//}
					solve_time += omp_get_wtime() - solve_start;
					return;
				}
				solve_time += omp_get_wtime() - solve_start;
			}
		}

		void change_x(state &input_state)
		{
			x_ = input_state;
			if ((!x_.vect_state.size()) && (!x_.SO3_state.size()) && (!x_.S2_state.size()))
			{
				x_.build_S2_state();
				x_.build_SO3_state();
				x_.build_vect_state();
			}
		}
		// 修改P_的状态
		void change_P(cov &input_cov)
		{
			P_ = input_cov;
		}
		// 获得x_的状态
		const state &get_x() const
		{
			return x_;
		}
		const cov &get_P() const
		{
			return P_;
		}

	private:
		state x_;
		measurement m_;
		cov P_;
		spMt l_;
		spMt f_x_1;
		spMt f_x_2;
		cov F_x1 = cov::Identity();
		cov F_x2 = cov::Identity();
		cov L_ = cov::Identity();

		processModel *f;
		processMatrix1 *f_x;
		processMatrix2 *f_w;

		measurementModel *h;
		measurementMatrix1 *h_x;
		measurementMatrix2 *h_v;

		measurementModel_dyn *h_dyn;
		measurementMatrix1_dyn *h_x_dyn;
		measurementMatrix2_dyn *h_v_dyn;

		measurementModel_share *h_share;
		measurementModel_dyn_share *h_dyn_share;

		int maximum_iter = 0;
		scalar_type limit[n];

		template <typename T>
		T check_safe_update(T _temp_vec)
		{
			T temp_vec = _temp_vec;
			if (std::isnan(temp_vec(0, 0)))
			{
				temp_vec.setZero();
				return temp_vec;
			}
			double angular_dis = temp_vec.block(0, 0, 3, 1).norm() * 57.3;
			double pos_dis = temp_vec.block(3, 0, 3, 1).norm();
			if (angular_dis >= 20 || pos_dis > 1)
			{
				printf("Angular dis = %.2f, pos dis = %.2f\r\n", angular_dis, pos_dis);
				temp_vec.setZero();
			}
			return temp_vec;
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

} // namespace esekfom

#endif //  ESEKFOM_EKF_HPP
