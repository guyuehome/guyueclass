/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * robotis_trajectory_calculator.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef ROBOTIS_MATH_ROBOTIS_TRAJECTORY_CALCULATOR_H_
#define ROBOTIS_MATH_ROBOTIS_TRAJECTORY_CALCULATOR_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_linear_algebra.h"
#include "robotis_math_base.h"
#include "fifth_order_polynomial_trajectory.h"
#include "simple_trapezoidal_velocity_profile.h"


namespace robotis_framework
{
// minimum jerk trajectory
Eigen::MatrixXd calcMinimumJerkTra(double pos_start, double vel_start, double accel_start,
                                   double pos_end,   double vel_end,   double accel_end,
                                   double smp_time,  double mov_time);

Eigen::MatrixXd calcMinimumJerkTraPlus(double pos_start, double vel_start, double accel_start,
                                       double pos_end,   double vel_end,   double accel_end,
                                       double smp_time,  double mov_time);

Eigen::MatrixXd calcMinimumJerkTraWithViaPoints(int via_num,
                                                double pos_start, double vel_start, double accel_start,
                                                Eigen::MatrixXd pos_via,  Eigen::MatrixXd vel_via, Eigen::MatrixXd accel_via,
                                                double pos_end, double vel_end, double accel_end,
                                                double smp_time, Eigen::MatrixXd via_time, double mov_time);

Eigen::MatrixXd calcMinimumJerkTraWithViaPointsPosition(int via_num,
                                                        double pos_start, double vel_start, double accel_start,
                                                        Eigen::MatrixXd pos_via,
                                                        double pos_end, double vel_end, double accel_end,
                                                        double smp_time, Eigen::MatrixXd via_time, double mov_time);

Eigen::MatrixXd calcArc3dTra(double smp_time, double mov_time,
                             Eigen::MatrixXd center_point, Eigen::MatrixXd normal_vector, Eigen::MatrixXd start_point,
                             double rotation_angle, double cross_ratio);

}

#endif /* ROBOTIS_MATH_ROBOTIS_TRAJECTORY_CALCULATOR_H_ */
