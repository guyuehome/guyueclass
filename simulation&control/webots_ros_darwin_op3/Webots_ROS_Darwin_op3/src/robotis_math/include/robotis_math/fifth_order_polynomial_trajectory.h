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
 * fifth_order_polynomial_trajectory.h
 *
 *  Created on: 2016. 8. 24.
 *      Author: Jay Song
 */

#ifndef ROBOTIS_MATH_FIFTH_ORDER_POLYNOMIAL_TRAJECTORY_H_
#define ROBOTIS_MATH_FIFTH_ORDER_POLYNOMIAL_TRAJECTORY_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_linear_algebra.h"
#include "robotis_math_base.h"

namespace robotis_framework
{
class FifthOrderPolynomialTrajectory
{
public:
  FifthOrderPolynomialTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                                 double final_time,   double final_pos,   double final_vel,   double final_acc);
  FifthOrderPolynomialTrajectory();
  ~FifthOrderPolynomialTrajectory();

  bool changeTrajectory(double final_pos,   double final_vel,   double final_acc);
  bool changeTrajectory(double final_time,   double final_pos,   double final_vel,   double final_acc);
  bool changeTrajectory(double initial_time, double initial_pos, double initial_vel, double initial_acc,
                        double final_time,   double final_pos,   double final_vel,   double final_acc);

  double getPosition(double time);
  double getVelocity(double time);
  double getAcceleration(double time);

  void setTime(double time);
  double getPosition();
  double getVelocity();
  double getAcceleration();

  double initial_time_;
  double initial_pos_;
  double initial_vel_;
  double initial_acc_;

  double current_time_;
  double current_pos_;
  double current_vel_;
  double current_acc_;

  double final_time_;
  double final_pos_;
  double final_vel_;
  double final_acc_;

  Eigen::MatrixXd position_coeff_;
  Eigen::MatrixXd velocity_coeff_;
  Eigen::MatrixXd acceleration_coeff_;
  Eigen::MatrixXd time_variables_;
};
}

#endif /* ROBOTIS_MATH_FIFTH_ORDER_POLYNOMIAL_TRAJECTORY_H_ */
