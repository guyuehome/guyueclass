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
 * simple_trapezoidal_velocity_profile.h
 *
 *  Created on: 2016. 8. 24.
 *      Author: Jay Song
 */

#ifndef ROBOTIS_MATH_TRAPEZOIDAL_VELOCITY_PROFILE_H_
#define ROBOTIS_MATH_TRAPEZOIDAL_VELOCITY_PROFILE_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_linear_algebra.h"
#include "robotis_math_base.h"

namespace robotis_framework
{
class SimpleTrapezoidalVelocityProfile
{
public:
  SimpleTrapezoidalVelocityProfile();
  ~SimpleTrapezoidalVelocityProfile();

  void setVelocityBaseTrajectory(double init_pos, double final_pos, double acceleration, double max_velocity);
  void setVelocityBaseTrajectory(double init_pos, double final_pos, double acceleration, double deceleration, double max_velocity);
  void setTimeBaseTrajectory(double init_pos, double final_pos, double accel_time, double total_time);
  void setTimeBaseTrajectory(double init_pos, double final_pos, double accel_time, double decel_time, double total_time);

  double getPosition(double time);
  double getVelocity(double time);
  double getAcceleration(double time);

  void setTime(double time);
  double getPosition();
  double getVelocity();
  double getAcceleration();

  double getTotalTime();
  double getConstantVelocitySectionStartTime();
  double getDecelerationSectionStartTime();

private:
  Eigen::MatrixXd pos_coeff_accel_section_;
  Eigen::MatrixXd vel_coeff_accel_section_;

  Eigen::MatrixXd pos_coeff_const_section_;
  Eigen::MatrixXd vel_coeff_const_section_;

  Eigen::MatrixXd pos_coeff_decel_section_;
  Eigen::MatrixXd vel_coeff_decel_section_;

  Eigen::MatrixXd time_variables_;

  double acceleration_;
  double deceleration_;
  double max_velocity_;

  double initial_pos_;

  double current_time_;
  double current_pos_;
  double current_vel_;
  double current_acc_;

  double final_pos_;

  double accel_time_;
  double const_time_;
  double decel_time_;

  double const_start_time_;
  double decel_start_time_;
  double total_time_;
};

}

#endif /* ROBOTIS_MATH_TRAPEZOIDAL_VELOCITY_PROFILE_H_ */
