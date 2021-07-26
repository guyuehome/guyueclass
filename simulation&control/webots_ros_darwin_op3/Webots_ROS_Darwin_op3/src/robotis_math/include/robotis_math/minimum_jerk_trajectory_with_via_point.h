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

#ifndef ROBOTIS_MATH_MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_
#define ROBOTIS_MATH_MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_linear_algebra.h"
#include "robotis_math_base.h"

#include <ros/ros.h>
#include <stdint.h>
#include <vector>

namespace robotis_framework
{

class MinimumJerkViaPoint
{
public:
  MinimumJerkViaPoint(double ini_time, double fin_time, double via_time, double ratio,
                      std::vector<double_t> ini_pos, std::vector<double_t> ini_vel, std::vector<double_t> ini_acc,
                      std::vector<double_t> fin_pos, std::vector<double_t> fin_vel, std::vector<double_t> fin_acc,
                      std::vector<double_t> via_pos, std::vector<double_t> via_vel, std::vector<double_t> via_acc);
  virtual ~MinimumJerkViaPoint();

  std::vector<double_t> getPosition(double time);
  std::vector<double_t> getVelocity(double time);
  std::vector<double_t> getAcceleration(double time);

  double cur_time_;
  std::vector<double_t> cur_pos_;
  std::vector<double_t> cur_vel_;
  std::vector<double_t> cur_acc_;

  Eigen::MatrixXd position_coeff_;
  Eigen::MatrixXd velocity_coeff_;
  Eigen::MatrixXd acceleration_coeff_;
  Eigen::MatrixXd time_variables_;

private:
  int number_of_joint_;
  double ratio_;
  double input_ini_time_, input_fin_time_;
  double ini_time_, fin_time_, via_time_;
  std::vector<double_t> ini_pos_, ini_vel_, ini_acc_;
  std::vector<double_t> fin_pos_, fin_vel_, fin_acc_;
  std::vector<double_t> via_pos_, via_vel_, via_acc_;
};

}

#endif /* ROBOTIS_MATH_MINIMUM_JERK_TRAJECTORY_WITH_VIA_POINT_H_ */
