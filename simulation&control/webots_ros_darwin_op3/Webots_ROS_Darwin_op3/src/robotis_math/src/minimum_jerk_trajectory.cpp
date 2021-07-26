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

#include "robotis_math/minimum_jerk_trajectory.h"

using namespace robotis_framework;

MinimumJerk::MinimumJerk(double ini_time, double fin_time,
                         std::vector<double_t> ini_pos, std::vector<double_t> ini_vel, std::vector<double_t> ini_acc,
                         std::vector<double_t> fin_pos, std::vector<double_t> fin_vel, std::vector<double_t> fin_acc)
{
  ini_time_ = ini_time;
  fin_time_ = fin_time;

  ini_pos_ = ini_pos;
  ini_vel_ = ini_vel;
  ini_acc_ = ini_acc;
  fin_pos_ = fin_pos;
  fin_vel_ = fin_vel;
  fin_acc_ = fin_acc;

  number_of_joint_ = ini_pos.size();

  position_coeff_.resize(6,number_of_joint_);
  velocity_coeff_.resize(6,number_of_joint_);
  acceleration_coeff_.resize(6,number_of_joint_);
  time_variables_.resize(1,6);

  position_coeff_.fill(0.0);
  velocity_coeff_.fill(0.0);
  acceleration_coeff_.fill(0.0);
  time_variables_.fill(0.0);

  if(fin_time_ > ini_time_)
  {
    Eigen::MatrixXd time_mat;
    Eigen::MatrixXd conditions_mat;

    time_mat.resize(6,6);
    time_mat <<      powDI(ini_time_,5),      powDI(ini_time_,4),     powDI(ini_time_,3), powDI(ini_time_,2), ini_time_, 1.0,
                 5.0*powDI(ini_time_,4),  4.0*powDI(ini_time_,3), 3.0*powDI(ini_time_,2),      2.0*ini_time_,       1.0, 0.0,
                20.0*powDI(ini_time_,3), 12.0*powDI(ini_time_,2),          6.0*ini_time_,                2.0,       0.0, 0.0,
                     powDI(fin_time_,5),      powDI(fin_time_,4),     powDI(fin_time_,3), powDI(fin_time_,2), fin_time_, 1.0,
                 5.0*powDI(fin_time_,4),  4.0*powDI(fin_time_,3), 3.0*powDI(fin_time_,2),      2.0*fin_time_,       1.0, 0.0,
                20.0*powDI(fin_time_,3), 12.0*powDI(fin_time_,2),          6.0*fin_time_,                2.0,       0.0, 0.0;

    conditions_mat.resize(6,1);
    conditions_mat.fill(0.0);

    for (int i = 0; i < number_of_joint_; i++)
    {
      conditions_mat.coeffRef(0,0) = ini_pos[i];
      conditions_mat.coeffRef(1,0) = ini_vel[i];
      conditions_mat.coeffRef(2,0) = ini_acc[i];
      conditions_mat.coeffRef(3,0) = fin_pos[i];
      conditions_mat.coeffRef(4,0) = fin_vel[i];
      conditions_mat.coeffRef(5,0) = fin_acc[i];

      Eigen::MatrixXd position_coeff = time_mat.inverse() * conditions_mat;

      position_coeff_.block(0,i,6,1) = position_coeff;

      velocity_coeff_.coeffRef(0,i) = 0.0;
      velocity_coeff_.coeffRef(1,i) = 5.0*position_coeff.coeff(0,0);
      velocity_coeff_.coeffRef(2,i) = 4.0*position_coeff.coeff(1,0);
      velocity_coeff_.coeffRef(3,i) = 3.0*position_coeff.coeff(2,0);
      velocity_coeff_.coeffRef(4,i) = 2.0*position_coeff.coeff(3,0);
      velocity_coeff_.coeffRef(5,i) = position_coeff.coeff(4,0);

      acceleration_coeff_.coeffRef(0,i) = 0.0;
      acceleration_coeff_.coeffRef(1,i) = 0.0;
      acceleration_coeff_.coeffRef(2,i) = 20.0*position_coeff.coeff(0,0);
      acceleration_coeff_.coeffRef(3,i) = 12.0*position_coeff.coeff(1,0);
      acceleration_coeff_.coeffRef(4,i) = 6.0*position_coeff.coeff(2,0);
      acceleration_coeff_.coeffRef(5,i) = 2.0*position_coeff.coeff(3,0);
    }
  }

  cur_time_ = 0.0;
  cur_pos_.resize(number_of_joint_);
  cur_vel_.resize(number_of_joint_);
  cur_acc_.resize(number_of_joint_);
}

MinimumJerk::~MinimumJerk()
{

}

std::vector<double_t> MinimumJerk::getPosition(double time)
{
  if(time >= fin_time_)
    cur_pos_  = fin_pos_;
  else if(time <= ini_time_)
    cur_pos_  = ini_pos_;
  else
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_pos = time_variables_ * position_coeff_.block(0,i,6,1);
      cur_pos_[i] = cur_pos.coeff(0,0);
    }
  }
  return cur_pos_;
}

std::vector<double_t> MinimumJerk::getVelocity(double time)
{
  if(time >= fin_time_)
    cur_vel_  = fin_vel_;
  else if(time <= ini_time_)
    cur_vel_  = ini_vel_;
  else
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_vel = time_variables_ * velocity_coeff_.block(0,i,6,1);
      cur_vel_[i] = cur_vel.coeff(0,0);
    }
  }
  return cur_vel_;
}

std::vector<double_t> MinimumJerk::getAcceleration(double time)
{
  if(time >= fin_time_)
    cur_acc_  = fin_acc_;
  else if(time <= ini_time_)
    cur_acc_  = ini_acc_;
  else
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_acc = time_variables_ * acceleration_coeff_.block(0,i,6,1);
      cur_acc_[i] = cur_acc.coeff(0,0);
    }
  }
  return cur_acc_;
}
