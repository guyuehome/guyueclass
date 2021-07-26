/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "robotis_math/minimum_jerk_trajectory_with_via_point.h"

using namespace robotis_framework;

MinimumJerkViaPoint::MinimumJerkViaPoint(double ini_time, double fin_time, double via_time, double ratio,
                                         std::vector<double_t> ini_pos, std::vector<double_t> ini_vel, std::vector<double_t> ini_acc,
                                         std::vector<double_t> fin_pos, std::vector<double_t> fin_vel, std::vector<double_t> fin_acc,
                                         std::vector<double_t> via_pos, std::vector<double_t> via_vel, std::vector<double_t> via_acc)
{
  ratio_ = ratio;

//  input_ini_time_ = ini_time;
//  input_fin_time_ = fin_time;
  via_time_ = via_time;

  double time_size = fin_time - ini_time;
  double time_ratio = 0.5*ratio_*time_size;

  ini_time_ = ini_time + time_ratio;
  fin_time_ = fin_time - time_ratio;

  ini_pos_ = ini_pos;
  ini_vel_ = ini_vel;
  ini_acc_ = ini_acc;
  fin_pos_ = fin_pos;
  fin_vel_ = fin_vel;
  fin_acc_ = fin_acc;
  via_pos_ = via_pos;
//  via_vel_ = via_vel;
//  via_acc_ = via_acc;

  number_of_joint_ = ini_pos.size();

  position_coeff_.resize(7,number_of_joint_);
  velocity_coeff_.resize(7,number_of_joint_);
  acceleration_coeff_.resize(7,number_of_joint_);
  time_variables_.resize(1,7);

  position_coeff_.fill(0.0);
  velocity_coeff_.fill(0.0);
  acceleration_coeff_.fill(0.0);
  time_variables_.fill(0.0);

  if(fin_time_ > ini_time_)
  {
    Eigen::MatrixXd time_mat;
    Eigen::MatrixXd conditions_mat;

    time_mat.resize(7,7);
    time_mat <<      powDI(ini_time_,5),      powDI(ini_time_,4),     powDI(ini_time_,3), powDI(ini_time_,2), ini_time_, 1.0,                                  0.0,
                 5.0*powDI(ini_time_,4),  4.0*powDI(ini_time_,3), 3.0*powDI(ini_time_,2),      2.0*ini_time_,       1.0, 0.0,                                  0.0,
                20.0*powDI(ini_time_,3), 12.0*powDI(ini_time_,2),          6.0*ini_time_,                2.0,       0.0, 0.0,                                  0.0,
                     powDI(fin_time_,5),      powDI(fin_time_,4),     powDI(fin_time_,3), powDI(fin_time_,2), fin_time_, 1.0, powDI(fin_time_ - via_time_,5)/120.0,
                 5.0*powDI(fin_time_,4),  4.0*powDI(fin_time_,3), 3.0*powDI(fin_time_,2),      2.0*fin_time_,       1.0, 0.0,  powDI(fin_time_ - via_time_,4)/24.0,
                20.0*powDI(fin_time_,3), 12.0*powDI(fin_time_,2),          6.0*fin_time_,                2.0,       0.0, 0.0,   powDI(fin_time_ - via_time_,3)/6.0,
                     powDI(via_time_,5),      powDI(via_time_,4),     powDI(via_time_,3), powDI(via_time_,2), via_time_, 1.0,                                  0.0;

    conditions_mat.resize(7,1);
    conditions_mat.fill(0.0);

    for (int i = 0; i < number_of_joint_; i++)
    {
      conditions_mat.coeffRef(0,0) = ini_pos[i];
      conditions_mat.coeffRef(1,0) = ini_vel[i];
      conditions_mat.coeffRef(2,0) = ini_acc[i];
      conditions_mat.coeffRef(3,0) = fin_pos[i];
      conditions_mat.coeffRef(4,0) = fin_vel[i];
      conditions_mat.coeffRef(5,0) = fin_acc[i];
      conditions_mat.coeffRef(6,0) = via_pos[i];

      Eigen::MatrixXd position_coeff = time_mat.inverse() * conditions_mat;

      position_coeff_.block(0,i,7,1) = position_coeff;

      velocity_coeff_.coeffRef(0,i) = 0.0;
      velocity_coeff_.coeffRef(1,i) = 5.0*position_coeff.coeff(0,0);
      velocity_coeff_.coeffRef(2,i) = 4.0*position_coeff.coeff(1,0);
      velocity_coeff_.coeffRef(3,i) = 3.0*position_coeff.coeff(2,0);
      velocity_coeff_.coeffRef(4,i) = 2.0*position_coeff.coeff(3,0);
      velocity_coeff_.coeffRef(5,i) = position_coeff.coeff(4,0);
      velocity_coeff_.coeffRef(6,i) = position_coeff.coeff(6,0);

      acceleration_coeff_.coeffRef(0,i) = 0.0;
      acceleration_coeff_.coeffRef(1,i) = 0.0;
      acceleration_coeff_.coeffRef(2,i) = 20.0*position_coeff.coeff(0,0);
      acceleration_coeff_.coeffRef(3,i) = 12.0*position_coeff.coeff(1,0);
      acceleration_coeff_.coeffRef(4,i) = 6.0*position_coeff.coeff(2,0);
      acceleration_coeff_.coeffRef(5,i) = 2.0*position_coeff.coeff(3,0);
      acceleration_coeff_.coeffRef(6,i) = position_coeff.coeff(6,0);
    }
  }

  cur_time_ = 0.0;
  cur_pos_.resize(number_of_joint_);
  cur_vel_.resize(number_of_joint_);
  cur_acc_.resize(number_of_joint_);
}

MinimumJerkViaPoint::~MinimumJerkViaPoint()
{

}

std::vector<double_t> MinimumJerkViaPoint::getPosition(double time)
{
  if(time >= fin_time_)
    cur_pos_ = fin_pos_;
  else if(time <= ini_time_)
    cur_pos_ = ini_pos_;
  else if (time > ini_time_ && time <= via_time_)
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0, 0.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_pos = time_variables_ * position_coeff_.block(0,i,7,1);
      cur_pos_[i] = cur_pos.coeff(0,0);
    }
  }
  else if (time < fin_time_ && time > via_time_)
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0, powDI(time - via_time_,5)/120.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_pos = time_variables_ * position_coeff_.block(0,i,7,1);
      cur_pos_[i] = cur_pos.coeff(0,0);
    }
  }
  return cur_pos_;
}

std::vector<double_t> MinimumJerkViaPoint::getVelocity(double time)
{
  if(time >= fin_time_)
    cur_vel_ = fin_vel_;
  else if(time <= ini_time_)
    cur_vel_ = ini_vel_;
  else if (time > ini_time_ && time <= via_time_)
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0, 0.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_vel = time_variables_ * velocity_coeff_.block(0,i,7,1);
      cur_vel_[i] = cur_vel.coeff(0,0);
    }
  }
  else if (time < fin_time_ && time > via_time_)
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0, powDI(time - via_time_,4)/24.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_vel = time_variables_ * velocity_coeff_.block(0,i,7,1);
      cur_vel_[i] = cur_vel.coeff(0,0);
    }
  }
  return cur_vel_;
}

std::vector<double_t> MinimumJerkViaPoint::getAcceleration(double time)
{
  if(time >= fin_time_)
    cur_acc_  = fin_acc_;
  else if(time <= ini_time_)
    cur_acc_  = ini_acc_;
  else if (time > ini_time_ && time <= via_time_)
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0, 0.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_acc = time_variables_ * acceleration_coeff_.block(0,i,7,1);
      cur_acc_[i] = cur_acc.coeff(0,0);
    }
  }
  else if (time < fin_time_ && time > via_time_)
  {
    cur_time_ = time;
    time_variables_ << powDI(time, 5), powDI(time, 4), powDI(time, 3), powDI(time, 2), time, 1.0, powDI(time - via_time_,3)/6.0;

    for (int i = 0; i < number_of_joint_; i++)
    {
      Eigen::MatrixXd cur_acc = time_variables_ * acceleration_coeff_.block(0,i,7,1);
      cur_acc_[i] = cur_acc.coeff(0,0);
    }
  }

  return cur_acc_;
}
