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
 * simple_trapezoidal_velocity_profile.cpp
 *
 *  Created on: 2016. 8. 24.
 *      Author: Jay Song
 */

#include <iostream>
#include "robotis_math/simple_trapezoidal_velocity_profile.h"

using namespace robotis_framework;

SimpleTrapezoidalVelocityProfile::SimpleTrapezoidalVelocityProfile()
{
  pos_coeff_accel_section_.resize(3, 1);
  pos_coeff_accel_section_.fill(0);
  vel_coeff_accel_section_.resize(3, 1);
  vel_coeff_accel_section_.fill(0);

  pos_coeff_const_section_.resize(3, 1);
  pos_coeff_const_section_.fill(0);
  vel_coeff_const_section_.resize(3, 1);
  vel_coeff_const_section_.fill(0);

  pos_coeff_decel_section_.resize(3, 1);
  pos_coeff_decel_section_.fill(0);
  vel_coeff_decel_section_.resize(3, 1);
  vel_coeff_decel_section_.fill(0);

  time_variables_.resize(1, 3);
  time_variables_.fill(0);

  acceleration_ = 0;
  deceleration_ = 0;
  max_velocity_ = 0;

  initial_pos_ = 0;
  final_pos_ = 0;

  current_time_ = 0;
  current_pos_ = 0;
  current_vel_ = 0;
  current_acc_ = 0;

  accel_time_ = 0;
  const_time_ = 0;
  decel_time_ = 0;

  const_start_time_ = 0;
  decel_start_time_ = 0;
  total_time_ = 0;
}

SimpleTrapezoidalVelocityProfile::~SimpleTrapezoidalVelocityProfile()
{

}

void SimpleTrapezoidalVelocityProfile::setVelocityBaseTrajectory(double init_pos, double final_pos, double acceleration, double max_velocity)
{
  setVelocityBaseTrajectory(init_pos, final_pos, acceleration, acceleration, max_velocity);
}

void SimpleTrapezoidalVelocityProfile::setVelocityBaseTrajectory(double init_pos, double final_pos, double acceleration, double deceleration, double max_velocity)
{
  double pos_diff = final_pos - init_pos;
  double acc_time = fabs(max_velocity / acceleration);
  double dec_time = fabs(max_velocity / deceleration);

  acceleration_ = copysign(fabs(acceleration),  pos_diff);
  deceleration_ = copysign(fabs(deceleration), -pos_diff);
  max_velocity_ = copysign(fabs(max_velocity),  pos_diff);

  initial_pos_ = init_pos;
  final_pos_   = final_pos;

  if(fabs(pos_diff) > 0.5 * fabs(max_velocity_) * ( acc_time + dec_time ))
  {
    accel_time_ = acc_time;
    decel_time_ = dec_time;

    const_time_ = (fabs(pos_diff) - 0.5 * fabs(max_velocity_) * ( acc_time + dec_time )) / fabs(max_velocity_);
    total_time_ = accel_time_ + const_time_ + decel_time_;
    const_start_time_ = accel_time_;
    decel_start_time_ = accel_time_ + const_time_;
  }
  else
  {
    accel_time_ = sqrt(2*fabs(pos_diff)*fabs(deceleration_) / (acceleration_*acceleration_ + fabs(acceleration_*deceleration_)) );
    decel_time_ = accel_time_*fabs(acceleration_/deceleration_);

    const_time_ = 0;
    total_time_ = accel_time_ + const_time_ + decel_time_;
    const_start_time_ = accel_time_;
    decel_start_time_ = accel_time_ + const_time_;
  }

  //acc section
  pos_coeff_accel_section_.coeffRef(0,0) = 0.5*acceleration_;
  pos_coeff_accel_section_.coeffRef(1,0) = 0;
  pos_coeff_accel_section_.coeffRef(2,0) = initial_pos_;

  vel_coeff_accel_section_.coeffRef(0,0) = 0;
  vel_coeff_accel_section_.coeffRef(1,0) = acceleration_;
  vel_coeff_accel_section_.coeffRef(2,0) = 0;

  //const section
  pos_coeff_const_section_.coeffRef(0,0) = 0;
  pos_coeff_const_section_.coeffRef(1,0) = max_velocity_;
  pos_coeff_const_section_.coeffRef(2,0) = -0.5*acceleration_*accel_time_*accel_time_ + initial_pos_;

  vel_coeff_const_section_.coeffRef(0,0) = 0;
  vel_coeff_const_section_.coeffRef(1,0) = 0;
  vel_coeff_const_section_.coeffRef(2,0) = max_velocity_;

  //decel section
  pos_coeff_decel_section_.coeffRef(0,0) = 0.5*deceleration_;
  pos_coeff_decel_section_.coeffRef(1,0) = -deceleration_*total_time_;
  pos_coeff_decel_section_.coeffRef(2,0) = 0.5*deceleration_*total_time_*total_time_ + final_pos_;

  vel_coeff_decel_section_.coeffRef(0,0) = 0;
  vel_coeff_decel_section_.coeffRef(1,0) = deceleration_;
  vel_coeff_decel_section_.coeffRef(2,0) = -deceleration_*total_time_;
}

void SimpleTrapezoidalVelocityProfile::setTimeBaseTrajectory(double init_pos, double final_pos, double accel_time, double total_time)
{
  setTimeBaseTrajectory(init_pos, final_pos, accel_time, accel_time, total_time);
}

void SimpleTrapezoidalVelocityProfile::setTimeBaseTrajectory(double init_pos, double final_pos, double accel_time, double decel_time, double total_time)
{
  initial_pos_ = init_pos;
  final_pos_   = final_pos;
  total_time_  = fabs(total_time);

  if((fabs(accel_time) + fabs(decel_time)) <= total_time_)
  {
    accel_time_ = fabs(accel_time);
    decel_time_ = fabs(decel_time);
    const_time_ = total_time - accel_time_ - decel_time_;
  }
  else
  {
    double time_gain = total_time_ / (fabs(accel_time) + fabs(decel_time));
    accel_time_ = time_gain*fabs(accel_time);
    decel_time_ = time_gain*fabs(decel_time);
    const_time_ = 0;
  }

  const_start_time_ = accel_time_;
  decel_start_time_ = accel_time_ + const_time_;

  double pos_diff = final_pos - init_pos;
  max_velocity_ = 2*pos_diff / (total_time_ + const_time_);
  acceleration_ = max_velocity_ / accel_time_;
  deceleration_ = max_velocity_ / decel_time_;

  //acc section
  pos_coeff_accel_section_.coeffRef(0,0) = 0.5*acceleration_;
  pos_coeff_accel_section_.coeffRef(1,0) = 0;
  pos_coeff_accel_section_.coeffRef(2,0) = initial_pos_;

  vel_coeff_accel_section_.coeffRef(0,0) = 0;
  vel_coeff_accel_section_.coeffRef(1,0) = acceleration_;
  vel_coeff_accel_section_.coeffRef(2,0) = 0;

  //const section
  pos_coeff_const_section_.coeffRef(0,0) = 0;
  pos_coeff_const_section_.coeffRef(1,0) = max_velocity_;
  pos_coeff_const_section_.coeffRef(2,0) = -0.5*acceleration_*accel_time_*accel_time_ + initial_pos_;

  vel_coeff_const_section_.coeffRef(0,0) = 0;
  vel_coeff_const_section_.coeffRef(1,0) = 0;
  vel_coeff_const_section_.coeffRef(2,0) = max_velocity_;

  //decel section
  pos_coeff_decel_section_.coeffRef(0,0) = 0.5*deceleration_;
  pos_coeff_decel_section_.coeffRef(1,0) = -deceleration_*total_time_;
  pos_coeff_decel_section_.coeffRef(2,0) = 0.5*deceleration_*total_time_*total_time_ + final_pos_;

  vel_coeff_decel_section_.coeffRef(0,0) = 0;
  vel_coeff_decel_section_.coeffRef(1,0) = deceleration_;
  vel_coeff_decel_section_.coeffRef(2,0) = -deceleration_*total_time_;
}

double SimpleTrapezoidalVelocityProfile::getPosition(double time)
{
  setTime(time);
  return current_pos_;
}

double SimpleTrapezoidalVelocityProfile::getVelocity(double time)
{
  setTime(time);
  return current_vel_;
}

double SimpleTrapezoidalVelocityProfile::getAcceleration(double time)
{
  setTime(time);
  return current_acc_;
}

void SimpleTrapezoidalVelocityProfile::setTime(double time)
{
  time_variables_ << powDI(time, 2), time, 1.0;

  if(time <= 0)
  {
    current_time_ = 0;
    current_pos_  = initial_pos_;
    current_vel_  = 0;
    current_acc_  = 0;
  }
  else if(time < const_start_time_)
  {
    current_time_ = time;
    current_pos_  = (time_variables_ * pos_coeff_accel_section_).coeff(0,0);
    current_vel_  = (time_variables_ * vel_coeff_accel_section_).coeff(0,0);
    current_acc_  = acceleration_;
  }
  else if(time <= decel_start_time_)
  {
    current_time_ = time;
    current_pos_  = (time_variables_ * pos_coeff_const_section_).coeff(0,0);
    current_vel_  = (time_variables_ * vel_coeff_const_section_).coeff(0,0);
    current_acc_  = 0;
  }
  else if(time < total_time_)
  {
    current_time_ = time;
    current_pos_  = (time_variables_ * pos_coeff_decel_section_).coeff(0,0);
    current_vel_  = (time_variables_ * vel_coeff_decel_section_).coeff(0,0);
    current_acc_  = deceleration_;
  }
  else
  {
    current_time_ = total_time_;
    current_pos_  = final_pos_;
    current_vel_  = 0;
    current_acc_  = 0;
  }
}

double SimpleTrapezoidalVelocityProfile::getPosition()
{
  return current_pos_;
}

double SimpleTrapezoidalVelocityProfile::getVelocity()
{
  return current_vel_;
}

double SimpleTrapezoidalVelocityProfile::getAcceleration()
{
  return current_acc_;
}

double SimpleTrapezoidalVelocityProfile::getTotalTime()
{
  return total_time_;
}

double SimpleTrapezoidalVelocityProfile::getConstantVelocitySectionStartTime()
{
  return const_start_time_;
}

double SimpleTrapezoidalVelocityProfile::getDecelerationSectionStartTime()
{
  return decel_start_time_;
}


