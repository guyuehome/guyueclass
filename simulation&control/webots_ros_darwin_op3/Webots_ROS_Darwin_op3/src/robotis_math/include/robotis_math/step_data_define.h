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
 * step_data_define.h
 *
 *  Created on: 2016. 8. 10.
 *      Author: Jay Song
 */

#ifndef ROBOTIS_MATH_STEP_DATA_DEFINE_H_
#define ROBOTIS_MATH_STEP_DATA_DEFINE_H_

#include <ostream>

namespace robotis_framework
{

typedef struct
{
  double x, y, z;
} Position3D;

typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

typedef struct
{
  int    moving_foot;
  double foot_z_swap, body_z_swap;
  double x_zmp_shift, y_zmp_shift;
  double shoulder_swing_gain, elbow_swing_gain;
  double waist_roll_angle, waist_pitch_angle, waist_yaw_angle;
  Pose3D left_foot_pose;
  Pose3D right_foot_pose;
  Pose3D body_pose;
} StepPositionData;

typedef struct
{
  int    walking_state;
  double abs_step_time, dsp_ratio;
  double start_time_delay_ratio_x,    start_time_delay_ratio_y,     start_time_delay_ratio_z;
  double start_time_delay_ratio_roll, start_time_delay_ratio_pitch, start_time_delay_ratio_yaw;
  double finish_time_advance_ratio_x,    finish_time_advance_ratio_y,     finish_time_advance_ratio_z;
  double finish_time_advance_ratio_roll, finish_time_advance_ratio_pitch, finish_time_advance_ratio_yaw;
} StepTimeData;

typedef struct
{
  StepPositionData position_data;
  StepTimeData     time_data;
} StepData;

std::ostream& operator<<(std::ostream& out, const Pose3D& pose);
std::ostream& operator<<(std::ostream& out, const StepPositionData& position_data);
std::ostream& operator<<(std::ostream& out, const StepTimeData& time_data);
std::ostream& operator<<(std::ostream& out, const StepData& step_data);
}

#endif /* ROBOTIS_MATH_STEP_DATA_DEFINE_H_ */
