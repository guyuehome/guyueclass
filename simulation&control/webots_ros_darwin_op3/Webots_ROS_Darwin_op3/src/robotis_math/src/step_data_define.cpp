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

#include <robotis_math/step_data_define.h>
#include <iostream>
#include <iomanip>

namespace robotis_framework
{
std::string dispatchMovingFoot(int moving_foot)
{
  switch (moving_foot)
  {
    case 1: return "LEFT_FOOT_SWING";
    case 2: return "RIGHT_FOOT_SWING";
    case 3: return "STANDING";
    default: return "UNKOWN";
  }
}

std::string dispatchWalkingState(int walking_state)
{
  switch (walking_state)
  {
    case 0: return "IN_WALKING_STARTING";
    case 1: return "IN_WALKING";
    case 2: return "IN_WALKING_ENDING";
    default: return "UNKOWN";
  }
}

std::ostream& operator<<(std::ostream& out, const Pose3D& pose)
{
  return out << std::fixed << std::setprecision(3) << pose.x << "/" << pose.y << "/" << pose.z << "|" << pose.roll << "/" << pose.pitch << "/" << pose.yaw;
}

std::ostream& operator<<(std::ostream& out, const StepPositionData& position_data)
{
  out <<  std::fixed << std::setprecision(3);
  out << "[  Left] " << position_data.left_foot_pose << "\n";
  out << "[ RIGHT] " << position_data.right_foot_pose << "\n";
  out << "[  BODY] " << position_data.body_pose << "\n";
  out << "[ WAIST] " << "r/p/y: " << position_data.waist_roll_angle << "/" << position_data.waist_pitch_angle << "/" << position_data.waist_yaw_angle << "\n";
  out << "[PARAMS] " << "moving_foot: " << dispatchMovingFoot(position_data.moving_foot) << " | foot_z_swap: " << position_data.foot_z_swap << " | body_z_swap: " << position_data.body_z_swap << "\n";
  out << "[PARAMS] " << "x_zmp_shift: " << position_data.x_zmp_shift  << " | y_zmp_shift: " << position_data.y_zmp_shift << "\n";
  out << "[PARAMS] " << "shoulder_swing_gain: " << position_data.shoulder_swing_gain << " | elbow_swing_gain: " << position_data.elbow_swing_gain;
  return out;
}

std::ostream& operator<<(std::ostream& out, const StepTimeData& time_data)
{
  out <<  std::fixed << std::setprecision(3);
  out << "[PARAMS] " << "walking_state: " << dispatchWalkingState(time_data.walking_state) << " | abs_step_time: " << time_data.abs_step_time << " | dsp_ratio: " << time_data.dsp_ratio << "\n";
  out << "[ START] " << time_data.start_time_delay_ratio_x << "/" << time_data.start_time_delay_ratio_y << "/" << time_data.start_time_delay_ratio_z << "|"
      << time_data.start_time_delay_ratio_roll << "/" << time_data.start_time_delay_ratio_pitch << "/" << time_data.start_time_delay_ratio_yaw << "\n";
  out << "[FINISH] " << time_data.finish_time_advance_ratio_x << "/" << time_data.finish_time_advance_ratio_y << "/" << time_data.finish_time_advance_ratio_z << "|"
      << time_data.finish_time_advance_ratio_roll << "/" << time_data.finish_time_advance_ratio_pitch << "/" << time_data.finish_time_advance_ratio_yaw;
}

std::ostream& operator<<(std::ostream& out, const StepData& step_data)
{
  out << "------- StepPositionData: -------\n" << step_data.position_data << "\n";
  out << "--------- StepTimeData: ---------\n" << step_data.time_data;
  return out;
}
}
