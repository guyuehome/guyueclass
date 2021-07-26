/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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

/* Authors: SCH, Jay Song */

#include "op3_kinematics_dynamics/link_data.h"

namespace robotis_op
{

LinkData::LinkData()
{
  name_ = "";

  parent_ = -1;
  sibling_ = -1;
  child_ = -1;

  mass_ = 0.0;

  relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  joint_limit_max_ = 100.0;
  joint_limit_min_ = -100.0;

  joint_angle_ = 0.0;
  joint_velocity_ = 0.0;
  joint_acceleration_ = 0.0;

  position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
  transformation_ = robotis_framework::getTransformationXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

LinkData::~LinkData()
{
}

}
