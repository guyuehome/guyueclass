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

/* Author: SCH */

#ifndef OP3_KINEMATICS_DYNAMICS_DEFINE_H_
#define OP3_KINEMATICS_DYNAMICS_DEFINE_H_

namespace robotis_op
{
#define MAX_JOINT_ID    (20)
#define ALL_JOINT_ID    (31)

#define MAX_ARM_ID      (3)
#define MAX_LEG_ID      (6)
#define MAX_ITER        (5)

#define ID_HEAD_END     (20)
#define ID_COB          (29)
#define ID_TORSO        (29)

#define ID_R_ARM_START  (1)
#define ID_L_ARM_START  (2)
#define ID_R_ARM_END    (21)
#define ID_L_ARM_END    (22)

#define ID_R_LEG_START  (7)
#define ID_L_LEG_START  (8)
#define ID_R_LEG_END    (31)
#define ID_L_LEG_END    (30)

#define GRAVITY_ACCELERATION (9.8)
}

#endif /* OP3_KINEMATICS_DYNAMICS_DEFINE_H_ */
