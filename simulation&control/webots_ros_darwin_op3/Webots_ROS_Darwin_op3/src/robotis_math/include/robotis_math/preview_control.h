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

#ifndef ROBOTIS_MATH_PREVIEW_CONTROL_H_
#define ROBOTIS_MATH_PREVIEW_CONTROL_H_

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_linear_algebra.h"
#include "robotis_math_base.h"

#include <ros/ros.h>
#include <stdint.h>
#include <vector>

namespace robotis_framework
{

class PreviewControl
{
public:
  PreviewControl();
  virtual ~PreviewControl();

  Eigen::MatrixXd calcPreviewParam(double preview_time, double control_cycle,
                                   double lipm_height,
                                   Eigen::MatrixXd K, Eigen::MatrixXd P);

private:

};

}

#endif /* ROBOTIS_MATH_MINIMUM_JERK_TRAJECTORY_H_ */
