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
 * bezier_curve.h
 *
 *  Created on: 2016. 8. 12.
 *      Author: Jay Song
 */

#ifndef ROBOTIS_MATH_BEZIER_CURVE_H_
#define ROBOTIS_MATH_BEZIER_CURVE_H_

#include <vector>

#include "robotis_math_base.h"
#include "robotis_linear_algebra.h"

namespace robotis_framework
{

class BezierCurve
{
public:
  BezierCurve();
  ~BezierCurve();

  void setBezierControlPoints(const std::vector<Point2D>& points);

  Point2D getPoint(double t);

private:
  std::vector<Point2D> control_points_;

};

}


#endif /* ROBOTIS_MATH_BEZIER_CURVE_H_ */
