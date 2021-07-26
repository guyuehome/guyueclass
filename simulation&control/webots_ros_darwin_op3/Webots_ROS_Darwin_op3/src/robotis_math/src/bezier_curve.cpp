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
 * bezier_curve.cpp
 *
 *  Created on: 2016. 8. 12.
 *      Author: Jay Song
 */

#include "robotis_math/bezier_curve.h"

using namespace robotis_framework;

BezierCurve::BezierCurve()
{

}

BezierCurve::~BezierCurve()
{

}

void BezierCurve::setBezierControlPoints(const std::vector<Point2D>& points)
{
  control_points_.clear();
  control_points_ = points;
}

Point2D BezierCurve::getPoint(double t)
{
  if(t > 1)
    t = 1;
  else if(t < 0)
    t = 0;

  int points_num = control_points_.size();
  Point2D point_at_t;
  point_at_t.x = 0;
  point_at_t.y = 0;

  if(points_num < 2)
    return point_at_t;

  point_at_t.x = control_points_[0].x * powDI(1-t, points_num - 1);
  point_at_t.y = control_points_[0].y * powDI(1-t, points_num - 1);

  for(unsigned int i = 1; i < (points_num - 1); i++)
  {
    point_at_t.x += control_points_[i].x * combination(points_num - 1, i)* powDI(1-t, points_num - 1 - i)*powDI(t, i);
    point_at_t.y += control_points_[i].y * combination(points_num - 1, i)* powDI(1-t, points_num - 1 - i)*powDI(t, i);
  }

  point_at_t.x += control_points_[points_num - 1].x * powDI(t, points_num - 1);
  point_at_t.y += control_points_[points_num - 1].y * powDI(t, points_num - 1);

  return point_at_t;
}

