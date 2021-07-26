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
 * robotis_math_base.h
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#ifndef ROBOTIS_MATH_ROBOTIS_MATH_BASE_H_
#define ROBOTIS_MATH_ROBOTIS_MATH_BASE_H_

#include <iostream>
#include <cmath>

namespace robotis_framework
{

#define PRINT_VAR(X) std::cout << #X << " : " << X << std::endl
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

#define DEGREE2RADIAN (M_PI / 180.0)
#define RADIAN2DEGREE (180.0 / M_PI)

inline double powDI(double a, int b)
{
	return (b == 0 ? 1 : (b > 0 ? a * powDI(a, b - 1) : 1 / powDI(a, -b)));
}

double sign(double x);

int combination(int n, int r);

typedef struct
{
  double x, y;
} Point2D;

}



#endif /* ROBOTIS_MATH_ROBOTIS_MATH_BASE_H_ */
