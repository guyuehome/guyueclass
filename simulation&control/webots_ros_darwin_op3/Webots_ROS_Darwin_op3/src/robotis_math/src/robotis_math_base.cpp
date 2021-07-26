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

#include "robotis_math/robotis_math_base.h"

namespace robotis_framework
{

double sign(double x)
{
  if ( x < 0.0 )
    return -1.0;
  else if ( x > 0.0)
    return 1.0;
  else
    return 0.0;
}

int combination(int n, int r)
{
  if(n == r || r == 0)
    return 1;
  else
    return combination(n - 1, r - 1) + combination(n - 1, r);
}


}
