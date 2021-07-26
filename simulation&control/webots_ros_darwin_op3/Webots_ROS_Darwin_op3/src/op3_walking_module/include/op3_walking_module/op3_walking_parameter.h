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

/* Author: Kayman */

#ifndef OP3_WALKING_PARAMETER_H_
#define OP3_WALKING_PARAMETER_H_

class WalkingTimeParameter
{
 public:
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

 private:
  double periodtime;
  double dsp_ratio;
  double ssp_ratio;
  double x_swap_periodtime;
  double x_move_periodtime;
  double y_swap_periodtime;
  double y_move_periodtime;
  double z_swap_periodtime;
  double z_move_periodtime;
  double a_move_periodtime;
  double ssp_time;
  double ssp_time_start_l;
  double ssp_time_end_l;
  double ssp_time_start_r;
  double ssp_time_end_r;
  double phase1_time;
  double phase2_time;
  double phase3_time;
};

class WalkingMovementParameter
{
 private:

};

class WalkingBalanceParameter
{

};

#endif /* OP3_WALKING_PARAMETER_H_ */
