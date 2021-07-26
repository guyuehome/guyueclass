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
 * robotis_trajectory_calculator.cpp
 *
 *  Created on: June 7, 2016
 *      Author: SCH
 */

#include "robotis_math/robotis_trajectory_calculator.h"

namespace robotis_framework
{

Eigen::MatrixXd calcMinimumJerkTra(
    double pos_start, double vel_start, double accel_start,
    double pos_end,   double vel_end,   double accel_end,
    double smp_time,  double mov_time
    )
/*
   simple minimum jerk trajectory

   pos_start : position at initial state
   vel_start : velocity at initial state
   accel_start : acceleration at initial state

   pos_end : position at final state
   vel_end : velocity at final state
   accel_end : acceleration at final state

   smp_time : sampling time
   mov_time : movement time
 */

{
  Eigen::MatrixXd poly_matrix(3,3);
  Eigen::MatrixXd poly_vector(3,1);

  poly_matrix <<
      pow(mov_time,3), pow(mov_time,4), pow(mov_time,5),
      3*pow(mov_time,2), 4*pow(mov_time,3), 5*pow(mov_time,4),
      6*mov_time, 12*pow(mov_time,2), 20*pow(mov_time,3);

  poly_vector <<
      pos_end-pos_start-vel_start*mov_time-accel_start*pow(mov_time,2)/2,
      vel_end-vel_start-accel_start*mov_time,
      accel_end-accel_start ;

  Eigen::Matrix<double,3,1> poly_coeff = poly_matrix.inverse() * poly_vector;

  double time_steps = mov_time/smp_time;
  int all_time_steps = round(time_steps+1);

  Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps,1);
  Eigen::MatrixXd minimum_jerk_tra = Eigen::MatrixXd::Zero(all_time_steps,1);

  for (int step=0; step<all_time_steps; step++)
    time.coeffRef(step,0) = step*smp_time;

  for (int step=0; step<all_time_steps; step++)
  {
    minimum_jerk_tra.coeffRef( step , 0 ) =
        pos_start +
        vel_start*time.coeff(step,0) +
        0.5*accel_start*pow(time.coeff(step,0),2) +
        poly_coeff.coeff(0,0)*pow(time.coeff(step,0),3) +
        poly_coeff.coeff(1,0)*pow(time.coeff(step,0),4) +
        poly_coeff.coeff(2,0)*pow(time.coeff(step,0),5);
  }

  return minimum_jerk_tra;
}

Eigen::MatrixXd calcMinimumJerkTraPlus(
    double pos_start, double vel_start, double accel_start,
    double pos_end,   double vel_end,   double accel_end,
    double smp_time,  double mov_time
    )
/*
   simple minimum jerk trajectory

   pos_start : position at initial state
   vel_start : velocity at initial state
   accel_start : acceleration at initial state

   pos_end : position at final state
   vel_end : velocity at final state
   accel_end : acceleration at final state

   smp_time : sampling time
   mov_time : movement time
 */

{
  Eigen::MatrixXd poly_matrix(3,3);
  Eigen::MatrixXd poly_vector(3,1);

  poly_matrix <<
      pow(mov_time,3), pow(mov_time,4), pow(mov_time,5),
      3*pow(mov_time,2), 4*pow(mov_time,3), 5*pow(mov_time,4),
      6*mov_time, 12*pow(mov_time,2), 20*pow(mov_time,3);

  poly_vector <<
      pos_end-pos_start-vel_start*mov_time-accel_start*pow(mov_time,2)/2,
      vel_end-vel_start-accel_start*mov_time,
      accel_end-accel_start ;

  Eigen::Matrix<double,3,1> poly_coeff = poly_matrix.inverse() * poly_vector;

  double time_steps = mov_time/smp_time;
  int all_time_steps = round(time_steps+1);

  Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps,1);
  Eigen::MatrixXd minimum_jerk_tra = Eigen::MatrixXd::Zero(all_time_steps,3);

  for (int step=0; step<all_time_steps; step++)
    time.coeffRef(step,0) = step*smp_time;

  for (int step=0; step<all_time_steps; step++)
  {
    minimum_jerk_tra.coeffRef( step , 0 ) =
        pos_start +
        vel_start*time.coeff(step,0) +
        0.5*accel_start*pow(time.coeff(step,0),2) +
        poly_coeff.coeff(0,0)*pow(time.coeff(step,0),3) +
        poly_coeff.coeff(1,0)*pow(time.coeff(step,0),4) +
        poly_coeff.coeff(2,0)*pow(time.coeff(step,0),5);

    minimum_jerk_tra.coeffRef( step , 1 ) =
        vel_start +
        accel_start*time.coeff(step,0) +
        3*poly_coeff.coeff(0,0)*pow(time.coeff(step,0),2) +
        4*poly_coeff.coeff(1,0)*pow(time.coeff(step,0),3) +
        5*poly_coeff.coeff(2,0)*pow(time.coeff(step,0),4);

    minimum_jerk_tra.coeffRef( step , 2 ) =
        accel_start +
        6*poly_coeff.coeff(0,0)*time.coeff(step,0) +
        12*poly_coeff.coeff(1,0)*pow(time.coeff(step,0),2) +
        20*poly_coeff.coeff(2,0)*pow(time.coeff(step,0),3);
  }

  return minimum_jerk_tra;
}

Eigen::MatrixXd calcMinimumJerkTraWithViaPoints(int via_num,
                                                double pos_start, double vel_start, double accel_start,
                                                Eigen::MatrixXd pos_via,  Eigen::MatrixXd vel_via, Eigen::MatrixXd accel_via,
                                                double pos_end, double vel_end, double accel_end,
                                                double smp_time, Eigen::MatrixXd via_time, double mov_time )
/*
   minimum jerk trajectory with via-points
   (via-point constraints: position and velocity at each point)

   via_num  : the number of via-points

   pos_start : position at initial state
   vel_start : velocity at initial state
   accel_start : acceleration at initial state

   pos_via  : position matrix at via-points state ( size : n x 1 )
   vel_via : velocity matrix at via-points state ( size : n x 1 )
   accel_via : acceleration matrix at via-points state ( size : n x 1 )

   pos_end : position at final state
   vel_end : velocity at final state
   accel_end : acceleration at final state

   smp_time : sampling time

   via_time  : time matrix passing through via-points state ( size : n x 1 )
   via_time : movement time
 */

{
  //    int i,j,k ;

  Eigen::MatrixXd poly_vector = Eigen::MatrixXd::Zero(3*via_num+3,1);

  for (int i=1; i<=via_num; i++)
  {
    poly_vector.coeffRef(3*i-3,0) =
        pos_via.coeff(i-1,0) -
        pos_start -
        vel_start*via_time.coeff(i-1,0) -
        (accel_start/2)*pow(via_time.coeff(i-1,0),2) ;

    poly_vector.coeffRef(3*i-2,0) =
        vel_via.coeff(i-1,0) -
        vel_start -
        accel_start*via_time.coeff(i-1,0) ;

    poly_vector.coeffRef(3*i-1,0) =
        accel_via.coeff(i-1,0) -
        accel_start;
  }

  poly_vector.coeffRef(3*via_num,0) =
      pos_end -
      pos_start -
      vel_start*mov_time -
      (accel_start/2)*pow(mov_time,2) ;

  poly_vector.coeffRef(3*via_num+1,0) =
      vel_end -
      vel_start -
      accel_start*mov_time ;

  poly_vector.coeffRef(3*via_num+2,0) =
      accel_end -
      accel_start ;

  Eigen::MatrixXd poly_matrix_part1 = Eigen::MatrixXd::Zero(3*via_num,3);

  for (int i=1; i<=via_num; i++)
  {
    poly_matrix_part1.coeffRef(3*i-3,0) = pow(via_time.coeff(i-1,0),3) ;
    poly_matrix_part1.coeffRef(3*i-3,1) = pow(via_time.coeff(i-1,0),4) ;
    poly_matrix_part1.coeffRef(3*i-3,2) = pow(via_time.coeff(i-1,0),5) ;

    poly_matrix_part1.coeffRef(3*i-2,0) = 3*pow(via_time.coeff(i-1,0),2) ;
    poly_matrix_part1.coeffRef(3*i-2,1) = 4*pow(via_time.coeff(i-1,0),3) ;
    poly_matrix_part1.coeffRef(3*i-2,2) = 5*pow(via_time.coeff(i-1,0),4) ;

    poly_matrix_part1.coeffRef(3*i-1,0) = 6*via_time.coeff(i-1,0) ;
    poly_matrix_part1.coeffRef(3*i-1,1) = 12*pow(via_time.coeff(i-1,0),2) ;
    poly_matrix_part1.coeffRef(3*i-1,2) = 20*pow(via_time.coeff(i-1,0),3) ;
  }

  Eigen::MatrixXd poly_matrix_part2 = Eigen::MatrixXd::Zero(3*via_num,3*via_num);

  for (int i=1; i<=via_num; i++)
  {
    for (int j=1; j<=via_num; j++)
    {
      int k;

      if (i > j)
        k = i ;
      else
        k = j ;

      poly_matrix_part2.coeffRef(3*j-3,3*i-3) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),3)/6 ;
      poly_matrix_part2.coeffRef(3*j-3,3*i-2) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),4)/24 ;
      poly_matrix_part2.coeffRef(3*j-3,3*i-1) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),5)/120 ;

      poly_matrix_part2.coeffRef(3*j-2,3*i-3) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),2)/2 ;
      poly_matrix_part2.coeffRef(3*j-2,3*i-2) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),3)/6 ;
      poly_matrix_part2.coeffRef(3*j-2,3*i-1) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),4)/24 ;

      poly_matrix_part2.coeffRef(3*j-1,3*i-3) = via_time.coeff(k-1,0)-via_time.coeff(i-1,0) ;
      poly_matrix_part2.coeffRef(3*j-1,3*i-2) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),2)/2 ;
      poly_matrix_part2.coeffRef(3*j-1,3*i-1) = pow((via_time.coeff(k-1,0)-via_time.coeff(i-1,0)),3)/6 ;
    }
  }


  Eigen::MatrixXd poly_matrix_part3 = Eigen::MatrixXd::Zero(3,3*via_num+3);

  poly_matrix_part3.coeffRef(0,0) = pow(mov_time,3);
  poly_matrix_part3.coeffRef(0,1) = pow(mov_time,4);
  poly_matrix_part3.coeffRef(0,2) = pow(mov_time,5);

  poly_matrix_part3.coeffRef(1,0) = 3*pow(mov_time,2);
  poly_matrix_part3.coeffRef(1,1) = 4*pow(mov_time,3);
  poly_matrix_part3.coeffRef(1,2) = 5*pow(mov_time,4);

  poly_matrix_part3.coeffRef(2,0) = 6*mov_time;
  poly_matrix_part3.coeffRef(2,1) = 12*pow(mov_time,2);
  poly_matrix_part3.coeffRef(2,2) = 20*pow(mov_time,3);

  for (int i=1; i<=via_num; i++)
  {
    poly_matrix_part3.coeffRef(0,3*i) = pow(mov_time-via_time.coeff(i-1,0),3)/6 ;
    poly_matrix_part3.coeffRef(1,3*i) = pow(mov_time-via_time.coeff(i-1,0),2)/2 ;
    poly_matrix_part3.coeffRef(2,3*i) = mov_time-via_time.coeff(i-1,0) ;

    poly_matrix_part3.coeffRef(0,3*i+1) = pow(mov_time-via_time.coeff(i-1,0),4)/24 ;
    poly_matrix_part3.coeffRef(1,3*i+1) = pow(mov_time-via_time.coeff(i-1,0),3)/6 ;
    poly_matrix_part3.coeffRef(2,3*i+1) = pow(mov_time-via_time.coeff(i-1,0),2)/2 ;

    poly_matrix_part3.coeffRef(0,3*i+2) = pow(mov_time-via_time.coeff(i-1,0),5)/120 ;
    poly_matrix_part3.coeffRef(1,3*i+2) = pow(mov_time-via_time.coeff(i-1,0),4)/24 ;
    poly_matrix_part3.coeffRef(2,3*i+2) = pow(mov_time-via_time.coeff(i-1,0),3)/6 ;
  }

  Eigen::MatrixXd poly_matrix = Eigen::MatrixXd::Zero(3*via_num+3,3*via_num+3);

  poly_matrix.block(0,0,3*via_num,3) = poly_matrix_part1 ;
  poly_matrix.block(0,3,3*via_num,3*via_num) = poly_matrix_part2 ;
  poly_matrix.block(3*via_num,0,3,3*via_num+3) = poly_matrix_part3 ;

  Eigen::MatrixXd poly_coeff(3*via_num+3,1);
  //C = A.inverse()*B;
  poly_coeff = poly_matrix.colPivHouseholderQr().solve(poly_vector);

  int all_time_steps;
  all_time_steps = round(mov_time/smp_time) ;

  Eigen::MatrixXd time_vector = Eigen::MatrixXd::Zero(all_time_steps+1,1);

  for (int i=1; i<=all_time_steps+1; i++)
    time_vector.coeffRef(i-1,0) = (i-1)*smp_time;

  Eigen::MatrixXd via_time_vector = Eigen::MatrixXd::Zero(via_num,1);

  for (int i=1; i<=via_num; i++)
    via_time_vector.coeffRef(i-1,0) = round(via_time.coeff(i-1,0)/smp_time)+2;

  Eigen::MatrixXd minimum_jerk_tra_with_via_points = Eigen::MatrixXd::Zero(all_time_steps+1,1);

  for (int i=1; i<=all_time_steps+1; i++)
  {
    minimum_jerk_tra_with_via_points.coeffRef(i-1,0) =
        pos_start +
        vel_start*time_vector.coeff(i-1,0) +
        0.5*accel_start*pow(time_vector.coeff(i-1,0),2) +
        poly_coeff.coeff(0,0)*pow(time_vector.coeff(i-1,0),3) +
        poly_coeff.coeff(1,0)*pow(time_vector.coeff(i-1,0),4) +
        poly_coeff.coeff(2,0)*pow(time_vector.coeff(i-1,0),5) ;
  }

  for (int i=1; i<=via_num; i++)
  {
    for (int j=via_time_vector.coeff(i-1,0); j<=all_time_steps+1; j++)
    {
      minimum_jerk_tra_with_via_points.coeffRef(j-1,0) =
          minimum_jerk_tra_with_via_points.coeff(j-1,0) +
          poly_coeff.coeff(3*i,0)*pow((time_vector.coeff(j-1,0)-via_time.coeff(i-1,0)),3)/6 +
          poly_coeff.coeff(3*i+1,0)*pow((time_vector.coeff(j-1,0)-via_time.coeff(i-1,0)),4)/24 +
          poly_coeff.coeff(3*i+2,0)*pow((time_vector.coeff(j-1,0)-via_time.coeff(i-1,0)),5)/120 ;

    }
  }

  return minimum_jerk_tra_with_via_points;
}

Eigen::MatrixXd calcMinimumJerkTraWithViaPointsPosition(int via_num,
                                                        double pos_start, double vel_start, double accel_start,
                                                        Eigen::MatrixXd pos_via,
                                                        double pos_end, double vel_end, double accel_end,
                                                        double smp_time, Eigen::MatrixXd via_time, double mov_time)

/* n is the number of via-points.

   x0, v0 and a0 are initial values of
   joint angle, angular velocity and angular acceleration.

   x (Matirx) and dx (Matirx) are
   joint angles and angular velocities at the via-points.

   xf, vf and af are final values of
   joint angle, angular velocity and angular acceleration.

   smp is sampling time.

   t (Matirx) is the time values passing via-points.

   tf is movement time */

{
  int i,j,k ;

  /* Calculation Matrix B	*/

  Eigen::MatrixXd poly_vector = Eigen::MatrixXd::Zero(via_num+3,1);

  for (i=1; i<=via_num; i++){
    poly_vector.coeffRef(i-1,0) =
        pos_via.coeff(i-1,0) -
        pos_start -
        vel_start*via_time.coeff(i-1,0) -
        (accel_start/2)*pow(via_time.coeff(i-1,0),2) ;
  }

  poly_vector.coeffRef(via_num,0) =
      pos_end -
      pos_start -
      vel_start*mov_time -
      (accel_start/2)*pow(mov_time,2) ;

  poly_vector.coeffRef(via_num+1,0) =
      vel_end -
      vel_start -
      accel_start*mov_time ;

  poly_vector.coeffRef(via_num+2,0) =
      accel_end -
      accel_start ;

  /* Calculation Matrix A	*/

  Eigen::MatrixXd poly_part1 = Eigen::MatrixXd::Zero(via_num, 3);

  for (i = 1; i <= via_num; i++) {
    poly_part1.coeffRef(i-1,0) = pow(via_time.coeff(i-1,0),3);
    poly_part1.coeffRef(i-1,1) = pow(via_time.coeff(i-1,0),4);
    poly_part1.coeffRef(i-1,2) = pow(via_time.coeff(i-1,0),5);
  }

  Eigen::MatrixXd poly_part2 = Eigen::MatrixXd::Zero(via_num, via_num);

  for (i = 1; i <= via_num; i++) {
    for (j = 1; j <= via_num; j++) {
      if (i > j) {
        k = i;
      } else {
        k = j;
      }
      poly_part2.coeffRef(j-1, i-1) = pow(
          (via_time.coeff(k-1,0) - via_time.coeff(i-1,0)),5)/120;
    }
  }

  Eigen::MatrixXd poly_part3 = Eigen::MatrixXd::Zero(3,via_num+3);

  poly_part3.coeffRef(0,0) = pow(mov_time,3);
  poly_part3.coeffRef(0,1) = pow(mov_time,4);
  poly_part3.coeffRef(0,2) = pow(mov_time,5);

  poly_part3.coeffRef(1,0) = 3 * pow(mov_time,2);
  poly_part3.coeffRef(1,1) = 4 * pow(mov_time,3);
  poly_part3.coeffRef(1,2) = 5 * pow(mov_time,4);

  poly_part3.coeffRef(2,0) = 6 * mov_time;
  poly_part3.coeffRef(2,1) = 12 * pow(mov_time,2);
  poly_part3.coeffRef(2,2) = 20 * pow(mov_time,3);

  for (i = 1; i <= via_num; i++) {
    poly_part3.coeffRef(0, i+2) = pow(mov_time - via_time.coeff(i-1,0),5)/120;
    poly_part3.coeffRef(1, i+2) = pow(mov_time - via_time.coeff(i-1,0),4)/24;
    poly_part3.coeffRef(2, i+2) = pow(mov_time - via_time.coeff(i-1,0),3)/6;
  }

  Eigen::MatrixXd poly_matrix = Eigen::MatrixXd::Zero(via_num+3,via_num+3);

  poly_matrix.block(0,0,via_num,3) = poly_part1;
  poly_matrix.block(0,3,via_num,via_num) = poly_part2;
  poly_matrix.block(via_num,0,3,via_num+3) = poly_part3;

  /* Calculation Matrix C (coefficient of polynomial function) */

  Eigen::MatrixXd poly_inv_matrix(2*via_num+3,1);
  //C = A.inverse() * B;
  poly_inv_matrix = poly_matrix.colPivHouseholderQr().solve(poly_vector);

  /* Time */

  int all_time_steps;
  all_time_steps = round( int (mov_time/smp_time) ) ;

  Eigen::MatrixXd time_vector = Eigen::MatrixXd::Zero(all_time_steps+1,1);

  for (i=1; i<=all_time_steps+1; i++)
    time_vector.coeffRef(i-1,0) = (i-1)*smp_time;

  /* Time_via */
  Eigen::MatrixXd via_time_vector = Eigen::MatrixXd::Zero(via_num,1);

  for (i=1; i<=via_num; i++)
    via_time_vector.coeffRef(i-1,0) = round(via_time.coeff(i-1,0)/smp_time)+2;

  /* Minimum Jerk Trajectory with Via-points */

  Eigen::MatrixXd minimum_jerk_tra_with_via_points = Eigen::MatrixXd::Zero(all_time_steps+1,1);

  for (i=1; i<=all_time_steps+1; i++)
  {
    minimum_jerk_tra_with_via_points.coeffRef(i-1,0) =
        pos_start +
        vel_start*time_vector.coeff(i-1,0) +
        0.5*accel_start*pow(time_vector.coeff(i-1,0),2) +
        poly_inv_matrix.coeff(0,0)*pow(time_vector.coeff(i-1,0),3) +
        poly_inv_matrix.coeff(1,0)*pow(time_vector.coeff(i-1,0),4) +
        poly_inv_matrix.coeff(2,0)*pow(time_vector.coeff(i-1,0),5) ;
  }

  for (i=1; i<=via_num; i++)
  {
    for (j=via_time_vector.coeff(i-1,0); j<=all_time_steps+1; j++)
    {
      minimum_jerk_tra_with_via_points.coeffRef(j-1,0) =
          minimum_jerk_tra_with_via_points.coeff(j-1,0) +
          poly_inv_matrix.coeff(i+2,0)*pow((time_vector.coeff(j-1,0)-via_time.coeff(i-1,0)),5)/120 ;
    }
  }

  return minimum_jerk_tra_with_via_points;

}

Eigen::MatrixXd calcArc3dTra(double smp_time, double mov_time,
    Eigen::MatrixXd center_point, Eigen::MatrixXd normal_vector, Eigen::MatrixXd start_point,
    double rotation_angle, double cross_ratio )
{
  int all_time_steps = int (round(mov_time/smp_time))+1;

  Eigen::MatrixXd angle_tra = calcMinimumJerkTra(0.0, 0.0, 0.0,
      rotation_angle, 0.0, 0.0,
      smp_time, mov_time);

  Eigen::MatrixXd arc_tra = Eigen::MatrixXd::Zero(3,all_time_steps);

  for (int step = 0; step < all_time_steps; step++ )
  {
    double time = ((double)step)*smp_time;
    double theta = angle_tra.coeff(step,0);

    Eigen::MatrixXd weight_wedge(3,3);

    weight_wedge <<
        0.0, -normal_vector.coeff(2,0), normal_vector.coeff(1,0),
        normal_vector.coeff(2,0), 0.0, -normal_vector.coeff(0,0),
        -normal_vector.coeff(1,0), normal_vector.coeff(0,0), 0.0;

    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd rotation = identity + weight_wedge*sin(theta) + weight_wedge*weight_wedge*(1-cos(theta));
    double cross = cross_ratio*(1.0-2.0*(abs(0.5-(time/mov_time))));

    arc_tra.block(0,step,3,1) = (1+cross)*(rotation*(start_point-center_point))+center_point;
  }

  Eigen::MatrixXd act_tra_trans = arc_tra.transpose();

  return act_tra_trans;
}

}
