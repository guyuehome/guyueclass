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

/* Authors: SCH, Jay Song, Kayman */

#ifndef OP3_KINEMATICS_DYNAMICS_H_
#define OP3_KINEMATICS_DYNAMICS_H_

#include <vector>
#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include "op3_kinematics_dynamics_define.h"
#include "link_data.h"
#include <iostream>

namespace robotis_op
{

enum TreeSelect
{
  Manipulation,
  Walking,
  WholeBody
};

enum Phase{
  DOUBLE_SUPPORT,
  LEFT_SUPPORT,
  RIGHT_SUPPORT
};

class OP3KinematicsDynamics
{

 public:
  OP3KinematicsDynamics();
  ~OP3KinematicsDynamics();
  OP3KinematicsDynamics(TreeSelect tree);

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMC(int joint_id);
  Eigen::MatrixXd calcCOM(Eigen::MatrixXd mc);

  void calcForwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
                             double ik_err);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                             int max_iter, double ik_err);

  // with weight
  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
                             double ik_err, Eigen::MatrixXd weight);
  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                             int max_iter, double ik_err, Eigen::MatrixXd weight);

  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch,
                                        double yaw);
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch,
                                       double yaw);

  bool calcInverseKinematicsForLeg(double *out, tf::Pose& body_pose,
                                   tf::Pose& left_foot,  tf::Pose& right_foot);

  LinkData *op3_link_data_[ ALL_JOINT_ID + 1];

  LinkData *getLinkData(const std::string link_name);
  LinkData *getLinkData(const int link_id);
  Eigen::MatrixXd getJointAxis(const std::string link_name);
  double getJointDirection(const std::string link_name);
  double getJointDirection(const int link_id);

  Eigen::MatrixXd calcPreviewParam(double preview_time, double control_cycle,
                                   double lipm_height,
                                   Eigen::MatrixXd K, Eigen::MatrixXd P);

  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;
};

}

#endif /* OP3_KINEMATICS_DYNAMICS_H_ */
