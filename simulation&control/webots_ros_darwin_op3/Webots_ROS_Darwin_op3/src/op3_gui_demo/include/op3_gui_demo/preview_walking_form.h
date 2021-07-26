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

/* Author: Kayman Jung */

#ifndef PREVIEW_WALKING_FORM_H
#define PREVIEW_WALKING_FORM_H

#include <iostream>
#include <QWidget>

#include "ui_preview_walking_form.h"
#include "qnode.hpp"

namespace Ui {
class PreviewWalkingForm;
}

class PreviewWalkingForm : public QWidget
{
  Q_OBJECT

public:
  explicit PreviewWalkingForm(QWidget *parent = 0);
  ~PreviewWalkingForm();

  bool setQNode(robotis_op::QNodeOP3 *qnode);
  bool init(robotis_op::QNodeOP3 *qnode);

public Q_SLOTS:

 /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
 void on_button_p_walking_turn_l_clicked(bool check);
 void on_button_p_walking_turn_r_clicked(bool check);
 void on_button_p_walking_forward_clicked(bool check);
 void on_button_p_walking_backward_clicked(bool check);
 void on_button_p_walking_stop_clicked(bool check);
 void on_button_p_walking_left_clicked(bool check);
 void on_button_p_walking_right_clicked(bool check);

 void on_button_set_walking_param_clicked(bool check);
 void on_button_send_body_offset_clicked(bool check);
 void on_button_send_foot_distance_clicked(bool check);

 void on_button_p_walking_init_pose_clicked(bool check);
 void on_button_p_walking_balance_on_clicked(bool check);
 void on_button_p_walking_balance_off_clicked(bool check);

 void on_button_marker_set_clicked(bool check);
 void on_button_marker_clear_clicked(bool check);

 void on_button_footstep_plan_clicked(bool check);
 void on_button_footstep_clear_clicked(bool check);
 void on_button_footstep_go_clicked(bool check);

 void on_dSpinBox_marker_pos_x_valueChanged(double value);
 void on_dSpinBox_marker_pos_y_valueChanged(double value);
 void on_dSpinBox_marker_pos_z_valueChanged(double value);
 void on_dSpinBox_marker_ori_r_valueChanged(double value);
 void on_dSpinBox_marker_ori_p_valueChanged(double value);
 void on_dSpinBox_marker_ori_y_valueChanged(double value);

 // Interactive marker
 void updatePointPanel(const geometry_msgs::Point point);
 void updatePosePanel(const geometry_msgs::Pose pose);

private:
  Ui::PreviewWalkingForm *p_walking_ui;
  robotis_op::QNodeOP3 *qnode_op3_;

  // preview walking
  void sendPWalkingCommand(const std::string &command, bool set_start_foot = true);

  // interactive marker
  void makeInteractiveMarker();
  void updateInteractiveMarker();
  void clearMarkerPanel();

  // update marker UI
  void getPoseFromMarkerPanel(geometry_msgs::Pose &current);
  void setPoseToMarkerPanel(const geometry_msgs::Pose &current);
  void getPointFromMarkerPanel(geometry_msgs::Point &current);
  void setPointToMarkerPanel(const geometry_msgs::Point &current);

  /******************************************
   ** Transformation
   *******************************************/
  Eigen::Vector3d rotation2rpy(const Eigen::MatrixXd &rotation);
  Eigen::MatrixXd rpy2rotation(const double &roll, const double &pitch, const double &yaw);
  Eigen::Quaterniond rpy2quaternion(const Eigen::Vector3d &euler);
  Eigen::Quaterniond rpy2quaternion(const double &roll, const double &pitch, const double &yaw);
  Eigen::Quaterniond rotation2quaternion(const Eigen::MatrixXd &rotation);
  Eigen::Vector3d quaternion2rpy(const Eigen::Quaterniond &quaternion);
  Eigen::Vector3d quaternion2rpy(const geometry_msgs::Quaternion &quaternion);
  Eigen::MatrixXd quaternion2rotation(const Eigen::Quaterniond &quaternion);
  Eigen::MatrixXd rotationX(const double &angle);
  Eigen::MatrixXd rotationY(const double &angle);
  Eigen::MatrixXd rotationZ(const double &angle);

  bool is_updating_;
};

#endif // PREVIEW_WALKING_FORM_H
