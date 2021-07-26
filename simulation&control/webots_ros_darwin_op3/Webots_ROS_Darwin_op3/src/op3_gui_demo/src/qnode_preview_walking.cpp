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

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../include/op3_gui_demo/qnode.hpp"

namespace robotis_op
{

void QNodeOP3::init_preview_walking(ros::NodeHandle &ros_node)
{
  // preview walking
  foot_step_command_pub_ = ros_node.advertise<op3_online_walking_module_msgs::FootStepCommand>("/robotis/online_walking/foot_step_command", 0);
  walking_param_pub_ = ros_node.advertise<op3_online_walking_module_msgs::WalkingParam>("/robotis/online_walking/walking_param", 0);
  set_walking_footsteps_pub_ = ros_node.advertise<op3_online_walking_module_msgs::Step2DArray>(
        "/robotis/online_walking/footsteps_2d", 0);

  body_offset_pub_ = ros_node.advertise<geometry_msgs::Pose>("/robotis/online_walking/body_offset", 0);
  foot_distance_pub_ = ros_node.advertise<std_msgs::Float64>("/robotis/online_walking/foot_distance", 0);
  wholebody_balance_pub_ = ros_node.advertise<std_msgs::String>("/robotis/online_walking/wholebody_balance_msg", 0);
  reset_body_msg_pub_ = ros_node.advertise<std_msgs::Bool>("/robotis/online_walking/reset_body", 0);
  joint_pose_msg_pub_ = ros_node.advertise<op3_online_walking_module_msgs::JointPose>("/robotis/online_walking/goal_joint_pose", 0);

  humanoid_footstep_client_ = ros_node.serviceClient<humanoid_nav_msgs::PlanFootsteps>("plan_footsteps");
  marker_pub_ = ros_node.advertise<visualization_msgs::MarkerArray>("/robotis/demo/foot_step_marker", 0);

  // interacrive marker
  rviz_clicked_point_sub_ = ros_node.subscribe("clicked_point", 0, &QNodeOP3::pointStampedCallback, this);
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("Feet_Pose", "", false));

  ROS_INFO("Initialized node handle for preview walking");
}

bool QNodeOP3::transformPose(const std::string &from_id, const std::string &to_id, const geometry_msgs::Pose &from_pose, geometry_msgs::Pose &to_pose, bool inverse)
{
  tf::StampedTransform desired_transform;

  try
  {
    tf_listener_->lookupTransform(from_id, to_id, ros::Time(0), desired_transform);
    Eigen::Vector3d transform_position(desired_transform.getOrigin().x(),
                                       desired_transform.getOrigin().y(),
                                       desired_transform.getOrigin().z());
    Eigen::Quaterniond transform_orientation(desired_transform.getRotation().w(),
                                             desired_transform.getRotation().x(),
                                             desired_transform.getRotation().y(),
                                             desired_transform.getRotation().z());

    //    desired_transform.
    //    tf::Transform after_tf;
    //    tf::poseMsgToTF(from_pose, after_tf);
    //    if(inverse == false)
    //      after_tf = desired_transform * after_tf;
    //    else
    //      after_tf = after_tf * desired_transform;

    //    tf::poseTFToMsg(after_tf, to_pose);

    //    // except position z
    //    to_pose.position.z = from_pose.position.z;
    Eigen::Vector3d before_position, after_position;
    Eigen::Quaterniond before_orientation, after_orientation;

    tf::pointMsgToEigen(from_pose.position, before_position);
    tf::quaternionMsgToEigen(from_pose.orientation, before_orientation);

    // default : world to local
    if(inverse == false)
    {
      after_position = transform_orientation.inverse().toRotationMatrix() * (before_position - transform_position);
      after_orientation = before_orientation * transform_orientation.inverse();
    }
    else
    {
      after_position = transform_orientation.toRotationMatrix() * before_position + transform_position;
      after_orientation = transform_orientation * before_orientation;
    }

    tf::pointEigenToMsg(after_position, to_pose.position);
    tf::quaternionEigenToMsg(after_orientation, to_pose.orientation);
    //to_pose.position.z = from_pose.position.z;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }

  return true;
}

// demo
void QNodeOP3::pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  ROS_INFO("get position from rviz");

  frame_id_ = msg->header.frame_id;

  // transform : world to local
  geometry_msgs::Pose local_pose, world_pose;
  world_pose.position = msg->point;
  bool result = transformPose("/world", "/body_link", world_pose, local_pose);
  if(result == false)
  {
    log(Warn, "transformation is failed.");
    local_pose = world_pose;
  }

  // update point ui
  //Q_EMIT updateDemoPoint(msg->point);
  Q_EMIT updateDemoPoint(local_pose.position);
}

// interactive marker
void QNodeOP3::interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // event
  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {

      // transform : world to local
      geometry_msgs::Pose local_pose;
      bool result = transformPose("/world", "/body_link", feedback->pose, local_pose);
      if(result == false)
      {
        log(Warn, "transformation is failed.");
        local_pose = feedback->pose;
      }

      current_pose_ = local_pose;

      // update pose ui
      Q_EMIT updateDemoPose(current_pose_);

      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      break;

    default:
      break;
  }

  interactive_marker_server_->applyChanges();
}

void QNodeOP3::makeInteractiveMarker(const geometry_msgs::Pose &marker_pose)
{
  if (frame_id_ == "")
  {
    ROS_ERROR("No frame id!!!");
    // return;

    frame_id_ = "world";
  }

  ROS_INFO_STREAM(
        "Make Interactive Marker! - " << marker_pose.position.x << ", " << marker_pose.position.y << ", " << marker_pose.position.z << " [" << marker_pose.orientation.x << ", " << marker_pose.orientation.y << ", " << marker_pose.orientation.z << " | " << marker_pose.orientation.w << "]");

  interactive_marker_server_->clear();

  // transform : local to world
  geometry_msgs::Pose world_pose;
  bool result = transformPose("/world", "/body_link", marker_pose, world_pose, true);
  if(result == false) world_pose = marker_pose;

  visualization_msgs::InteractiveMarker interactive_marker;
  interactive_marker.pose = world_pose;    // set pose

  // Visualize Interactive Marker
  interactive_marker.header.frame_id = frame_id_;
  interactive_marker.scale = 0.3;

  interactive_marker.name = marker_name_;  //"pose_marker";
  interactive_marker.description = "3D Pose Control";

  // ----- center marker
  visualization_msgs::InteractiveMarkerControl center_marker_control;

  center_marker_control.always_visible = true;
  center_marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;

  // center cube
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;

  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  // axis x
  marker.pose.position.x = 0.05;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  // axis y
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.05;
  marker.pose.position.z = 0.0;

  marker.scale.x = 0.01;
  marker.scale.y = 0.1;
  marker.scale.z = 0.01;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  // axis z
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.05;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.1;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  interactive_marker.controls.push_back(center_marker_control);

  // ----- controller
  visualization_msgs::InteractiveMarkerControl interactive_control;

  // move and rotate along axis x : default
  interactive_control.orientation.x = 1;
  interactive_control.orientation.y = 0;
  interactive_control.orientation.z = 0;
  interactive_control.orientation.w = 1;
  interactive_control.name = "rotate";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(interactive_control);
  interactive_control.name = "move";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(interactive_control);

  // move and rotate along axis y
  interactive_control.orientation.x = 0;
  interactive_control.orientation.y = 1;
  interactive_control.orientation.z = 0;
  interactive_control.orientation.w = 1;
  interactive_control.name = "rotate";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(interactive_control);
  interactive_control.name = "move";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(interactive_control);

  // move and rotate along axis z
  interactive_control.orientation.x = 0;
  interactive_control.orientation.y = 0;
  interactive_control.orientation.z = 1;
  interactive_control.orientation.w = 1;
  interactive_control.name = "rotate";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(interactive_control);
  interactive_control.name = "move";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(interactive_control);

  interactive_marker_server_->insert(interactive_marker);
  interactive_marker_server_->setCallback(interactive_marker.name,
                                          boost::bind(&QNodeOP3::interactiveMarkerFeedback, this, _1));

  interactive_marker_server_->applyChanges();
}

bool QNodeOP3::updateInteractiveMarker(const geometry_msgs::Pose &pose)
{
  ROS_INFO("Update Interactive Marker Pose");

  visualization_msgs::InteractiveMarker interactive_marker;
  bool result_getting = false;

  result_getting = interactive_marker_server_->get(marker_name_, interactive_marker);
  if (result_getting == false)
  {
    ROS_ERROR("No Interactive marker to set pose");
    return false;
  }

  // transform : local to world
  geometry_msgs::Pose world_pose;
  bool result = transformPose("/world", "/body_link", pose, world_pose, true);
  if(result == false) world_pose = pose;

  interactive_marker_server_->setPose(interactive_marker.name, world_pose);
  interactive_marker_server_->applyChanges();

  return true;
}

void QNodeOP3::getInteractiveMarkerPose()
{
  ROS_INFO("Get Interactive Marker Pose");

  visualization_msgs::InteractiveMarker _interactive_marker;
  if (!(interactive_marker_server_->get(marker_name_, _interactive_marker)))
  {
    ROS_ERROR("No Interactive marker to get pose");
    return;
  }

  // transform : world to local
  geometry_msgs::Pose local_pose;
  bool result = transformPose("/world", "/body_link", _interactive_marker.pose, local_pose);
  if(result == false) local_pose = _interactive_marker.pose;

  // update pose ui
  Q_EMIT updateDemoPose(local_pose);

  clearInteractiveMarker();
}

void QNodeOP3::clearInteractiveMarker()
{
  ROS_INFO("Clear Interactive Marker");

  // clear and apply
  interactive_marker_server_->clear();
  interactive_marker_server_->applyChanges();
}

// footstep
void QNodeOP3::setWalkingFootsteps(const double &step_time)
{
  if (preview_foot_steps_.size() != preview_foot_types_.size())
  {
    log(Error, "Footsteps are corrupted.");
    return;
  }
  else if (preview_foot_steps_.size() == 0)
  {
    log(Warn, "No Footsteps");
    return;
  }

  op3_online_walking_module_msgs::Step2DArray footsteps;

  for (int ix = 0; ix < preview_foot_steps_.size(); ix++)
  {
    op3_online_walking_module_msgs::Step2D step;

    step.moving_foot = preview_foot_types_[ix];
    step.step2d = preview_foot_steps_[ix];

    footsteps.footsteps_2d.push_back(step);
  }

  footsteps.step_time = step_time;

  set_walking_footsteps_pub_.publish(footsteps);

  log(Info, "Set command to walk using footsteps");

  //clearFootsteps();
}

void QNodeOP3::clearFootsteps()
{
  // clear foot step marker array
  visualizePreviewFootsteps(true);

  preview_foot_steps_.clear();
  preview_foot_types_.clear();
}

void QNodeOP3::makeFootstepUsingPlanner()
{
  makeFootstepUsingPlanner(current_pose_);
}

void QNodeOP3::makeFootstepUsingPlanner(const geometry_msgs::Pose &target_foot_pose)
{
  //foot step service
  humanoid_nav_msgs::PlanFootsteps get_step;

  geometry_msgs::Pose2D start;
  geometry_msgs::Pose2D goal;
  goal.x = target_foot_pose.position.x;
  goal.y = target_foot_pose.position.y;

  Eigen::Quaterniond goal_orientation;
  tf::quaternionMsgToEigen(target_foot_pose.orientation, goal_orientation);

  Eigen::Vector3d forward, f_x(1, 0, 0);
  forward = goal_orientation.toRotationMatrix() * f_x;
  double theta = forward.y() > 0 ? acos(forward.x()) : -acos(forward.x());
  goal.theta = theta;

  get_step.request.start = start;
  get_step.request.goal = goal;

  std::stringstream call_msg;
  call_msg << "Start [" << start.x << ", " << start.y << " | " << start.theta << "]" << " , Goal [" << goal.x << ", "
           << goal.y << " | " << goal.theta << "]";
  log(Info, call_msg.str());

  // clear visualization
  visualizePreviewFootsteps(true);

  // init foot steps
  preview_foot_steps_.clear();
  preview_foot_types_.clear();

  if (humanoid_footstep_client_.call(get_step))
  {
    if (get_step.response.result)
    {
      for (int ix = 0; ix < get_step.response.footsteps.size(); ix++)
      {
        // foot step log
        int type = get_step.response.footsteps[ix].leg;
        int foot_type;
        std::string foot_string;
        if (type == humanoid_nav_msgs::StepTarget::right)
        {
          foot_type = op3_online_walking_module_msgs::Step2D::RIGHT_FOOT_SWING;
          foot_string = "right";
        }
        else if (type == humanoid_nav_msgs::StepTarget::left)
        {
          foot_type = op3_online_walking_module_msgs::Step2D::LEFT_FOOT_SWING;
          foot_string = "left";
        }
        else
          foot_type = op3_online_walking_module_msgs::Step2D::STANDING;

        std::stringstream msg_stream;
        geometry_msgs::Pose2D foot_pose = get_step.response.footsteps[ix].pose;

        // log footsteps
        msg_stream << "Foot Step #" << ix + 1 << " [ " << foot_string << "] - [" << foot_pose.x << ", " << foot_pose.y
                   << " | " << (foot_pose.theta * 180 / M_PI) << "]";
        log(Info, msg_stream.str());

        preview_foot_steps_.push_back(foot_pose);
        preview_foot_types_.push_back(foot_type);
      }

      double y_feet_offset = 0.186;
      ros::param::get("/footstep_planner/foot/separation", y_feet_offset);
      geometry_msgs::Pose2D target_r_foot_pose, target_l_foot_pose;
      target_r_foot_pose.x = goal.x - (-0.5*y_feet_offset)*sin(theta);
      target_r_foot_pose.y = goal.y + (-0.5*y_feet_offset)*cos(theta);
      target_r_foot_pose.theta = theta;

      target_l_foot_pose.x = goal.x - ( 0.5*y_feet_offset)*sin(theta);
      target_l_foot_pose.y = goal.y + ( 0.5*y_feet_offset)*cos(theta);
      target_l_foot_pose.theta = theta;

      if(preview_foot_types_[preview_foot_types_.size() - 1] == op3_online_walking_module_msgs::Step2D::RIGHT_FOOT_SWING)
      {
        preview_foot_steps_.push_back(target_l_foot_pose);
        preview_foot_types_.push_back(op3_online_walking_module_msgs::Step2D::LEFT_FOOT_SWING);
        preview_foot_steps_.push_back(target_r_foot_pose);
        preview_foot_types_.push_back(op3_online_walking_module_msgs::Step2D::RIGHT_FOOT_SWING);
        preview_foot_steps_.push_back(target_l_foot_pose);
        preview_foot_types_.push_back(op3_online_walking_module_msgs::Step2D::LEFT_FOOT_SWING);
      }
      else if(preview_foot_types_[preview_foot_types_.size() - 1] == op3_online_walking_module_msgs::Step2D::LEFT_FOOT_SWING)
      {
        preview_foot_steps_.push_back(target_r_foot_pose);
        preview_foot_types_.push_back(op3_online_walking_module_msgs::Step2D::RIGHT_FOOT_SWING);
        preview_foot_steps_.push_back(target_l_foot_pose);
        preview_foot_types_.push_back(op3_online_walking_module_msgs::Step2D::LEFT_FOOT_SWING);
      }
      else
      {
        preview_foot_steps_.push_back(target_r_foot_pose);
        preview_foot_types_.push_back(op3_online_walking_module_msgs::Step2D::RIGHT_FOOT_SWING);
        preview_foot_steps_.push_back(target_l_foot_pose);
        preview_foot_types_.push_back(op3_online_walking_module_msgs::Step2D::LEFT_FOOT_SWING);
      }

      // visualize foot steps
      visualizePreviewFootsteps(false);
    }
    else
    {
      log(Info, "fail to get foot step from planner");
      return;
    }
  }
  else
  {
    log(Error, "cannot call service");
    return;
  }

  return;
}

void QNodeOP3::visualizePreviewFootsteps(bool clear)
{
  if (clear && preview_foot_steps_.size() == 0)
    return;

  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();
  visualization_msgs::Marker rviz_marker;

  rviz_marker.header.frame_id = "body_link";
  rviz_marker.header.stamp = now;
  rviz_marker.ns = "foot_step_marker";

  rviz_marker.id = 1;
  rviz_marker.type = visualization_msgs::Marker::CUBE;
  rviz_marker.action = (clear == false) ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

  rviz_marker.scale.x = 0.128;
  rviz_marker.scale.y = 0.08;
  rviz_marker.scale.z = 0.01;

  double alpha = 0.7;
  double height = -0.229;

  geometry_msgs::Pose local_pose, world_pose;
  bool result = transformPose("/world", "/body_link", world_pose, local_pose);
  if(result == true)
    height = local_pose.position.z;

  for (int ix = preview_foot_types_.size() - 1; ix >= 0; ix--)
  {
    // foot step marker array
    rviz_marker.id += 10;

    if (!clear)
    {
      Eigen::Vector3d marker_position(preview_foot_steps_[ix].x, preview_foot_steps_[ix].y, height);
      Eigen::Vector3d marker_position_offset;

      Eigen::Vector3d toward(1, 0, 0), direction(cos(preview_foot_steps_[ix].theta), sin(preview_foot_steps_[ix].theta),
                                                 0);
      Eigen::Quaterniond marker_orientation(Eigen::Quaterniond::FromTwoVectors(toward, direction));

      if (debug_)
      {
        std::stringstream msg;
        msg << "Foot Step #" << ix << " [ " << preview_foot_types_[ix] << "] - [" << rviz_marker.pose.position.x << ", "
            << rviz_marker.pose.position.y << "]";
        log(Info, msg.str());
      }
      alpha *= 0.9;

      // set foot step color
      if (preview_foot_types_[ix] == op3_online_walking_module_msgs::Step2D::LEFT_FOOT_SWING)  // left
      {
        rviz_marker.color.r = 0.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 1.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, 0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;
      }
      else if (preview_foot_types_[ix] == op3_online_walking_module_msgs::Step2D::RIGHT_FOOT_SWING)  //right
      {
        rviz_marker.color.r = 1.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 0.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, -0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;
      }

      marker_position = marker_position_offset + marker_position;

      tf::pointEigenToMsg(marker_position, rviz_marker.pose.position);
      tf::quaternionEigenToMsg(marker_orientation, rviz_marker.pose.orientation);

      // apply foot x offset
    }

    marker_array.markers.push_back(rviz_marker);
  }

  // publish foot step marker array
  if (clear == false)
    log(Info, "Visualize Preview Footstep Marker Array");
  else
    log(Info, "Clear Visualize Preview Footstep Marker Array");

  marker_pub_.publish(marker_array);
}

// Preview walking
void QNodeOP3::sendFootStepCommandMsg(op3_online_walking_module_msgs::FootStepCommand msg)
{
  foot_step_command_pub_.publish(msg);
  log( Info , "Send Foot Step Command Msg" );
}

void QNodeOP3::sendWalkingParamMsg(op3_online_walking_module_msgs::WalkingParam msg)
{
  walking_param_pub_.publish(msg);
  log( Info, "Set Walking Parameter");
}

void QNodeOP3::sendBodyOffsetMsg(geometry_msgs::Pose msg)
{
  body_offset_pub_.publish(msg);
  log( Info, "Send Body Offset");
}

void QNodeOP3::sendFootDistanceMsg(std_msgs::Float64 msg)
{
  foot_distance_pub_.publish(msg);
  log( Info, "Send Foot Distance");
}

void QNodeOP3::sendResetBodyMsg( std_msgs::Bool msg )
{
  reset_body_msg_pub_.publish( msg );
  log( Info , "Reset Body Pose" );
}

void QNodeOP3::sendWholebodyBalanceMsg(std_msgs::String msg)
{
  wholebody_balance_pub_.publish( msg );
  log( Info , "Wholebody Balance Msg" );
}

void QNodeOP3::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Fail to load yaml file. [" << path << "]");
    return;
  }

  op3_online_walking_module_msgs::JointPose msg;

  // parse movement time
  double mov_time = doc["mov_time"].as<double>();
  msg.mov_time = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name = it->first.as<std::string>();
    double value = it->second.as<double>();

    msg.pose.name.push_back(joint_name);
    msg.pose.position.push_back(value * DEG2RAD);
  }

  sendJointPoseMsg( msg );
}

void QNodeOP3::sendJointPoseMsg(op3_online_walking_module_msgs::JointPose msg)
{
  joint_pose_msg_pub_.publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

}
