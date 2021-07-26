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

void QNodeOP3::init_default_demo(ros::NodeHandle &ros_node)
{
  init_gyro_pub_ = ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  set_head_joint_angle_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);

  current_joint_states_sub_ = ros_node.subscribe("/robotis/present_joint_states", 10,
                                                 &QNodeOP3::updateHeadJointStatesCallback, this);

  // Walking
  set_walking_command_pub = ros_node.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub = ros_node.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
  get_walking_param_client_ = ros_node.serviceClient<op3_walking_module_msgs::GetWalkingParam>(
      "/robotis/walking/get_params");

  // Action
  motion_index_pub_ = ros_node.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  // Demo
  demo_command_pub_ = ros_node.advertise<std_msgs::String>("/robotis/demo_command", 0);

  std::string default_motion_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/gui_motion.yaml";
  std::string motion_path = ros_node.param<std::string>("gui_motion", default_motion_path);
  parseMotionMapFromYaml(motion_path);

  ROS_INFO("Initialized node handle for default demo");
}

void QNodeOP3::updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double head_pan, head_tilt;
  int num_get = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      head_pan = -msg->position[ix];
      num_get += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      head_tilt = msg->position[ix];
      num_get += 1;
    }

    if (num_get == 2)
      break;
  }

  if (num_get > 0)
    Q_EMIT updateHeadAngles(head_pan, head_tilt);
}

void QNodeOP3::setHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(-pan);
  head_angle_msg.position.push_back(tilt);

  set_head_joint_angle_pub_.publish(head_angle_msg);
}

// Walking
void QNodeOP3::setWalkingCommand(const std::string &command)
{
  std_msgs::String _commnd_msg;
  _commnd_msg.data = command;
  set_walking_command_pub.publish(_commnd_msg);

  std::stringstream ss_log;
  ss_log << "Set Walking Command : " << _commnd_msg.data << std::endl;

  log(Info, ss_log.str());
}

void QNodeOP3::refreshWalkingParam()
{
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_walking_param_client_.call(walking_param_msg))
  {
    walking_param_ = walking_param_msg.response.parameters;

    // update ui
    Q_EMIT updateWalkingParameters(walking_param_);
    log(Info, "Get walking parameters");
  }
  else
    log(Error, "Fail to get walking parameters.");
}

void QNodeOP3::saveWalkingParam()
{
  std_msgs::String command_msg;
  command_msg.data = "save";
  set_walking_command_pub.publish(command_msg);

  log(Info, "Save Walking parameters.");
}

void QNodeOP3::applyWalkingParam(const op3_walking_module_msgs::WalkingParam &walking_param)
{
  walking_param_ = walking_param;

  set_walking_param_pub.publish(walking_param_);
  log(Info, "Apply Walking parameters.");
}

void QNodeOP3::initGyro()
{
  robotis_controller_msgs::SyncWriteItem init_gyro_msg;
  init_gyro_msg.item_name = "imu_control";
  init_gyro_msg.joint_name.push_back("open-cr");
  init_gyro_msg.value.push_back(0x08);

  init_gyro_pub_.publish(init_gyro_msg);

  log(Info, "Initialize Gyro");
}

// Motion
void QNodeOP3::playMotion(int motion_index)
{
  if (motion_table_.find(motion_index) == motion_table_.end())
  {
    log(Error, "Motion index is not valid.");
    return;
  }

  std::stringstream log_ss;
  switch (motion_index)
  {
    case -2:
      log_ss << "Brake Motion";
      break;

    case -1:
      log_ss << "STOP Motion";
      break;

    default:
      std::string _motion_name = motion_table_[motion_index];
      log_ss << "Play Motion : [" << motion_index << "] " << _motion_name;
  }

  // publish motion index
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);

  log(Info, log_ss.str());
}

// Demo
void QNodeOP3::setDemoCommand(const std::string &command)
{
  std_msgs::String demo_msg;
  demo_msg.data = command;

  demo_command_pub_.publish(demo_msg);

  std::stringstream log_ss;
  log_ss << "Demo command : " << command;
  log(Info, log_ss.str());
}

void QNodeOP3::setActionModuleBody()
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string module_name = "action_module";

  for (int ix = 1; ix <= 18; ix++)
  {
    std::string joint_name;

    if (getJointNameFromID(ix, joint_name) == false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    control_msg.module_name.push_back(module_name);
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  setJointControlMode(control_msg);
}

void QNodeOP3::setModuleToDemo()
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string body_module = "walking_module";
  std::string head_module = "head_control_module";

  for (int ix = 1; ix <= 20; ix++)
  {
    std::string joint_name;

    if (getJointNameFromID(ix, joint_name) == false)
      continue;

    control_msg.joint_name.push_back(joint_name);
    if (ix <= 18)
      control_msg.module_name.push_back(body_module);
    else
      control_msg.module_name.push_back(head_module);

  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  setJointControlMode(control_msg);
}

void QNodeOP3::parseMotionMapFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load motion yaml.");
    return;
  }

  // parse motion_table
  YAML::Node motion_sub_node = doc["motion"];
  for (YAML::iterator yaml_it = motion_sub_node.begin(); yaml_it != motion_sub_node.end(); ++yaml_it)
  {
    int motion_index;
    std::string motion_name;

    motion_index = yaml_it->first.as<int>();
    motion_name = yaml_it->second.as<std::string>();

    motion_table_[motion_index] = motion_name;
  }

  // parse shortcut_table
  YAML::Node _shoutcut_sub_node = doc["motion_shortcut"];
  for (YAML::iterator _it = _shoutcut_sub_node.begin(); _it != _shoutcut_sub_node.end(); ++_it)
  {
    int shortcut_prefix = 0x30;
    int motion_index;
    int shortcut_index;

    motion_index = _it->first.as<int>();
    shortcut_index = _it->second.as<int>();

    if (shortcut_index < 0 || shortcut_index > 9)
      continue;

    motion_shortcut_table_[motion_index] = shortcut_index + shortcut_prefix;
  }
}

}
