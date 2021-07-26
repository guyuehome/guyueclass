/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/op3_webots_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_webots_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{
  debug_ = false;
}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"op3_webots_gui");
  if ( ! ros::master::check() ) {
    ROS_INFO("Master is not online");

    logging_model_.insertRows(logging_model_.rowCount(), 1);
    QVariant new_row(QString("ROS Master is not online"));
    logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
    Q_EMIT loggingUpdated();  // used to readjust the scrollbar

    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle ros_node;

  desired_joint_state_pub_     = ros_node.advertise<sensor_msgs::JointState>("/robotis/set_joint_states", 10);
  init_pose_pub_ = ros_node.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  // Walking
  set_walking_command_pub = ros_node.advertise<std_msgs::String>("/robotis/walking/command", 0);
  set_walking_param_pub = ros_node.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
  get_walking_param_client_ = ros_node.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params");


  // Config
  std::string config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/joints_config.yaml";
  parseJointNameFromYaml(config_path);

  // start time
  start_time_ = ros::Time::now();

  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(1);
  int count = 0;
  while ( ros::ok() ) {

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;

  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

}

bool QNode::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return false;
  }
  // parse joint names and ids
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = id_sub_node.begin(); _it != id_sub_node.end(); ++_it)
  {
    int joint_id;
    std::string joint_name;

    joint_id = _it->first.as<int>();
    joint_name = _it->second.as<std::string>();
    joint_names_[joint_id] = joint_name;

    if (debug_)
      std::cout << "ID : " << joint_id << " - " << joint_name << std::endl;
  }
  return true;
}
// move ini pose : wholedody module
void QNode::moveInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_.publish(init_msg);

  log(Info, "Go to robot initial pose.");
}
int QNode::getJointSize(){
  return joint_names_.size();
}

std::string QNode::getJointNameFromIndex(int joint_index){
  if(joint_index < 1 || joint_index > joint_names_.size()){
    ROS_ERROR("joint index[%d] out of index ", joint_index);
    return "";
  }
  return joint_names_[size_t(joint_index)];
}
void QNode::sendJointValue(int joint_index, double joint_value){
  if(joint_names_.find(joint_index) == joint_names_.end()){
    ROS_ERROR("sendJointValue index error");
    return ;
  }
  sensor_msgs::JointState desired_joint_state;
  desired_joint_state.name.push_back(joint_names_[(size_t)joint_index]);
  desired_joint_state.position.push_back(joint_value  * M_PI / 180);
  desired_joint_state.velocity.push_back(0);
  desired_joint_state.effort.push_back(0);
  //
  desired_joint_state_pub_.publish(desired_joint_state);
}


// Walking
bool QNode::setWalkingCommand(const std::string &command)
{
  std_msgs::String _commnd_msg;
  _commnd_msg.data = command;
  set_walking_command_pub.publish(_commnd_msg);

  std::stringstream ss_log;
  ss_log << "Set Walking Command : " << _commnd_msg.data << std::endl;

  log(Info, ss_log.str());
  return true;
}

bool QNode::refreshWalkingParam()
{
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_walking_param_client_.call(walking_param_msg))
  {
    walking_param_ = walking_param_msg.response.parameters;

    // update ui
    Q_EMIT updateWalkingParameters(walking_param_);
    log(Info, "Get walking parameters");
    return true;
  }
  else{
    log(Error, "Fail to get walking parameters.");
    return false;
  }
}

bool QNode::saveWalkingParam()
{
  std_msgs::String command_msg;
  command_msg.data = "save";
  set_walking_command_pub.publish(command_msg);

  log(Info, "Save Walking parameters.");
  return true;
}

bool QNode::applyWalkingParam(const op3_walking_module_msgs::WalkingParam &walking_param)
{
  walking_param_ = walking_param;

  set_walking_param_pub.publish(walking_param_);
  log(Info, "Apply Walking parameters.");
  return true;
}

// LOG
void QNode::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNode::log(const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;

  ros::Duration duration_time = ros::Time::now() - start_time_;
  int current_time = duration_time.sec;
  int min_time = 0, sec_time = 0;
  min_time = (int) (current_time / 60);
  sec_time = (int) (current_time % 60);

  std::stringstream min_str, sec_str;
  if (min_time < 10)
    min_str << "0";
  if (sec_time < 10)
    sec_str << "0";
  min_str << min_time;
  sec_str << sec_time;

  std::stringstream sender_ss;
  sender_ss << "[" << sender << "] ";

  switch (level)
  {
    case (Debug):
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Info):
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Warn):
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[WARN] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Error):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "<ERROR> [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (Fatal):
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void QNode::clearLog()
{
  if (logging_model_.rowCount() == 0)
    return;

  logging_model_.removeRows(0, logging_model_.rowCount());
}


}  // namespace op3_webots_gui
