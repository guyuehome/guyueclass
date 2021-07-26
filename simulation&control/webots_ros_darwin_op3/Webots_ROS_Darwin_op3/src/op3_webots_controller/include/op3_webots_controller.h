#ifndef OP3_WEBOTS_CONTROLLER_H
#define OP3_WEBOTS_CONTROLLER_H

#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Speaker.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <map>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "op3_walking_module/op3_walking_module.h"
#include "robotis_framework_common/motion_module.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <ros/package.h>

#include <yaml-cpp/yaml.h>

#include <boost/thread.hpp>

namespace robotis_op
{
class op3_webots_controller : public webots::Robot
{
public:
  op3_webots_controller();
  bool parseJointNameFromYaml(const std::string &path);
  bool init();
  bool startTimer();
  bool stopTimer();
  bool readAllMotors(sensor_msgs::JointState& joint_present_state,
                     sensor_msgs::JointState& joint_target_state);
  double readSingleMotor(const std::string& joint_sensor_name);
  bool writeAllMotors(const sensor_msgs::JointState &joint_desired_state);
  bool writeSingleMotor(const std::string& joint_name, double joint_value);
  void setJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
private:
  void webotsTimerThread();
  void process();
  boost::thread   webots_timer_thread_;
  bool stop_timer_;
  bool is_timer_running_;
  double time_step_;
  int control_cycle_msec_;
  std::map<int, std::string> joint_names_;
  std::map<std::string, webots::Motor*> joint_motor_map_;
  std::map<std::string, webots::PositionSensor*> joint_sensor_map_;
  bool debug_;

  robotis_framework::MotionModule* motionModule_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber ini_pose_msg_sub_;

  ros::Publisher current_module_pub_;
  ros::Publisher goal_joint_state_pub_;
  ros::Publisher present_joint_state_pub_;
};
}


#endif // OP3_WEBOTS_CONTROLLER_H
