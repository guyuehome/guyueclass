#include "op3_webots_controller.h"

using namespace webots;
using namespace std;
namespace robotis_op
{

op3_webots_controller::op3_webots_controller()
{
  debug_ = false;
  stop_timer_ = false;
  control_cycle_msec_ = 1; // 1ms control
  time_step_ = getBasicTimeStep();
  motionModule_ = (robotis_framework::MotionModule* )WalkingModule::getInstance();

  ros::NodeHandle ros_node;
  current_module_pub_       = ros_node.advertise<robotis_controller_msgs::JointCtrlModule>(
                                               "/robotis/present_joint_ctrl_modules", 10);
  goal_joint_state_pub_     = ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_states", 10);
  present_joint_state_pub_  = ros_node.advertise<sensor_msgs::JointState>("/robotis/present_joint_states", 10);

  joint_states_sub_   = ros_node.subscribe("/robotis/set_joint_states", 10,
                                                               &op3_webots_controller::setJointStatesCallback, this);
  /* subscribe topics */
  ini_pose_msg_sub_ = ros_node.subscribe("/robotis/base/ini_pose", 10,
                                                        &op3_webots_controller::initPoseMsgCallback,this);

  // Config
  std::string config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/joints_config.yaml";
  parseJointNameFromYaml(config_path);

  motionModule_->initialize(joint_names_, control_cycle_msec_);

}

bool op3_webots_controller::init(){
  for(size_t joint_index = 1; joint_index <= joint_names_.size(); joint_index++){
    string joint_name = joint_names_[joint_index];
    string joint_sensor_name = joint_name + "S";
    joint_motor_map_[joint_name] = getMotor(joint_name);
    joint_motor_map_[joint_name]->setPosition(0.0);
    joint_motor_map_[joint_name]->enableForceFeedback(time_step_);
    joint_motor_map_[joint_name]->enableTorqueFeedback(time_step_);
    joint_sensor_map_[joint_sensor_name] = getPositionSensor(joint_sensor_name);
    joint_sensor_map_[joint_sensor_name]->enable(time_step_);
  }
  step(time_step_);
  return true;
}

bool op3_webots_controller::parseJointNameFromYaml(const std::string &path)
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

bool op3_webots_controller::startTimer(){
  if(is_timer_running_ == true)
    return false;
  webots_timer_thread_ = boost::thread(boost::bind(&op3_webots_controller::webotsTimerThread, this));
  return true;
}

void op3_webots_controller::webotsTimerThread(){
  ros::Rate webots_rate(1000 / control_cycle_msec_);
  while(!stop_timer_){
    process();
    webots_rate.sleep();
  }
}

bool op3_webots_controller::stopTimer(){
  if(is_timer_running_ == true){
    webots_timer_thread_.join();
  }

  // close module such as walking
  while (motionModule_->isRunning())
    usleep(control_cycle_msec_ * 1000);
  motionModule_->setModuleEnable(false);

  stop_timer_ = false;
  is_timer_running_ = true;
}

void op3_webots_controller::process(){
  // avoid duplicated function call
  static bool is_process_running = false;
  if (is_process_running == true)
    return;
  is_process_running = true;

  sensor_msgs::JointState goal_state;
  sensor_msgs::JointState present_state;

  present_state.header.stamp = ros::Time::now();
  goal_state.header.stamp = present_state.header.stamp;

  // update all motor states
  readAllMotors(present_state, goal_state);

  if(motionModule_->isRunning()){
    motionModule_->process(present_state);
    writeAllMotors(motionModule_->desired_joints_state_);
  }


  // -> publish present joint_states & goal joint states topic
  present_joint_state_pub_.publish(present_state);

  is_process_running = false;
  step(time_step_);
}

bool op3_webots_controller::writeAllMotors(const sensor_msgs::JointState &joint_desired_state){
  for(size_t joint_index = 0; joint_index < joint_desired_state.name.size(); joint_index++){
    string joint_name = joint_desired_state.name[joint_index];

    double joint_position = joint_desired_state.position[joint_index];
//    ROS_INFO_THROTTLE(1.0,"%s recieved: %f", joint_name.c_str(), joint_position);
    joint_motor_map_[joint_name]->setPosition(joint_position);
  }

  goal_joint_state_pub_.publish(joint_desired_state);
  return true;
}

bool op3_webots_controller::writeSingleMotor(const std::string& joint_name, double joint_value){
  joint_motor_map_[joint_name]->getMaxPosition();
  joint_motor_map_[joint_name]->getMinPosition();
  joint_motor_map_[joint_name]->setPosition(joint_value);
  return true;
}

bool op3_webots_controller::readAllMotors(sensor_msgs::JointState& joint_present_state,
                                          sensor_msgs::JointState& joint_target_state){
  for(size_t joint_index = 1; joint_index <= joint_names_.size(); joint_index++){
    string joint_name = joint_names_[joint_index];
    string joint_sensor_name = joint_name + "S";

    joint_present_state.name.push_back(joint_name);
    joint_present_state.position.push_back(joint_sensor_map_[joint_sensor_name]->getValue());
    joint_present_state.velocity.push_back(joint_motor_map_[joint_name]->getVelocity());
    joint_present_state.effort.push_back(joint_motor_map_[joint_name]->getTorqueFeedback());

    joint_target_state.name.push_back(joint_name);
    joint_target_state.position.push_back(joint_motor_map_[joint_name]->getTargetPosition());
    joint_target_state.velocity.push_back(joint_motor_map_[joint_name]->getVelocity());
    joint_target_state.effort.push_back(joint_motor_map_[joint_name]->getTorqueFeedback());

//    ROS_INFO_THROTTLE(0.5, "[sensor: %s] reading: %f[pos]", joint_sensor_name.c_str(), joint_present_state.position[joint_index]);

  }
  return true;
}

double op3_webots_controller::readSingleMotor(const string& joint_sensor_name){
  return joint_sensor_map_[joint_sensor_name]->getValue();
}

void op3_webots_controller::setJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  ROS_INFO("recieve");
  writeAllMotors(*msg);
}

void op3_webots_controller::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg){

  if (msg->data == "ini_pose")
  {
    ROS_INFO("init pose");
    motionModule_->setModuleEnable(true);
  }
  return;
}

}
