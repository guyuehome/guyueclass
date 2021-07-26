/**
 * @file /include/op3_webots_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef op3_webots_gui_QNODE_HPP_
#define op3_webots_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>

#include "sensor_msgs/JointState.h"

#include "robotis_controller_msgs/StatusMsg.h"

// walking demo
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

#include <yaml-cpp/yaml.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>




/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_webots_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
public:
  enum LogLevel
  {
    Debug = 0,
    Info = 1,
    Warn = 2,
    Error = 3,
    Fatal = 4
  };
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();

  void run();

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "Gui");
  void clearLog();


  bool parseJointNameFromYaml(const std::string &path);
  void sendJointValue(int joint_index, double joint_value);
  int getJointSize();
  std::string getJointNameFromIndex(int joint_index);
  void moveInitPose();

  // Walking
  bool setWalkingCommand(const std::string &command);
  bool refreshWalkingParam();
  bool saveWalkingParam();
  bool applyWalkingParam(const op3_walking_module_msgs::WalkingParam &walking_param);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void rosNoMaster();

  // Walking
  void updateWalkingParameters(op3_walking_module_msgs::WalkingParam params);

private:
  void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);

  int init_argc;
  char** init_argv;
  std::map<int, std::string> joint_names_;
  bool debug_;

  op3_walking_module_msgs::WalkingParam walking_param_;

  ros::Publisher init_pose_pub_;
  ros::Publisher desired_joint_state_pub_;

  // Walking
  ros::Publisher set_walking_command_pub;
  ros::Publisher set_walking_param_pub;
  ros::ServiceClient get_walking_param_client_;

  ros::Time start_time_;

  QStringListModel logging_model_;

};

}  // namespace op3_webots_gui

#endif /* op3_webots_gui_QNODE_HPP_ */
