/**
 * @file /include/op3_webots_gui/main_window.hpp
 *
 * @brief Qt based gui for op3_webots_gui.
 *
 * @date November 2010
 **/
#ifndef op3_webots_gui_MAIN_WINDOW_H
#define op3_webots_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace op3_webots_gui {
#define DEGREE2RADIAN     (M_PI / 180.0)
#define RADIAN2DEGREE     (180.0 / M_PI)
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void parseJointNameFromYaml(const std::string &path);
  void initJointSliders();

  void ReadSettings(); // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

  void closeEvent(QCloseEvent *event); // Overloaded function


public Q_SLOTS:

  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_button_clear_log_clicked(bool check);
  void on_button_init_pose_clicked(bool check);

  void setJointAngle(QWidget *widget);
  //	void on_button_connect_clicked(bool check );
  //	void on_checkbox_use_environment_stateChanged(int state);
  // Walking
  void on_button_init_gyro_clicked(bool check);
  void on_button_walking_start_clicked(bool check);
  void on_button_walking_stop_clicked(bool check);

  void on_button_param_refresh_clicked(bool check);
  void on_button_param_apply_clicked(bool check);
  void on_button_param_save_clicked(bool check);

  /******************************************
    ** Manual connections
    *******************************************/
  //    void updateLoggingView(); // no idea why this can't connect automatically
  // Walking
  void updateWalkingParams(op3_walking_module_msgs::WalkingParam params);
  void walkingCommandShortcut();

  void updateLoggingView();

  void showNoMasterMessage();

private:

  void setUserShortcut();

  void applyWalkingParams();

  Ui::MainWindowDesign ui;
  QNode qnode;
  bool debug_;
  bool is_updating_;
  bool is_walking_;

};

}  // namespace op3_webots_gui

#endif // op3_webots_gui_MAIN_WINDOW_H
