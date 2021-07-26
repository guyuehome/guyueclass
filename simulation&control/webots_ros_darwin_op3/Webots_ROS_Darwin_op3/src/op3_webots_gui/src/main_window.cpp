/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/op3_webots_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_webots_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent),
    qnode(argc,argv),
    is_updating_(false),
    is_walking_(false)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  setWindowTitle("Webots Robotis-OP3 Joint Contorller");
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  QObject::connect(&qnode, SIGNAL(rosNoMaster()), this, SLOT(showNoMasterMessage()));

  qRegisterMetaType<op3_walking_module_msgs::WalkingParam>("op_walking_params");
  QObject::connect(&qnode, SIGNAL(updateWalkingParameters(op3_walking_module_msgs::WalkingParam)), this,
                   SLOT(updateWalkingParams(op3_walking_module_msgs::WalkingParam)));

  /*********************
   ** Logging
   **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  //初始化使能

  ui.button_param_apply->setEnabled(false);

  ui.tab_manager->setCurrentIndex(0);
  ui.tab_walking_module->setEnabled(false);
  /*********************
   ** Auto Start
   **********************/
  qnode.init();

  initJointSliders();

}

MainWindow::~MainWindow() {}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_clear_log_clicked(bool check)
{
  qnode.clearLog();
}
void MainWindow::on_button_init_pose_clicked(bool check)
{
  qnode.moveInitPose();
  ui.tab_walking_module->setEnabled(true);
}


// Walking
void MainWindow::on_button_init_gyro_clicked(bool check)
{
  //  qnode.initGyro();
}

void MainWindow::on_button_walking_start_clicked(bool check)
{
  is_walking_ = true;
  qnode.setWalkingCommand("start");
}

void MainWindow::on_button_walking_stop_clicked(bool check)
{
  is_walking_ = false;
  qnode.setWalkingCommand("stop");
}

void MainWindow::on_button_param_refresh_clicked(bool check)
{
  ROS_INFO("refresh walking param");

  if(qnode.refreshWalkingParam()){
    ui.button_param_apply->setEnabled(true);
  }else{

  }
}

void MainWindow::on_button_param_save_clicked(bool check)
{
  ROS_INFO("save walking param");
  qnode.setWalkingCommand("save");
}

void MainWindow::on_button_param_apply_clicked(bool check)
{
  applyWalkingParams();
}


void MainWindow::initJointSliders(){
  // joints
  QGridLayout *grid_layout = new QGridLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);

  for (int ix = 0; ix < qnode.getJointSize(); ix++)
  {
    std::stringstream label_stream;
    std::string joint_name = qnode.getJointNameFromIndex(ix + 1);
    //    std::string joint_name = ix + " ";
    QLabel *id_label = new QLabel(tr(joint_name.c_str()));

    QSpinBox* spinBox = new QSpinBox(ui.widget_joint_control);

    spinBox->setSuffix(" \260");
    spinBox->setRange(-90, 90);
    spinBox->setSingleStep(1);
    QSlider *slider = new QSlider(this);
    slider->setOrientation(Qt::Horizontal);  // 水平方向
    slider->setMinimum(-90);  // 最小值
    slider->setMaximum(90);  // 最大值
    slider->setValue(0);
    slider->setSingleStep(1);  // 步长
    slider->setObjectName(QString("slider %1").arg(ix + 1));
    // 连接信号槽（相互改变）
    //    connect(pSpinBox, SIGNAL(valueChanged(int)), pSlider, SLOT(setValue(int)));
    signalMapper->setMapping(slider, slider);
    QObject::connect(slider, SIGNAL(valueChanged(int)), signalMapper, SLOT(map()));

    QObject::connect(spinBox, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
    QObject::connect(slider, SIGNAL(valueChanged(int)), spinBox, SLOT(setValue(int)));


    int num_row = (ix / 2) * 2 + 1;
    int num_col = (ix % 2) * 3;

    grid_layout->addWidget(slider, num_row, (ix % 2) + num_col, 1, 3);
//    ROS_INFO("slider   ix: %d num_row:%d , num_col: %d", ix % 2, num_row, (ix % 2) + num_col);
    grid_layout->addWidget(id_label, num_row + 1, (ix % 2) + num_col, 1, 1);
//    ROS_INFO("id_label ix: %d num_row:%d , num_col: %d", ix % 2, num_row  + 1, (ix % 2) + num_col);
    grid_layout->addWidget(spinBox, num_row + 1, (ix % 2) + num_col + 1, 1, 2);
//    ROS_INFO("spinBox  ix: %d num_row:%d , num_col: %d", ix % 2, num_row  + 1, (ix % 2) + num_col + 1);
  }
  QObject::connect(signalMapper, SIGNAL(mapped(QWidget *)), this, SLOT(setJointAngle(QWidget *)));
  ui.widget_joint_control->setLayout(grid_layout);
}


void MainWindow::setJointAngle(QWidget *widget)
{
  QSlider *slider = qobject_cast<QSlider *>(widget);
  slider->value();
  QStringList list = slider->objectName().split(" ");
  if(list.size() >= 2){
    int joint_index = list[1].toInt();
    qnode.sendJointValue(joint_index, slider->value());
//    ROS_INFO("send joint [%d] value: %d", joint_index, slider->value());
  }

}


void MainWindow::updateLoggingView()
{
  ui.view_logging->scrollToBottom();
}

// user shortcut
void MainWindow::setUserShortcut()
{
  // Setup a signal mapper to avoid creating custom slots for each tab
  QSignalMapper *_sig_map = new QSignalMapper(this);

  // Setup the shortcut for the first tab : Mode
  QShortcut *_short_tab1 = new QShortcut(QKeySequence("F1"), this);
  connect(_short_tab1, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab1, 0);

  // Setup the shortcut for the second tab : Manipulation
  QShortcut *_short_tab2 = new QShortcut(QKeySequence("F2"), this);
  connect(_short_tab2, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab2, 1);

  // Setup the shortcut for the third tab : Walking
  QShortcut *_short_tab3 = new QShortcut(QKeySequence("F3"), this);
  connect(_short_tab3, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab3, 2);

  // Setup the shortcut for the fouth tab : Head control
  QShortcut *_short_tab4 = new QShortcut(QKeySequence("F4"), this);
  connect(_short_tab4, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab4, 3);

  // Setup the shortcut for the fouth tab : Motion
  QShortcut *_short_tab5 = new QShortcut(QKeySequence("F5"), this);
  connect(_short_tab5, SIGNAL(activated()), _sig_map, SLOT(map()));
  _sig_map->setMapping(_short_tab5, 4);

  // Wire the signal mapper to the tab widget index change slot
  connect(_sig_map, SIGNAL(mapped(int)), ui.tabWidget_control, SLOT(setCurrentIndex(int)));

  QShortcut *walking_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  connect(walking_shortcut, SIGNAL(activated()), this, SLOT(walkingCommandShortcut()));
}


// walking
void MainWindow::updateWalkingParams(op3_walking_module_msgs::WalkingParam params)
{
  // init pose
  ui.dSpinBox_init_offset_x->setValue(params.init_x_offset);
  ui.dSpinBox_init_offset_y->setValue(params.init_y_offset);
  ui.dSpinBox_init_offset_z->setValue(params.init_z_offset);
  ui.dSpinBox_init_offset_roll->setValue(params.init_roll_offset * RADIAN2DEGREE);
  ui.dSpinBox_init_offset_pitch->setValue(params.init_pitch_offset * RADIAN2DEGREE);
  ui.dSpinBox_init_offset_yaw->setValue(params.init_yaw_offset * RADIAN2DEGREE);
  ui.dSpinBox_hip_pitch_offset->setValue(params.hip_pitch_offset * RADIAN2DEGREE);
  // time
  ui.dSpinBox_period_time->setValue(params.period_time * 1000);       // s -> ms
  ui.dSpinBox_dsp_ratio->setValue(params.dsp_ratio);
  ui.dSpinBox_step_fb_ratio->setValue(params.step_fb_ratio);

  // walking
  ui.dSpinBox_x_move_amplitude->setValue(params.x_move_amplitude);
  ui.dSpinBox_y_move_amplitude->setValue(params.y_move_amplitude);
  ui.dSpinBox_z_move_amplitude->setValue(params.z_move_amplitude);
  ui.dSpinBox_y_move_amplitude->setValue(params.angle_move_amplitude);
  ui.checkBox_move_aim_on->setChecked(params.move_aim_on);
  ui.checkBox_move_aim_off->setChecked(!params.move_aim_on);
  // balance
  ui.checkBox_balance_on->setChecked(params.balance_enable);
  ui.checkBox_balance_off->setChecked(!params.balance_enable);
  ui.dSpinBox_hip_roll_gain->setValue(params.balance_hip_roll_gain);
  ui.dSpinBox_knee_gain->setValue(params.balance_knee_gain);
  ui.dSpinBox_ankle_roll_gain->setValue(params.balance_ankle_roll_gain);
  ui.dSpinBox_ankle_pitch_gain->setValue(params.balance_ankle_pitch_gain);
  ui.dSpinBox_y_swap_amplitude->setValue(params.y_swap_amplitude);
  ui.dSpinBox_z_swap_amplitude->setValue(params.z_swap_amplitude);
  ui.dSpinBox_pelvis_offset->setValue(params.pelvis_offset * RADIAN2DEGREE);
  ui.dSpinBox_arm_swing_gain->setValue(params.arm_swing_gain);
}

void MainWindow::applyWalkingParams()
{
  op3_walking_module_msgs::WalkingParam walking_param;

  // init pose
  walking_param.init_x_offset = ui.dSpinBox_init_offset_x->value();
  walking_param.init_y_offset = ui.dSpinBox_init_offset_y->value();
  walking_param.init_z_offset = ui.dSpinBox_init_offset_z->value();
  walking_param.init_roll_offset = ui.dSpinBox_init_offset_roll->value() * DEGREE2RADIAN;
  walking_param.init_pitch_offset = ui.dSpinBox_init_offset_pitch->value() * DEGREE2RADIAN;
  walking_param.init_yaw_offset = ui.dSpinBox_init_offset_yaw->value() * DEGREE2RADIAN;
  walking_param.hip_pitch_offset = ui.dSpinBox_hip_pitch_offset->value() * DEGREE2RADIAN;
  // time
  walking_param.period_time = ui.dSpinBox_period_time->value() * 0.001;     // ms -> s
  walking_param.dsp_ratio = ui.dSpinBox_dsp_ratio->value();
  walking_param.step_fb_ratio = ui.dSpinBox_step_fb_ratio->value();
  ;
  // walking
  walking_param.x_move_amplitude = ui.dSpinBox_x_move_amplitude->value();
  walking_param.y_move_amplitude = ui.dSpinBox_y_move_amplitude->value();
  walking_param.z_move_amplitude = ui.dSpinBox_z_move_amplitude->value();
  walking_param.angle_move_amplitude = ui.dSpinBox_a_move_amplitude->value() * DEGREE2RADIAN;
  walking_param.move_aim_on = ui.checkBox_move_aim_on->isChecked();
  // balance
  walking_param.balance_enable = ui.checkBox_balance_on->isChecked();
  walking_param.balance_hip_roll_gain = ui.dSpinBox_hip_roll_gain->value();
  walking_param.balance_knee_gain = ui.dSpinBox_knee_gain->value();
  walking_param.balance_ankle_roll_gain = ui.dSpinBox_ankle_roll_gain->value();
  walking_param.balance_ankle_pitch_gain = ui.dSpinBox_ankle_pitch_gain->value();
  walking_param.y_swap_amplitude = ui.dSpinBox_y_swap_amplitude->value();
  walking_param.z_swap_amplitude = ui.dSpinBox_z_swap_amplitude->value();
  walking_param.pelvis_offset = ui.dSpinBox_pelvis_offset->value() * DEGREE2RADIAN;
  walking_param.arm_swing_gain = ui.dSpinBox_arm_swing_gain->value();

  qnode.applyWalkingParam(walking_param);
}


void MainWindow::walkingCommandShortcut()
{
  if (is_walking_ == true)
  {
    is_walking_ = false;
    qnode.setWalkingCommand("stop");
  }
  else
  {
    is_walking_ = true;
    qnode.setWalkingCommand("start");
  }
}
/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  ROS_INFO("Show no massage");
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_connect_clicked(bool check ) {
//  if ( ui.checkbox_use_environment->isChecked() ) {
//    if ( !qnode.init() ) {
//      showNoMasterMessage();
//    } else {
//      ui.button_connect->setEnabled(false);
//    }
//  } else {
//    if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
//           ui.line_edit_host->text().toStdString()) ) {
//      showNoMasterMessage();
//    } else {
//      ui.button_connect->setEnabled(false);
//      ui.line_edit_master->setReadOnly(true);
//      ui.line_edit_host->setReadOnly(true);
//      ui.line_edit_topic->setReadOnly(true);
//    }
//  }
//}


//void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
//  bool enabled;
//  if ( state == 0 ) {
//    enabled = true;
//  } else {
//    enabled = false;
//  }
//  ui.line_edit_master->setEnabled(enabled);
//  ui.line_edit_host->setEnabled(enabled);
//  //ui.line_edit_topic->setEnabled(enabled);
//}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
//void MainWindow::updateLoggingView() {
//        ui.view_logging->scrollToBottom();
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
  QSettings settings("Qt-Ros Package", "op3_webots_gui");
  //    restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
  QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
  //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();

  //ui.line_edit_topic->setText(topic_name);
  //    bool remember = settings.value("remember_settings", false).toBool();
  //    bool checked = settings.value("use_environment_variables", false).toBool();

}

void MainWindow::WriteSettings() {
  QSettings settings("Qt-Ros Package", "op3_webots_gui");

  //settings.setValue("topic_name",ui.line_edit_topic->text());
  //    settings.setValue("geometry", saveGeometry());
  //    settings.setValue("windowState", saveState());


}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace op3_webots_gui

