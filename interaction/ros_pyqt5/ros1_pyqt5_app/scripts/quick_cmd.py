#!/usr/bin/env python
# coding:utf-8
import rospy
import sys
from PyQt5.QtWidgets import*
import os
import subprocess
#导入ui的py文件
from ui.quick_cmd_window_ui import *
from std_msgs.msg import String
class RosCommNode():
    def __init__(self):
        self.pub_hello = rospy.Publisher("hello_pyqt5",String,queue_size=10)
    def send_hello(self):
        self.pub_hello.publish("hello pyqt5")

class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
def RunStartSim():
    subprocess.Popen("roslaunch turtlebot3_gazebo turtlebot3_house.launch",shell=True)
def RunStopSim():
    subprocess.Popen("ps -ef|grep gazebo|awk '{print $2}'|xargs kill -9",shell=True)
def RunStartMaping():
    subprocess.Popen("roslaunch turtlebot3_slam turtlebot3_gmapping.launch",shell=True)
def RunStopMaping():
    subprocess.Popen("ps -ef|grep turtlebot3_slam|awk '{print $2}'|xargs kill -9",shell=True)
    subprocess.Popen("rosrun map_server map_saver -f /home/chengyangkj/map",shell=True)
if __name__ == '__main__':
    # os.system("roscore&")
    subprocess.Popen("roscore",shell=True)
    # 创建节点
    rospy.init_node("rospyqt5")
     #创建qt主程序类
    app = QApplication(sys.argv)
    w = MainWindow()
    #显示该窗体
    w.show()
    w.pushButton_start_sim.clicked.connect(RunStartSim)
    w.pushButton_end_sim.clicked.connect(RunStopSim)
    w.pushButton_start_mapping.clicked.connect(RunStartMaping)
    w.pushButton_end_mapping.clicked.connect(RunStopMaping)
    ros_node=RosCommNode()
    #阻塞主函数 等待APP退出
    print("hello ros python")
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
