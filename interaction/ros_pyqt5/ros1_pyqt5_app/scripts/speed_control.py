#!/usr/bin/env python
# coding:utf-8
import rospy
import sys
from PyQt5.QtWidgets import*
#导入ui的py文件
from ui.speed_control_ui import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
class RosCommNode():
    def __init__(self):
        self.pub_hello = rospy.Publisher("hello_pyqt5",String,queue_size=10)
        self.pub_cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.speed_vx=0
        self.speed_vw=0
        timer_speed = threading.Timer(0.1,self.timer_send_speed)
        timer_speed.start()
    def timer_send_speed(self):
        send_msg = Twist()
        send_msg.linear.x=self.speed_vx
        send_msg.angular.z=self.speed_vw
        self.pub_cmd_vel.publish(send_msg)
        print("send msg")
        if not rospy.is_shutdown():
            timer_speed = threading.Timer(0.1,self.timer_send_speed)
            timer_speed.start()
    def addSpeedVx(self):
        self.speed_vx+=0.02
        if self.speed_vx >= 0.2:
            self.speed_vx=0.2
    def decreaseSpeedVx(self):
        self.speed_vx-=0.02
    def addSpeedVw(self):
        self.speed_vw+=0.02
    def decreaseSpeedVw(self):
        self.speed_vw-=0.02
        if self.speed_vw>=1:
            self.speed_vw=1
    def stopSpeed(self):
        self.speed_vw=0
        self.speed_vx=0
class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
if __name__ == '__main__':
    # 创建节点
    rospy.init_node("speed_control")
     #创建qt主程序类
    app = QApplication(sys.argv)
    w = MainWindow()
    #显示该窗体
    w.show()

    ros_node=RosCommNode()
    w.pushButton_front.clicked.connect(ros_node.addSpeedVx)
    w.pushButton_back.clicked.connect(ros_node.decreaseSpeedVx)
    w.pushButton_left.clicked.connect(ros_node.addSpeedVw)
    w.pushButton_right.clicked.connect(ros_node.decreaseSpeedVw)
    w.pushButton_stop.clicked.connect(ros_node.stopSpeed)
    #阻塞主函数 等待APP退出
    print("hello ros python")
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
