#!/usr/bin/env python
# coding:utf-8
import rospy
import sys
from PyQt5.QtWidgets import*
from PyQt5.QtCore import *
#导入ui的py文件
from ui.mainwindow_ui import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ui.dashboard import *
class RosCommNode(QObject):
    signal_odom = pyqtSignal(object)
    def __init__(self):
        super(RosCommNode,self).__init__()
        self.pub_hello = rospy.Publisher("hello_pyqt5",String,queue_size=10)
        self.sub_odom =rospy.Subscriber("odom",Odometry,self.OdomCallback)
       
    def send_hello(self):
        self.pub_hello.publish("hello pyqt5")
    def OdomCallback(self,msg):
        print("speed x:"+str(msg.twist.twist.linear.x))
        self.signal_odom.emit(msg.twist.twist.linear.x)
class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
        self.speed_dash =  Dashboard(self.widget_dashboard)
        self.speed_dash.setGeometry(0,0,500,500)
        self.speed_dash.setValue(0)
        self.resize(1000,1000)
    def SetSpeedDashValue(self,value):
        self.speed_dash.setValue(abs(value*100))
        print("set speed x:"+str(abs(value*100)))
if __name__ == '__main__':
    # 创建节点
    rospy.init_node("speed_dashboard")
     #创建qt主程序类
    app = QApplication(sys.argv)
    w = MainWindow()
    #显示该窗体
    w.show()
    ros_node=RosCommNode()
    w.pushButton.clicked.connect(ros_node.send_hello)
    ros_node.signal_odom.connect(w.SetSpeedDashValue)
    #阻塞主函数 等待APP退出
    print("hello ros python")
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
