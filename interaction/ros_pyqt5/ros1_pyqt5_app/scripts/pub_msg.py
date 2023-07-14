#!/usr/bin/env python
# coding:utf-8
import rospy
import sys
from PyQt5.QtWidgets import*
#导入ui的py文件
from ui.mainwindow_ui import *
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
if __name__ == '__main__':
    # 创建节点
    rospy.init_node("rospyqt5")
     #创建qt主程序类
    app = QApplication(sys.argv)
    w = MainWindow()
    #显示该窗体
    w.show()
    ros_node=RosCommNode()
    w.pushButton.clicked.connect(ros_node.send_hello)
    #阻塞主函数 等待APP退出
    print("hello ros python")
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
