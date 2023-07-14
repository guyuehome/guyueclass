#!/usr/bin/env python
# coding:utf-8
import rospy
import sys
from PyQt5.QtWidgets import*
#导入ui的py文件
from ui.mainwindow_ui import *
from std_msgs.msg import String
import threading
class RosCommNode():
    def __init__(self):
        self.pub_hello = rospy.Publisher("hello_pyqt5",String,queue_size=10)
        self.sub_info = rospy.Subscriber("hello_pyqt5",String,self.info_callback)
    def send_hello(self):
        self.pub_hello.publish("hello pyqt5")
    def info_callback(self,msg):
        print("i recv:"+msg.data)
class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
def run_spin():
    print("run spin")
    # rospy.spin()
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
    th_apin= threading.Thread(target=run_spin)
    th_apin.start()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
