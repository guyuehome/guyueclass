#!/usr/bin/env python
# coding:utf-8
import rospy
import sys
from PyQt5.QtWidgets import*
#导入ui的py文件
from ui.mainwindow_ui import *
from std_msgs.msg import String
from PyQt5.QtCore import *
import threading
class RosCommNode(QObject):
    signal_msg=pyqtSignal(object)
    def __init__(self):
        super(RosCommNode,self).__init__()
        self.pub_hello = rospy.Publisher("hello_pyqt5",String,queue_size=10)
        self.sub_info = rospy.Subscriber("hello_pyqt5",String,self.info_callback)
    def send_hello(self,msg):
        self.pub_hello.publish(msg)
    def info_callback(self,msg):
        print("i recv:"+msg.data)
        self.signal_msg.emit(msg.data)
class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
    def set_label_text(self,msg):
        self.label_msg.setText(msg)
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
    w.pushButton.clicked.connect(lambda:ros_node.send_hello(w.lineEdit_input.text()))
    #连接话题信号
    ros_node.signal_msg.connect(w.set_label_text)
    #阻塞主函数 等待APP退出
    print("hello ros python")
    th_apin= threading.Thread(target=run_spin)
    th_apin.start()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
