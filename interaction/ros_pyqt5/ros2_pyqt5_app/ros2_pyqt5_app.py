#coding utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
from PyQt5.QtWidgets import*
from PyQt5.QtCore import*
from ui.mainwindow_ui import*
import threading
import time
class RclCommNode(Node,QObject):
    signal_recv_msg = pyqtSignal(object)
    def __init__(self):
        Node.__init__(self,"ros2_pyqt5_demo")
        QObject.__init__(self)
        self.i=0
        self.chatter_publisher = self.create_publisher(String,"ros2_qt5_demo",10)
        self.chatter_subcreator=self.create_subscription(String,"ros2_qt5_demo",self.ChatterCallback,10)
    def PubChatter(self,msg):
        chatter_data =  String()
        chatter_data.data=msg
        self.i+=1
        self.chatter_publisher.publish(chatter_data)
    def ChatterCallback(self,data):
        print("i Recv msg:"+data.data)
        self.signal_recv_msg.emit(data.data)

class MainWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setupUi(self)
    def SetLabelText(self,msg):
        self.label.setText(msg)
def ros_spin(node):
    # rclpy.spin(node)
    while True:
        rclpy.spin_once(node)
        time.sleep(0.1)
def main(args=None):
    # 创建节点
    rclpy.init(args=args)
    app=QApplication(sys.argv)
    ros_node= RclCommNode()
    w=MainWindow()
    #显示该窗体
    w.show()
    w.pushButton_send_msg.clicked.connect(lambda: ros_node.PubChatter(w.lineEdit.text()))
    ros_node.signal_recv_msg.connect(w.SetLabelText)
    thread_spin = threading.Thread(target=ros_spin,args=(ros_node,))
    thread_spin.start()
    print("hello ros python")
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
