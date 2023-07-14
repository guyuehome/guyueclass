#encoding:utf-8
from PyQt5 import uic
import sys
from PyQt5.QtWidgets import  *
from PyQt5.QtCore import QObject, pyqtSignal
#创建自定义类，继承于py中的类
class myWindow(QMainWindow):
    def __init__(self):
        super(myWindow,self).__init__()
        #初始化UI
        uic.loadUi('./ui/MainWindow.ui', self)
        self.label.setText("load ui")
if __name__ == '__main__':
    #1,创建qt主程序类
    app = QApplication(sys.argv)
    w= myWindow()
    w.show()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
     
