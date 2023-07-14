#encoding:utf-8
import sys
from PyQt5.QtWidgets import  *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
#1 导入资源文件
from resource.image_rcc import *
from ui.mainwindow_ui import Ui_MainWindow
#创建自定义类，继承于py中的类
class myWindow(QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(myWindow,self).__init__()
        #初始化UI
        self.setupUi(self)
        pix = QPixmap("://images/noetic.png")
        # pix = QPixmap("./resource/images/noetic.png")
        self.label_image.setGeometry(0,0,300,200)
        self.label_image.setPixmap(pix)
if __name__ == '__main__':
    #1,创建qt主程序类
    app = QApplication(sys.argv)
    w= myWindow()
    w.show()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
