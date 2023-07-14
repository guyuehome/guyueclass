#encoding:utf-8
import sys
from PyQt5.QtWidgets import  QApplication,QWidget,QPushButton,QLabel
if __name__ == '__main__':
    #1,创建qt主程序类
    app = QApplication(sys.argv)
    #2,创建widge窗体
    w = QWidget()
    #3,设置窗名称
    w.setWindowTitle("PushButton")
    #4,创建btn对象
    label = QLabel("我是一个label文本框")
    #5,设置btn的父级对象
    label.setParent(w)
    #设置label的显示位置及大小 x y w h
    label.setGeometry(50,50,400,40)
    w.show()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
