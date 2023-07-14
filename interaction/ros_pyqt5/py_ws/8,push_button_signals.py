#encoding:utf-8
import sys
#1,导入QPushButton
from PyQt5.QtWidgets import  QApplication,QWidget,QPushButton
def show_clicked():
    print("clicked")
def show_pressed():
    print("pressed")
def show_release():
    print("release")
if __name__ == '__main__':
    #2,创建qt主程序类
    app = QApplication(sys.argv)
    #3,创建widge窗体
    w = QWidget()
    #4,设置窗名称
    w.setWindowTitle("PushButton")
    #5,创建btn对象
    btn = QPushButton("button")
    #6,设置btn的父级对象
    btn.setParent(w)
    #7,显示该窗体
    w.show()
    #点击按钮信号
    btn.clicked.connect(show_clicked)
    #按下信号
    btn.pressed.connect(show_pressed)
    #松开信号
    btn.released.connect(show_release)
    sys.exit(app.exec_())

