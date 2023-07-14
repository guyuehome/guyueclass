#encoding:utf-8
from cProfile import label
import sys
from PyQt5.QtWidgets import  QApplication,QWidget,QPushButton,QLabel,QLineEdit,QVBoxLayout,QHBoxLayout
if __name__ == '__main__':
    #1,创建qt主程序类
    app = QApplication(sys.argv)
    #2,创建widge窗体
    w = QWidget()
    w.resize(500,500)
    btn_one=QPushButton("one")
    btn_two=QPushButton("two")
    btn_three=QPushButton("three")
    btn_four=QPushButton("four")
    #创建垂直布局对象
    box_layout=QVBoxLayout()
    #依次添加按钮
    box_layout.addWidget(btn_one)
    box_layout.addWidget(btn_two)
    box_layout.addWidget(btn_three)
    box_layout.addWidget(btn_four)
    #添加一个弹簧
    box_layout.addStretch(1)
    #主窗体设置layout
    w.setLayout(box_layout)
    w.show()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
