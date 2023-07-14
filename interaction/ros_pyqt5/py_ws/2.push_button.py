#encoding:utf-8
import sys
#1,导入QPushButton
from PyQt5.QtWidgets import  QApplication,QWidget,QPushButton
if __name__ == '__main__':
    #2,创建qt主程序类
    app = QApplication(sys.argv)
    #3,创建widge窗体
    w = QWidget()
    #4,设置窗名称
    w.setWindowTitle("PushButton")
    #5,创建btn对象
    btn = QPushButton("button 按钮")
    #6,设置btn的父级对象
    btn.setParent(w)
    #7,显示该窗体
    w.show()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
 
