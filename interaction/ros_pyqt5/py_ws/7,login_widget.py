#encoding:utf-8
import sys
from PyQt5.QtWidgets import  *

class LoginWidget(QWidget):
    def __init__(self):
        super(LoginWidget,self).__init__()
        self.initUi()
    def initUi(self):
        #3,设置窗名称
        self.setWindowTitle("login")
        main_layout =QGridLayout()
        label_name = QLabel("name")
        label_paswd = QLabel("passwd")
        linedit_name = QLineEdit()
        linedit_name.setPlaceholderText("请输入账号")
        linedit_passwd = QLineEdit()
        linedit_passwd.setPlaceholderText("请输入密码")
        linedit_passwd.setEchoMode(QLineEdit.PasswordEchoOnEdit)
        btn_login = QPushButton("登陆",)
        check_box_show_paswd = QCheckBox("显示密码",)

        main_layout.addWidget(label_name,0,0)
        main_layout.addWidget(label_paswd,1,0)
        main_layout.addWidget(linedit_name,0,1)
        main_layout.addWidget(linedit_passwd,1,1)
        main_layout.addWidget(check_box_show_paswd,1,2)
        main_layout.addWidget(btn_login,2,1)
        horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum) #水平
        verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)#垂直
        main_layout.addItem(verticalSpacer,3,1)
        self.setLayout(main_layout)
        self.resize(600,600)



if __name__ == '__main__':
    #1,创建qt主程序类
    app = QApplication(sys.argv)
    w =LoginWidget()
    w.show()
    sys.exit(app.exec_())
