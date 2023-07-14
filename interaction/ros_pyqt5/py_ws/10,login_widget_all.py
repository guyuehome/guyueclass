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
        self.linedit_name = QLineEdit()
        self.linedit_name.setPlaceholderText("请输入账号")
        self.linedit_passwd = QLineEdit()
        self.linedit_passwd.setPlaceholderText("请输入密码")
        self.linedit_passwd.setEchoMode(QLineEdit.Password)
        self.btn_login = QPushButton("登陆",)
        self.check_box_show_paswd = QCheckBox("显示密码",)
        self.check_box_show_paswd.stateChanged.connect(self.SetPasswdState)
        main_layout.addWidget(label_name,0,0)
        main_layout.addWidget(label_paswd,1,0)
        main_layout.addWidget(self.linedit_name,0,1)
        main_layout.addWidget(self.linedit_passwd,1,1)
        main_layout.addWidget(self.check_box_show_paswd,1,2)
        main_layout.addWidget(self.btn_login,2,1)
        horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum) #水平
        verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)#垂直
        main_layout.addItem(verticalSpacer,3,1)
        self.setLayout(main_layout)
        self.resize(600,600)

    def SetPasswdState(self):
        print("checkstate changed:"+str(self.check_box_show_paswd.isChecked()))
        if self.check_box_show_paswd.isChecked():
            self.linedit_passwd.setEchoMode(QLineEdit.Normal)
        else:
            self.linedit_passwd.setEchoMode(QLineEdit.Password)


def CheckPasswd(w,new_widget):
    print("clicked")
    if w.linedit_name.text() == "test" and w.linedit_passwd.text()=="123456":
        print("ok")
        new_widget.setWindowTitle("hello:"+w.linedit_name.text())
        new_widget.show()
        w.close()
    else:
        print("error")
        QMessageBox.warning(w,"错误", "账号或密码错误", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes) 

if __name__ == '__main__':
    #1,创建qt主程序类
    app = QApplication(sys.argv)
    w =LoginWidget()
    w.show()
    new_widget =QWidget()
    w.btn_login.clicked.connect(lambda:CheckPasswd(w,new_widget))
    sys.exit(app.exec_())
