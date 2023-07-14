#encoding:utf-8
import sys
from PyQt5.QtWidgets import  *
if __name__ == '__main__':
    #1,创建qt主程序类
    app = QApplication(sys.argv)
    #2,创建widge窗体
    w = QWidget()
    #3,设置窗名称
    w.setWindowTitle("login")
    label_name = QLabel("name",w)
    label_name.setGeometry(0,0,100,40)
    label_paswd = QLabel("passwd",w)
    label_paswd.setGeometry(0,50,100,40)
    linedit_name = QLineEdit(w)
    linedit_name.setGeometry(120,0,300,40)
    linedit_name.setPlaceholderText("请输入账号")
    linedit_passwd = QLineEdit(w)
    linedit_passwd.setGeometry(120,50,300,40)
    linedit_passwd.setPlaceholderText("请输入密码")
    linedit_passwd.setEchoMode(QLineEdit.PasswordEchoOnEdit)
    btn_login = QPushButton("登陆",w)
    btn_login.setGeometry(100,100,100,50)
    check_box_show_paswd = QCheckBox("显示密码",w)
    check_box_show_paswd.setGeometry(450,50,200,40)
    w.resize(600,600)
    w.show()
    #阻塞主函数 等待APP退出
    sys.exit(app.exec_())
