# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './SpeedControl.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_front = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_front.setObjectName("pushButton_front")
        self.gridLayout.addWidget(self.pushButton_front, 0, 1, 1, 1)
        self.pushButton_left = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_left.setObjectName("pushButton_left")
        self.gridLayout.addWidget(self.pushButton_left, 1, 0, 1, 1)
        self.pushButton_stop = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_stop.setObjectName("pushButton_stop")
        self.gridLayout.addWidget(self.pushButton_stop, 1, 1, 1, 1)
        self.pushButton_right = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_right.setObjectName("pushButton_right")
        self.gridLayout.addWidget(self.pushButton_right, 1, 2, 1, 1)
        self.pushButton_back = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_back.setObjectName("pushButton_back")
        self.gridLayout.addWidget(self.pushButton_back, 2, 1, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 49))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_front.setText(_translate("MainWindow", "前"))
        self.pushButton_front.setShortcut(_translate("MainWindow", "W"))
        self.pushButton_left.setText(_translate("MainWindow", "左"))
        self.pushButton_left.setShortcut(_translate("MainWindow", "A"))
        self.pushButton_stop.setText(_translate("MainWindow", "停止"))
        self.pushButton_stop.setShortcut(_translate("MainWindow", "S"))
        self.pushButton_right.setText(_translate("MainWindow", "右"))
        self.pushButton_right.setShortcut(_translate("MainWindow", "D"))
        self.pushButton_back.setText(_translate("MainWindow", "后"))
        self.pushButton_back.setShortcut(_translate("MainWindow", "S"))

