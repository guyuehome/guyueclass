# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'QuickCmdWindow.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(837, 656)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(60, 180, 278, 132))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton_start_sim = QtWidgets.QPushButton(self.widget)
        self.pushButton_start_sim.setObjectName("pushButton_start_sim")
        self.verticalLayout.addWidget(self.pushButton_start_sim)
        self.pushButton_end_sim = QtWidgets.QPushButton(self.widget)
        self.pushButton_end_sim.setObjectName("pushButton_end_sim")
        self.verticalLayout.addWidget(self.pushButton_end_sim)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.pushButton_start_mapping = QtWidgets.QPushButton(self.widget)
        self.pushButton_start_mapping.setObjectName("pushButton_start_mapping")
        self.verticalLayout_2.addWidget(self.pushButton_start_mapping)
        self.pushButton_end_mapping = QtWidgets.QPushButton(self.widget)
        self.pushButton_end_mapping.setObjectName("pushButton_end_mapping")
        self.verticalLayout_2.addWidget(self.pushButton_end_mapping)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 837, 49))
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
        self.pushButton_start_sim.setText(_translate("MainWindow", "开启仿真"))
        self.pushButton_end_sim.setText(_translate("MainWindow", "关闭仿真"))
        self.pushButton_start_mapping.setText(_translate("MainWindow", "开始建图"))
        self.pushButton_end_mapping.setText(_translate("MainWindow", "结束建图"))

