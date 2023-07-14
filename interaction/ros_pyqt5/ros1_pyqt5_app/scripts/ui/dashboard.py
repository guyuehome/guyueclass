# coding:utf-8
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from math import *
import sys
import sys
import math
from PyQt5.QtGui import QFont, QColor, QPainter, QRadialGradient, QPolygon, QPen
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtWidgets import (QApplication, QWidget, QLCDNumber, QFrame)
class Dashboard(QWidget):
    def __init__(self, parent=None):
        super(Dashboard, self).__init__(parent)
        self.setWindowTitle("QPainter测试")
        self.setMinimumSize(600, 200)
        # self.setMaximumSize(700, 700)
        self._title = 'Title____'

        # 颜色设置
        self.pieColorStart = QColor(63, 191, 127)  # 绿色
        self.pieColorMid = QColor(63, 127, 191)  # 蓝色
        self.pieColorEnd = QColor(203, 72, 72)  # 红色
        self.pointerColor = QColor(72, 203, 203)  # 青色
        
        # 设置字符
        self.font = QFont("宋体", 8)
        self.font.setBold(True)
        # LCD初始化
        self.lcd_init()
        self.currentValue = 30
        self.startAngle = 45 #以QPainter坐标方向为准,建议画个草图看看
        self.endAngle = 45 #以以QPainter坐标方向为准
        self.scaleMainNum = 10 #主刻度数
        self.scaleSubNum = 10 #主刻度被分割份数
        self.minValue = 0
        self.maxValue = 100
        self._title = '×100m/s'
        self.value = 0
        self.minRadio = 100 #缩小比例,用于计算刻度数字
        self.decimals = 0 #小数位数
        self.scaleMajor = 8
    def setTitle(self, title):
        self._title = title

    def setValue(self, value):
        self.currentValue = value
        self.update()
    def paintEvent(self, event):
        # 坐标轴变换 默认640*480
        width = self.width()
        height = self.height()

        painter = QPainter(self)  # 初始化painter
        painter.translate(width / 2, height / 2)  # 坐标轴变换，调用translate()将坐标原点平移至窗口中心

        # 坐标刻度自适应
        side = min(width, height)
        painter.scale(side / 200.0, side / 200.0) 
        # 本项目中将坐标缩小为side/200倍，即画出length=10的直线，其实际长度应为10*(side/200)。

        # 启用反锯齿，使画出的曲线更平滑
        painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        painter.begin(self)
        
        # 开始画图
        self.drawColorPie(painter)
        self.drawPointerIndicator(painter)
        self.drawLine(painter)
        self.drawText(painter)
        # self.drawTitle(painter)
        
        # 放置lcd并显示数值
        # 根据自己屏幕的大小调整lcd的位置
        self.lcd.setGeometry(self.width() / 2.15, self.height() / 9 * 7, self.width() / 10,
                                self.height() / 10)  # 用setGeometry布局 2.25 , 9*7, 10,10
        formValue = "%d" % self.currentValue  # 数值的格式化显示
        self.lcd.display(formValue)  # 将指针所指的值显示在LCD屏幕上
    def drawColorPie(self, painter):  # 绘制三色环
        painter.save()  # save()保存当前坐标系
        # print("drawColorPie")
        # 设置扇形部分区域
        radius = 99  # 半径
        painter.setPen(Qt.NoPen)
        rect = QRectF(-radius, -radius, radius * 2, radius * 2)  # 扇形所在圆区域

        # 计算三色圆环范围角度。green：blue：red = 1：2：1
        angleAll = 360.0 - self.startAngle - self.endAngle  # self.startAngle = 45, self.endAngle = 45
        angleStart = angleAll * 0.25
        angleMid = angleAll * 0.5
        angleEnd = angleAll * 0.25

        # 圆的中心部分填充为透明色，形成环的样式
        rg = QRadialGradient(0, 0, radius, 0, 0)  # 起始圆心坐标，半径，焦点坐标
        ratio = 0.8  # 透明：实色 = 0.8 ：1

        # 绘制绿色环
        rg.setColorAt(0, Qt.transparent)  # 透明色
        rg.setColorAt(ratio, Qt.transparent)
        rg.setColorAt(ratio + 0.01, self.pieColorStart)
        rg.setColorAt(1, self.pieColorStart)

        painter.setBrush(rg)
        painter.drawPie(rect, (270 - self.startAngle - angleStart) * 16, angleStart * 16)

        # 绘制蓝色环
        rg.setColorAt(0, Qt.transparent)
        rg.setColorAt(ratio, Qt.transparent)
        rg.setColorAt(ratio + 0.01, self.pieColorMid)
        rg.setColorAt(1, self.pieColorMid)

        painter.setBrush(rg)
        painter.drawPie(rect, (270 - self.startAngle - angleStart - angleMid) * 16, angleMid * 16)

        # 绘制红色环
        rg.setColorAt(0, Qt.transparent)
        rg.setColorAt(ratio, Qt.transparent)
        rg.setColorAt(ratio + 0.01, self.pieColorEnd)
        rg.setColorAt(1, self.pieColorEnd)

        painter.setBrush(rg)
        painter.drawPie(rect, (270 - self.startAngle - angleStart - angleMid - angleEnd) * 16, angleEnd * 16)

        painter.restore()  # restore()恢复坐标系
    def drawPointerIndicator(self, painter):
        painter.save()
        # 绘制指针
        # print("drawPointerIndicator")
        radius = 58  # 指针长度
        painter.setPen(Qt.NoPen)
        painter.setBrush(self.pointerColor)

        # (-5, 0), (0, -8), (5, 0)和（0, radius) 四个点绘出指针形状
        # 绘制多边形做指针
        pts = QPolygon()
        pts.setPoints(-5, 0, 0, -8, 5, 0, 0, radius)
        # print("radius:" + str(radius))

        # 旋转指针，使得指针起始指向为0刻度处
        painter.rotate(self.startAngle)
        degRotate = (360.0 - self.startAngle - self.endAngle) / (self.maxValue - self.minValue) \
                    * (self.currentValue - self.minValue)
        painter.rotate(degRotate)
        painter.drawConvexPolygon(pts)
        painter.restore()
    def drawText(self, painter):
        painter.save()
        # 绘制刻度值
        # print("drawText")
        # 位置调整
        startRad = 4
        deltaRad = 0.6
        radius = 63
        offset = 5.5
        for i in range(self.scaleMajor + 1):  # self.scaleMajor = 8, 8个主刻度
            # 正余弦计算
            sina = math.sin(startRad - i * deltaRad)
            cosa = math.cos(startRad - i * deltaRad)

            # 刻度值计算
            value = math.ceil((1.0 * i * (
                        (self.maxValue - self.minValue) / self.scaleMajor) + self.minValue))  
                        # math.ceil(x)：返回不小于x的最小整数
            strValue = str(int(value))

            # 字符的宽度和高度
            textWidth = self.fontMetrics().width(strValue)
            textHeight = self.fontMetrics().height()

            # 字符串的起始位置。注意考虑到字符宽度和高度进行微调
            x = radius * cosa - textWidth / 2
            y = -radius * sina + textHeight / 4

            painter.setFont(self.font)
            painter.setPen(QColor(26, 95, 95)) # 还是用自己选的颜色
            painter.drawText(x - offset, y, strValue )
            # Y为自己加的单位，可以不加，直接在 Title 中进行总体设置也行
        painter.restore()
    
    def drawLine(self, painter):
        painter.save()
        # 绘制刻度线
        # print("drawLine")
        radius = 79
        painter.rotate(self.startAngle)  # self.startAngle = 45,旋转45度
        steps = self.scaleMajor  # 8个刻度
        angleStep = (360.0 - self.startAngle - self.endAngle) / steps  # 刻度角
        for i in range(steps + 1):
            if i < 3:
                color = self.pieColorStart
            elif i < 7:
                color = self.pieColorMid
            else:
                color = self.pieColorEnd
            painter.setPen(QPen(color, Qt.SolidLine))
            painter.drawLine(0, radius - 5, 0, radius)
            painter.rotate(angleStep)
        painter.restore()
    def lcd_init(self):
            # print("display")
            self.lcd = QLCDNumber(self)
            self.lcd.setSmallDecimalPoint(False)  # False：小数点单独占一位；True：小数点不占位
            self.lcd.setDigitCount(2)  # 最多显示2位数字
            self.lcd.setFrameStyle(QFrame.NoFrame)  # 无边框
            self.lcd.setMode(QLCDNumber.Dec)  # 十进制显示
            self.lcd.setStyleSheet("color: rgb(26, 95, 95)") # 显示数字的颜色
            self.lcd.setSegmentStyle(QLCDNumber.Flat)  # 数字显示样式为flat，不向外凸
