# -*- coding: utf-8 -*-
"""
Created on Tues Aug  3 17:06:02 2021

@author: wmy and wjx
"""
import three_dof_ik
import serial
import serial.tools.list_ports
import threading
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
import time

ifwork = True


def write_coordinates(filename, offset):
    try:
        f = open(filename, "w")
        f.write(offset)
        f.close()
    except:
        time.sleep(0.1)


def get_coordinates(filename):
    try:
        f = open(filename)
        data = f.read()
        num = data.split(',')
        f.close()
        return [int(num[0]), int(num[1]), int(num[2])]
    except:
        time.sleep(0.1)


class SerialPortAssistant(object):

    def __init__(self):
        self.serial = serial.Serial()
        self.device = None
        self.baudrate = 115200
        self.encoding = "gb2312"
        self.recthread = None
        self.connecting = False
        self.comports = []
        self.devices = []
        self.search()
        self.interface()
        self.updatethread = threading.Thread(target=self.update)
        self.updatethread.start()
        self.angle0 = 1500
        pass

    def interface(self):
        self.root = tk.Tk()
        self.root.title('机械臂串口助手 V0.0.1')
        self.root.geometry('960x640')
        self.face = tk.Frame(self.root)
        self.face.config(height=640, width=960)
        self.face.propagate(False)
        self.face.pack(anchor='nw')
        # operate frame
        operateframe = tk.Frame(self.face)
        operateframe.config(height=220, width=960)
        operateframe.propagate(False)
        operateframe.pack(anchor='nw', side='bottom')
        operatespaceframe = tk.Frame(operateframe)
        operatespaceframe.config(height=220, width=10)
        operatespaceframe.propagate(False)
        operatespaceframe.pack(anchor='nw', side='left')
        # send text
        operatetextframe = tk.Frame(operateframe)
        operatetextframe.config(height=220, width=725)
        operatetextframe.propagate(False)
        operatetextframe.pack(anchor='nw', side='left')
        operatespaceframe = tk.Frame(operatetextframe)
        operatespaceframe.config(height=10, width=725)
        operatespaceframe.propagate(False)
        operatespaceframe.pack(anchor='nw', side='top')
        operatespaceframe = tk.Frame(operatetextframe)
        operatespaceframe.config(height=10, width=725)
        operatespaceframe.propagate(False)
        operatespaceframe.pack(anchor='sw', side='bottom')
        # operate right
        operateframeright = tk.Frame(operateframe)
        operateframeright.config(height=240, width=210)
        operateframeright.propagate(False)
        operateframeright.pack(anchor='nw', side='left')

        # send botton
        spacelabel = tk.Label(operateframeright, width=5, height=1)
        spacelabel.pack()
        self.sendbutton = tk.Button(operateframeright, text='发送坐标', \
                                    width=20, height=1, command=self.sendbuttoncmd)
        self.sendbutton.pack(side='top')
        # text
        self.sendtext = tk.Text(operatetextframe, height=15, width=99, bg='white', fg="black")
        self.sendscrollbar = tk.Scrollbar(operatetextframe)
        self.sendtext['yscrollcommand'] = self.sendscrollbar.set
        self.sendscrollbar['command'] = self.sendtext.yview
        self.sendtext.pack(side=tk.LEFT)
        self.sendscrollbar.pack(side='left', fill=tk.Y)
        # space frame
        spaceframe = tk.Frame(self.face)
        spaceframe.config(height=420, width=10)
        spaceframe.propagate(False)
        spaceframe.pack(anchor='nw', side='left')
        # text frame
        textframe = tk.Frame(self.face)
        textframe.config(height=420, width=725)
        textframe.propagate(False)
        textframe.pack(anchor='nw', side='left')
        # option frame
        optionframe = tk.Frame(self.face)
        optionframe.config(height=420., width=225)
        optionframe.propagate(False)
        optionframe.pack(anchor='ne', side='right')
        # text
        self.rectext = tk.Text(textframe, height=35, width=99, bg='black', fg="#00FF00")
        self.recscrollbar = tk.Scrollbar(textframe)
        self.rectext['yscrollcommand'] = self.recscrollbar.set
        self.rectext.config(state=tk.DISABLED)
        self.recscrollbar['command'] = self.rectext.yview
        self.rectext.pack(side=tk.LEFT, fill=tk.BOTH)
        self.recscrollbar.pack(side='left', fill=tk.Y)
        # option
        optionframebottom = tk.Frame(optionframe)
        optionframebottom.config(height=150., width=210)
        optionframebottom.propagate(False)
        optionframebottom.pack(anchor='sw', side='bottom')
        # left
        optionframeleft = tk.Frame(optionframe)
        optionframeleft.config(height=420., width=60)
        optionframeleft.propagate(False)
        optionframeleft.pack(anchor='nw', side='left')
        # right
        optionframeright = tk.Frame(optionframe)
        optionframeright.config(height=420., width=150)
        optionframeright.propagate(False)
        optionframeright.pack(anchor='nw', side='left')
        # serial
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label1 = tk.Label(optionframeleft, text="端口号", width=5, height=1)
        label1.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.serialselect = ttk.Combobox(optionframeright, width=15, height=5)
        self.serialselect.bind("<<ComboboxSelected>>", self.serialselectcmd)
        self.serialselect.pack()
        # baudrate
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label2 = tk.Label(optionframeleft, text="波特率", width=5, height=1)
        label2.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.baudrateselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.baudrateselect.bind("<<ComboboxSelected>>", self.baudrateselectcmd)
        self.baudrateselect['value'] = [1382400, 921600, 460800, 256000, 230400, \
                                        128000, 115200, 76800, 57600, 43000, 38400, 19200, 14400, \
                                        9600, 4800, 2400, 1200]
        self.baudrateselect.current(6)
        self.baudrateselect.pack()
        # cal bit
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label3 = tk.Label(optionframeleft, text="校验位", width=5, height=1)
        label3.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.calbitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.calbitselect['value'] = ["无校验", "奇校验", "偶校验"]
        self.calbitselect.current(0)
        self.calbitselect.pack()
        # data bit
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label4 = tk.Label(optionframeleft, text="数据位", width=5, height=1)
        label4.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.databitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.databitselect['value'] = [8, 7, 6, 5]
        self.databitselect.current(0)
        self.databitselect.pack()
        # stop bit
        spacelabel = tk.Label(optionframeleft, width=5, height=1)
        spacelabel.pack()
        label5 = tk.Label(optionframeleft, text="停止位", width=5, height=1)
        label5.pack()
        spacelabel = tk.Label(optionframeright, width=5, height=1)
        spacelabel.pack()
        self.stopbitselect = ttk.Combobox(optionframeright, width=15, height=8)
        self.stopbitselect['value'] = [1]
        self.stopbitselect.current(0)
        self.stopbitselect.pack()
        # check
        # self.hexdisplay = tk.BooleanVar()
        # self.hexdisplaycheck = tk.Checkbutton(optionframebottom, text='十六进制显示', \
        #                                       onvalue=True, offvalue=False, variable=self.hexdisplay)
        # self.hexdisplaycheck.pack()
        # open
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.openbutton = tk.Button(optionframebottom, text='打开串口', \
                                    width=20, height=1, command=self.openbuttoncmd)
        self.openbutton.pack()

        # remote
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.remotebutton = tk.Button(optionframebottom, text='开始遥控', \
                                      width=20, height=1, command=self.remotebuttoncmd)
        self.remotebutton.pack()

        # clear
        spacelabel = tk.Label(optionframebottom, width=5, height=1)
        spacelabel.pack()
        self.runbutton = tk.Button(optionframebottom, text='开始抓取', \
                                   width=20, height=1, command=self.runbuttoncmd)
        self.runbutton.pack()
        pass

    def baudrateselectcmd(self, *args):
        self.baudrate = int(self.baudrateselect.get())
        self.serial.baudrate = self.baudrate
        print(self.baudrate)
        pass

    def serialselectcmd(self, *args):
        self.device = self.serialselect.get().split()[0]
        self.serial.port = self.device
        print(self.device)
        pass

    def search(self):
        self.devices = []
        self.comports = list(serial.tools.list_ports.comports())
        for comport in self.comports:
            self.devices.append(comport.device)
            pass
        pass

    def update(self):
        while True:
            if self.connecting == False:
                self.search()
                self.serialselect['value'] = self.comports
                if len(list(self.serialselect['value'])) == 0:
                    self.serialselect['value'] = [""]
                    self.serialselect.current(0)
                    self.device = None
                    pass
                elif self.device == None or self.device not in self.devices:
                    self.serialselect.current(0)
                    self.device = self.devices[0]
                    pass
                self.serialselect.update()
                self.face.update_idletasks()
                pass
            pass
        pass

    def serialopen(self):
        self.serial.port = self.device
        self.serial.baudrate = self.baudrate
        self.serial.timeout = 2
        try:
            self.serialclose()
            time.sleep(0.1)
            self.serial.open()
        except Exception as error:
            tk.messagebox.showinfo(title='无法连接到串口', message=error)
            return False
        else:
            if self.serial.isOpen():
                self.connecting = True
                # self.recthread = threading.Thread(target=self.receive)
                # self.recthread.start()
                return True
            else:
                return False
            pass
        pass

    def serialclose(self):
        self.connecting = False
        time.sleep(0.1)
        self.serial.close()
        pass

    def receive(self):
        while self.connecting:
            try:
                nchar = self.serial.inWaiting()
                pass
            except:
                self.connecting = False
                self.serialclose()
                self.openbutton['text'] = '打开串口'
                pass
            if nchar:
                if self.hexdisplay.get() == False:
                    data = ''.encode('utf-8')
                    data = data + self.serial.read(nchar)
                    try:
                        self.rectext.config(state=tk.NORMAL)
                        self.rectext.insert(tk.END, data.decode(self.encoding))
                        self.rectext.config(state=tk.DISABLED)
                        self.rectext.yview_moveto(1)
                        self.rectext.update()
                        pass
                    except:
                        pass
                    pass
                else:
                    data = self.serial.read(nchar)
                    convert = '0123456789ABCDEF'
                    string = ''
                    for char in data:
                        string += convert[char // 16] + convert[char % 16] + ' '
                        pass
                    self.rectext.config(state=tk.NORMAL)
                    self.rectext.insert(tk.END, string)
                    self.rectext.config(state=tk.DISABLED)
                    self.rectext.yview_moveto(1)
                    self.rectext.update()
                    pass
                pass
            pass
        pass

    def run(self):
        self.root.mainloop()
        self.exit()
        pass

    def exit(self):
        self.serialclose()
        pass

    # 按钮
    def openbuttoncmd(self):
        if self.openbutton['text'] == '打开串口':
            is_open = self.serialopen()
            if is_open:
                self.openbutton['text'] = '关闭串口'
                time.sleep(0.5)
                self.restoration()
                pass
            pass
        else:
            self.restoration()
            self.serialclose()
            self.openbutton['text'] = '打开串口'
            pass
        pass

    def remotebuttoncmd(self):
        if self.remotebutton['text'] == '开始遥控':
            self.root.bind("<Key>", self.func1)
            self.remotebutton['text'] = '结束遥控'
        else:
            self.root.unbind("<Key>")
            self.remotebutton['text'] = '开始遥控'

    def runbuttoncmd(self):
        global ifwork
        if self.runbutton['text'] == '开始抓取':
            t1 = threading.Thread(target=self.working)
            t1.start()
            ifwork = True
            self.runbutton['text'] = '结束抓取'
        else:
            ifwork = False
            self.runbutton['text'] = '开始抓取'

    # 读取界面参数控制机械臂
    def sendbuttoncmd(self):
        if self.connecting:
            data = self.sendtext.get(1.0, tk.END)
            num = data.split(',')
            num = list(map(int, num))
            self.robotrun(num)
        else:
            tk.messagebox.showinfo(title='无法发送', message='请先打开串口')
            pass
        pass

    # 键盘事件
    def func1(self, event):
        print("事件触发键盘输入:{0},对应的ASCII码:{1}".format(event.keysym, event.keycode))
        if event.keysym == "Up":
            self.angle0 = self.angle0 + 50
            if self.angle0 > 2500:
                self.angle0 = 2500
            data = "#000P" + str(self.angle0) + "T0100!\n"
            self.serial.write(data[0:-1].encode(self.encoding))
        elif event.keysym == "Down":
            self.angle0 = self.angle0 - 50
            if self.angle0 < 500:
                self.angle0 = 500
            data = "#000P" + str(self.angle0) + "T0100!\n"
            self.serial.write(data[0:-1].encode(self.encoding))

    ############################ 机械臂控制函数 ##################################
    # 复位
    def restoration(self):
        data = "$RST!\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.1)
        data = "$DST!\n"
        self.serial.write(data[0:-1].encode(self.encoding))

    # 气泵吸
    def suckup(self):
        data = "{#004P1000T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.2)

        data = "{#003P2000T1000!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.8)

        data = "{#003P1000T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.1)

    # 气泵放
    def suckdown(self):
        data = "{#004P2200T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.3)
        data = "{#004P1500T0200!}\n"
        self.serial.write(data[0:-1].encode(self.encoding))
        time.sleep(0.3)

    # 机械臂运动
    def robotrun(self, offset, t=1000):
        hasik, j1, j2, j3 = three_dof_ik.inverse_kinematics(offset[0], offset[1], offset[2])
        print(j1, j2, j3)
        if hasik:
            self.sendmsg(agl0=int(j1 * 7.41) + 1500, agl1=1500 - int(j2 * 7.41), agl2=1550 + int(j3 * 7.41),
                         run_time=t)
            return True
        else:
            return False

    def working(self):
        last_offset = []
        self.robotrun([0, -190, 120])  # 随便设定的初始位置

        while (ifwork):
            offset = get_coordinates("offset.txt") # 读取物体位置
            time.sleep(0.2)
            print("txt中坐标为：", offset)
            if offset[0] != -1:
                # 从目标位置高30mm的位置开始下降，这样的控制会更加稳定
                if self. robotrun([offset[0], offset[1], offset[2]+30]):  # 物体位置上方30mm
                    if self.robotrun(offset):          # 物体位置
                        self.suckup()                  # 吸住
                        self.robotrun([0, -210, 50])   # 随便设定的目标位置正上方30mm
                        self.robotrun([0, -210, 20])   # 随便设定的目标位置
                        self.suckdown()                # 放下
                        self.robotrun([0, -190, 120])  # 回到初始位置

                        write_coordinates("offset.txt", "-1,-1,-1")

    # 串口发送指令到机械臂
    def sendmsg(self, agl0=1500, agl1=1500, agl2=1550, agl3=1500, run_time=1000):
        data = "{#000P" + str(agl0) + "T" + str(run_time) + "!" + \
               "#001P" + str(agl1) + "T" + str(run_time) + "!" + \
               "#002P" + str(agl2) + "T" + str(run_time) + "!" + \
               "#003P" + str(agl3) + "T" + str(run_time) + "!" + "}\n"
        self.serial.write(data.encode(self.encoding))
        time.sleep(run_time / 1000.0)


if __name__ == '__main__':
    assistant = SerialPortAssistant()
    assistant.run()
