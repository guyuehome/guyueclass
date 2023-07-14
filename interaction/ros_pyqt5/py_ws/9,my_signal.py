#encoding:utf-8
import sys
#1,导入QPushButton
from PyQt5.QtWidgets import  QApplication,QWidget,QPushButton
from PyQt5.QtCore import QObject, pyqtSignal
class MyClass(QObject):
    my_signals= pyqtSignal(object)
    def __init__(self):
        super(MyClass,self).__init__()
    def SendSignal(self):
        self.my_signals.emit("hello signal test 222")
def slot_test(msg):
    print("I recv signal "+msg)
if __name__ == '__main__':
   
    app = QApplication(sys.argv)
    my_class= MyClass()
    my_class.my_signals.connect(slot_test)
    my_class.SendSignal()
    # sys.exit(app.exec_())
  
