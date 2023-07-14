#encoding:utf-8
from subprocess import PIPE, Popen
import os
# 返回的是 Popen 实例对象
proc = Popen(
    'ifconfig',  # cmd特定的查询空间的命令
    stdin=None,  # 标准输入 键盘
    stdout=PIPE,  # -1 标准输出（演示器、终端) 保存到管道中以便进行操作
    stderr=PIPE,  # 标准错误，保存到管道
    shell=True)

# print(proc.communicate()) # 标准输出的字符串+标准错误的字符串
outinfo, errinfo = proc.communicate()
print(outinfo.decode('utf-8'))  # 外部程序(windows系统)决定编码格式
print(errinfo.decode('utf-8'))


# ret = os.system("mkdir test2")
# print(ret)