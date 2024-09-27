import tkinter as tk  # 使用Tkinter前需要先导入
# 第1步，实例化object，建立窗口window
def window(input):
    window = tk.Tk()
    # 第2步，给窗口的可视化起名字
    window.title('result')

    # 第3步，设定窗口的大小(长 * 宽)
    window.geometry('500x300')  # 这里的乘是小x

    # 第4步，在图形界面上设定标签
    l = tk.Label(window, text='测 试 结 果 ：', bg='green', font=('Arial', 30), width=30, height=2)
    l2 = tk.Label(window, text=input, bg='white', font=('Arial', 70), width=30, height=2)
    # 说明： bg为背景，font为字体，width为长，height为高，这里的长和高是字符的长和高，比如height=2,就是标签有2个字符这么高
    # 第5步，放置标签
    l.pack()  # Label内容content区域放置位置，自动调节尺寸
    # 放置lable的方法有：1）l.pack(); 2)l.place();
    l2.pack()
    # 第6步，主窗口循环显示
    window.mainloop()