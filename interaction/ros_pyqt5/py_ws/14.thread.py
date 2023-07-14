#!/usr/bin/env python
# coding:utf-8
import threading
import time
def thread_job():
    while True:
        print("thread one ")
        #100ms执行一次
        time.sleep(0.1)
def thread_job2():
    while True:
        print("thread two")
        #200ms执行一次
        time.sleep(0.2)
if __name__ == '__main__':
    thread1 = threading.Thread(target = thread_job)
    thread1.start()
    thread2 = threading.Thread(target = thread_job2)
    thread2.start()
