#!/usr/bin/env python
# _*_ coding:UTF-8 _*_
import rospy
import signal
from std_msgs.msg import String
from webots_ros.srv import set_float
from webots_ros.srv import set_int
from geometry_msgs.msg import Twist

TIME_STEP = 32
MAX_SPEED = 5.0
controllerName = ""
v_x = 0
w_z = 0
motorNames = ["wheel1", "wheel2", "wheel3", "wheel4"]

def autoQuit(signum,frame):
    for i_ in range(len(motorNames)):
        serviceName = controllerName+"/"+motorNames[i_] +"/set_velocity"
        setVelClient = rospy.ServiceProxy(serviceName,set_float)
        setVelClient.call(0)

    serviceName = controllerName + "/robot/time_step"
    timeStepClient = rospy.ServiceProxy(serviceName,set_int)
    timeStepClient.call(0)

    rospy.signal_shutdown("closed!")
    rospy.loginfo("用户终止节点运行！" )
    return 

def controllerNameCallback(data):
    """webots端控制器名称回调函数"""
    global controllerName
    controllerName = "/" + data.data

def cmdCallback(data):
    """更新小车速度"""
    global v_x
    global w_z
    v_x = data.linear.x
    w_z = data.angular.z


def webots_controller_function():
    # 初始化节点
    rospy.init_node('my_webots_controller', anonymous=True)
    #rospy.wait_for_message("/model_name",String)

    # 订阅控制器名称
    while controllerName == "":
        nameSub = rospy.Subscriber("/model_name", String, controllerNameCallback)

    signal.signal(signal.SIGINT, autoQuit)

    # 设置基本仿真步长
    serviceName = controllerName + "/robot/time_step"
    timeStepClient = rospy.ServiceProxy(serviceName,set_int)
    resp = timeStepClient.call(TIME_STEP)
    if resp.success:
        rospy.loginfo("仿真步长设置成功:%s" %TIME_STEP )
    else:
        rospy.logwarn("服务调用失败!!!" ) 

    for i_ in range(len(motorNames)):
        # 定义服务
        serviceName =  controllerName + "/"+motorNames[i_] + "/set_position"
        setPosClient = rospy.ServiceProxy(serviceName,set_float)
        # 向server端发送请求，发送的request内容为set_float.srv中定义的
        resp = setPosClient.call(float("inf"))
        if resp.success:
            rospy.loginfo("位置模式设置成功:%s" %motorNames[i_] )
        else:
            rospy.logwarn("调用set_position服务失败:%s"%motorNames[i_] )
                
        serviceName = controllerName+"/"+motorNames[i_] +"/set_velocity"
        setVelClient = rospy.ServiceProxy(serviceName,set_float)
        resp = setVelClient.call(0)
        if resp.success:
            rospy.loginfo("速度模式设置成功:%s" %motorNames[i_] )
        else:
            rospy.logwarn("调用set_velocity服务失败:%s"%motorNames[i_] )

    while not rospy.is_shutdown():
        cmdSub = rospy.Subscriber("/cmd_vel",Twist,cmdCallback)
        print(str(v_x)+"   "+str(w_z))

        if v_x > 0.0 and w_z == 0.0: # 前
            for i_ in range(len(motorNames)):
                serviceName = controllerName+"/"+motorNames[i_] +"/set_velocity"
                setVelClient = rospy.ServiceProxy(serviceName,set_float)
                setVelClient.call(MAX_SPEED)

        if v_x < 0.0 and w_z == 0.0: # 后
            for i_ in range(len(motorNames)):
                serviceName = controllerName+"/"+motorNames[i_] +"/set_velocity"
                setVelClient = rospy.ServiceProxy(serviceName,set_float)
                setVelClient.call(- MAX_SPEED)

        if v_x == 0.0 and w_z > 0.0: # 左
            serviceName = controllerName+"/"+motorNames[0] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(- MAX_SPEED)

            serviceName = controllerName+"/"+motorNames[1] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(MAX_SPEED)

            serviceName = controllerName+"/"+motorNames[2] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(MAX_SPEED)

            serviceName = controllerName+"/"+motorNames[3] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(- MAX_SPEED)

        if v_x == 0.0 and w_z < 0.0: # 右
            serviceName = controllerName+"/"+motorNames[0] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(MAX_SPEED)

            serviceName = controllerName+"/"+motorNames[1] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(-MAX_SPEED)

            serviceName = controllerName+"/"+motorNames[2] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(-MAX_SPEED)
            
            serviceName = controllerName+"/"+motorNames[3] +"/set_velocity"
            setVelClient = rospy.ServiceProxy(serviceName,set_float)
            setVelClient.call(MAX_SPEED)
        if v_x == 0.0 and w_z == 0.0: # 停
            for i_ in range(len(motorNames)):
                serviceName = controllerName+"/"+motorNames[i_] +"/set_velocity"
                setVelClient = rospy.ServiceProxy(serviceName,set_float)
                setVelClient.call(0)
        
        # 判断webots端是否重置仿真，如果重置则退出运行
        resp = timeStepClient.call(TIME_STEP)
        if (not resp.success):
            rospy.logerr("Failed to call service time_step for next step.")
            rospy.signal_shutdown("closed!")
        
if __name__ == '__main__':
    webots_controller_function()