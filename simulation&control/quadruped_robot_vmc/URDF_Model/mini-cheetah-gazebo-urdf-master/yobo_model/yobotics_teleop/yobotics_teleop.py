#!/usr/bin/env python
#credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function

import roslib; roslib.load_manifest('yobotics_teleop')
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from yobotics_msgs.msg import Pose

import sys, select, termios, tty
import numpy as np

class Teleop:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.pose_publisher = rospy.Publisher('cmd_pose', Pose, queue_size = 1)
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)

        self.speed = rospy.get_param("~speed", 0.3)
        self.turn = rospy.get_param("~turn", 1.0)

        self.msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
u    i    o
j    k    l
m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
U    I    O
J    K    L
M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
        """

        self.velocityBindings = {
                'i':(1,0,0,0),
                'o':(1,0,0,-1),
                'j':(0,0,0,1),
                'l':(0,0,0,-1),
                'u':(1,0,0,1),
                ',':(-1,0,0,0),
                '.':(-1,0,0,1),
                'm':(-1,0,0,-1),
                'O':(1,-1,0,0),
                'I':(1,0,0,0),
                'J':(0,1,0,0),
                'L':(0,-1,0,0),
                'U':(1,1,0,0),
                '<':(-1,0,0,0),
                '>':(-1,-1,0,0),
                'M':(-1,1,0,0),
                'v':(0,0,1,0),
                'n':(0,0,-1,0),
            }

        self.poseBindings = {
                'f':(-1,0,0,0),
                'h':(1,0,0,0),
                't':(0,1,0,0),
                'b':(0,-1,0,0),
                'r':(0,0,1,0),
                'y':(0,0,-1,0),
            }

        self.speedBindings={
                'q':(1.1,1.1),
                'z':(.9,.9),
                'w':(1.1,1),
                'x':(.9,1),
                'e':(1,1.1),
                'c':(1,.9),
            }
        
        self.poll_keys()

    def joy_callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[7] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[6] * self.speed
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = (not data.buttons[4]) * data.axes[6] * self.turn
        self.velocity_publisher.publish(twist)

        body_pose = Pose()
        body_pose.x = 0
        body_pose.y = 0
        body_pose.roll = (not data.buttons[5]) *-data.axes[3] * 0.349066
        body_pose.pitch = data.axes[4] * 0.261799
        body_pose.yaw = data.buttons[5] * data.axes[3] * 0.436332
        if data.axes[5] < 0:
            body_pose.z = self.map(data.axes[5], 0, -1.0, 1, 0.00001)
        else:
            body_pose.z = 1.0
    
        self.pose_publisher.publish(body_pose)

    def poll_keys(self):
        self.settings = termios.tcgetattr(sys.stdin)

        x = 0
        y = 0
        z = 0
        th = 0
        roll = 0
        pitch = 0
        yaw = 0
        status = 0
        cmd_attempts = 0

        try:
            print(self.msg)
            print(self.vels( self.speed, self.turn))
            
            while not rospy.is_shutdown():
                key = self.getKey()
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]
                    
                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = x *self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0
                        twist.angular.y = 0
                        twist.angular.z = th * self.turn
                        self.velocity_publisher.publish(twist)

                    cmd_attempts += 1
                    
                elif key in self.poseBindings.keys():
                    #TODO: changes these values as rosparam
                    roll += 0.0174533 * self.poseBindings[key][0]
                    pitch += 0.0174533 * self.poseBindings[key][1]
                    yaw += 0.0174533 * self.poseBindings[key][2]

                    roll = np.clip(roll, -0.523599, 0.523599)
                    pitch = np.clip(pitch, -0.349066, 0.349066)
                    yaw = np.clip(yaw, -0.436332, 0.436332)

                    if cmd_attempts > 1:
                        body_pose = Pose()
                        body_pose.x = 0
                        body_pose.y = 0
                        body_pose.z = 0
                        body_pose.roll = roll
                        body_pose.pitch = pitch
                        body_pose.yaw = yaw
                        self.pose_publisher.publish(body_pose)

                    cmd_attempts += 1

                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    
                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(self.msg)
                    status = (status + 1) % 15

                else:
                    cmd_attempts = 0
                    if (key == '\x03'):
                        break

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.velocity_publisher.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

if __name__ == "__main__":
    rospy.init_node('yobotics_teleop')
    teleop = Teleop()