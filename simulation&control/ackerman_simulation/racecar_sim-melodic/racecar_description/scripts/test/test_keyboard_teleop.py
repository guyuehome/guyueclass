#!/usr/bin/env python
import rospy

from ackermann_msgs.msg import AckermannDrive
# from ackermann_msgs.msg import AckermannDriveStamped
import sys, select, termios, tty

banner = """
Reading from the keyboard  and Publishing to AckermannDriveStamped!
---------------------------
Moving around:
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 0.7
turn = 0.9

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  # pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDriveStamped,queue_size=1)
  pub = rospy.Publisher("/ackermann_cmd_mux/output", AckermannDrive,queue_size=1)
  rospy.init_node('keyop')

  x = 0
  th = 0
  status = 0

  try:
    while(1):
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break
       msg = AckermannDrive();
      #  msg.header.stamp = rospy.Time.now();
      #  msg.header.frame_id = "base_link";

       msg.speed = x*speed;
       msg.acceleration = 1;
       msg.jerk = 1;
       msg.steering_angle = th*turn
       msg.steering_angle_velocity = 1

       pub.publish(msg)

  except:
    print 'error'

  finally:
    msg = AckermannDrive();
    # msg.header.stamp = rospy.Time.now();
    # msg.header.frame_id = "base_link";

    msg.speed = 0;
    msg.acceleration = 1;
    msg.jerk = 1;
    msg.steering_angle = 0
    msg.steering_angle_velocity = 1
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
