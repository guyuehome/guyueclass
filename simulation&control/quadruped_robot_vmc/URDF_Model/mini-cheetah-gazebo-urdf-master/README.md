# mini-cheetah-gazebo-urdf
An urdf description file of a quadruped robot modeled on mini cheetah. With this you can import the robot into the gazebo environment to realize the control of the robot.

Ubuntu 16.04(ROS Kinetic)

gazebo 8.6

Step:

cd <your_ws>/src

git clone https://github.com/HitSZwang/mini-cheetah-gazebo-urdf

cd ..

catkin_make

source devel/setup.bash

1.If you want to browse the robot model

  roslaunch yobo_gazebo demo.launch
  
2.You can ues the teleop to control the robot

This model was used in champ and achieved good results.

the champ program https://github.com/chvmp/champ

if you use champ_setup_assistant:   https://github.com/chvmp/champ_setup_assistant

you should choose the yobotics.urdf


