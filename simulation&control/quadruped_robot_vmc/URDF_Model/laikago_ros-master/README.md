# Instructions: Laikago working with ROS. 

The test environment is: 

Ubuntu 16.04 + ROS Kinetic. 

Ubuntu 18.04 + ROS Melodic.

Remember rename lib file when switch **x86 platform** and **ARM platform**.

## Dependencies:
[Gazebo8](http://gazebosim.org/)

## Configuration:
Make sure the following exist in your `~/.bashrc` file or export them in terminal. `kinetic`, `gazebo-8` and `~/catkin_ws` should be replaced in your own case.
```
source /opt/ros/kinetic/setup.bash
source /usr/share/gazebo-8/setup.sh
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:${LD_LIBRARY_PATH}
```

## Build:
* `cd ~/catkin_ws`
* `catkin_make`

### laikago_controller:
This ros-type controller allow user control joints with position, velocity and torque.

### laikago_msgs:
ros-type message, including command and state of high-level and low-level control.
It would be better if it be compiled firstly, otherwise you may have dependency problems (such as that you can't find the header file).

### laikago_description:
including mesh, urdf and xacro files of laikago.
* `roslaunch laikago_description laikago_rviz.launch`

The robot should be spawned in Rviz.

### laikago_gazebo:
Gazebo8 is recommended. Notice that it is not compatible with other versions like Gazebo7.
Make sure unders have been installed:
```
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-robot-state-publisher ros-kinetic-gazebo8-ros ros-kinetic-gazebo8-ros-control ros-kinetic-gazebo8-ros-pkgs ros-kinetic-gazebo8-ros-dev
```



* `roslaunch laikago_gazebo normal.launch`

The robot should be lying on the ground with joints not activated.

* `rosrun laikago_gazebo laikago_servo`

The robot will stand up slowly.

* `rosrun laikago_gazebo laikago_external_force`

You can add external disturbances with this node, like a push or a kick.

