# Tensorflow Object Detector with ROS

## Requirements:

Tensorflow and ROS

This guide targets Ubuntu 16.04 and ROS Kinetic

## Steps:

To run Default SSD (Single Shot Detection) algorithm:

1. Install ROS: http://wiki.ros.org/kinetic/Installation/Ubuntu

2. Install camera dependencies

    `sudo apt-get install ros-kinetic-usb_cam ros-kinetic-openni2-launch`

3. Install tensorflow into python virtualenv: https://www.tensorflow.org/install/install_linux

    `sudo apt-get install python-pip python-dev python-virtualenv`

    `virtualenv --system-site-packages ~/tensorflow`

    `source ~/tensorflow/bin/activate`

    `easy_install -U pip`

    `pip install --upgrade tensorflow`

4. `mkdir ~/catkin_ws/ && mkdir ~/catkin_ws/src/`

5. Clone standard Vision messages repository and this repository into `catkin_ws/src`:

    `cd ~/catkin_ws/src`

    `git clone https://github.com/Kukanani/vision_msgs.git`

    `git clone https://github.com/osrf/tensorflow_object_detector.git`

6. Build tensorflow_object_detector and Vision message 

    `cd ~/catkin_ws && catkin_make`

7. Source catkin workspace's setup.bash:

    `source ~/catkin_ws/devel/setup.bash`

8. Plug in camera and launch Single Shot Detector (varies per camera, NOTE: `object_detect.launch` also launches the openni2.launch file for the camera. If you are using any other camera, please change the camera topic in the launch file before launching the file)

    `roslaunch tensorflow_object_detector object_detect.launch`

    OR

    `roslaunch tensorflow_object_detector usb_cam_detector.launch`



If you want to try any other ML model:

1. Download any Object Detection Models from the Tensorflow Object detection API and place it in `data/models/`. You can find the models in tensorflow Object Detection Model Zoo: https://github.com/tensorflow/models/blob/master/object_detection/g3doc/detection_model_zoo.md. Extract the `tar.gz` file.

2. Edit the MODEL_NAME and LABEL_NAME in detect_ros.py. By default it is `ssd_mobilenet_v1_coco_11_06_2017` with `mscoco_label_map.pbtxt` respectively. 

