# racecar_sim
适用于Ubuntu20+ros-Noetic  
*更新说明：*
*由于很多同学选择了ubuntu20，之前的版本在Ubuntu20中会报错，在此进行了更新，如需要Ubuntu 16、18，请切换到对应的分支（master）*  
---
*特别感谢：慕羽★ 在CSDN中的博客：https://blog.csdn.net/qq_44339029/article/details/120762922*  
***
## 常见问题
### 1：缺少必要功能包(建议下载之后把这些都安装了)
**解决方法：**  安装  
```shell
sudo apt-get install ros-noetic-ackermann-msgs
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-openslam-gmapping
sudo apt-get install ros-noetic-geographic-info
sudo apt-get install ros-noetic-controller-manager
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-effort-controllers
sudo apt-get install ros-noetic-joint-state-controller 
sudo apt-get install ros-noetic-position-controllers  
sudo apt-get install ros-noetic-teb-local-planner
```

### 2：Gazebo无法打开（卡在启动界面）
**解决方法：**手动下载模型，给予访问权限
```shell
cd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models.git models

sudo chmod 777 ~/.gazebo/models
sudo chmod 777 ~/.gazebo/models/*
```

### 3: /usr/bin/env: “python”: 没有那个文件或目录
**解决方法：** 设置软连接
```shell
cd /usr/bin
ln -s /usr/bin/python3.8 python
```