# Webots_ROS_Darwin_op3
Using ROS to control webots Darwin_op3 in Webots
## 仿真环境配置与插件安装
1. ROS(melodic)安装
参见http://wiki.ros.org/melodic/Installation/Ubuntu

2. Webots(2021a)安装与测试
sudo apt install ./webots_2021a_amd64.deb
webots
echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
source ./bashrc 

3. webots_ros安装
sudo apt install ros-melodic-webots-ros
roslaunch webots_ros e_puck_line.launch

4. multiplot安装
sudo apt install ros-melodic-rqt-multiplot

5. QtCreator
下载: https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html
增加可执行权限：sudo chmod +x .run
QtCreator中文语言使能
安装fcitx
cd /usr/lib/x86_64-linux-gnu/qt5/plugins/platforminputcontexts/
cp libfcitxplatforminputcontextplugin.so ~/QtCreator/latest/lib/Qt/plugins/platforminputcontexts/
cd ~/QtCreator/latest/lib/Qt/plugins/platforminputcontexts
sudo chmod +x libfcitxplatforminputcontextplugin.so

## ROS工程编译与依赖安装
1.工程编译所依赖安装
sudo apt-get install ros-melodic-qt-create
sudo apt-get install ros-melodic-qt-build
sudo apt-get install qt4-qmake
sudo apt-get install qt4-default

sudo apt-get install ros-melodic-teleop-twist-keyboard

2.单独编译消息包
catkin_make -DCATKIN_WHITELIST_PACKAGES="robotis_controller_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES="op3_walking_module_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

3.将Webots控制器库文件所在目录添加到环境变量LD_LIBRARY_PATH
export LD_LIBRARY_PATH=LD_LIBRARY_PATH:/usr/local/webots/lib/controller
