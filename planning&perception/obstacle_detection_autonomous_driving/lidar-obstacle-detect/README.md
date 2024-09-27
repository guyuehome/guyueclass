
1.安装相应的 ros 依赖包
```
sudo apt-get install ros-melodic-jsk-rviz-plugins
sudo apt-get install ros-melodic-jsk-recognition-msgs
sudo apt-get install ros-melodic-autoware-msgs
sudo apt-get install ros-melodic-visualization-msgs
```

2.启动
```
rosbag play -l kitti_2011_09_30_drive_0016_synced.bag /kitti/velo/pointcloud:=/velodyne_points
roslaunch lidar_obstacle_detection lidar_obstacle_detection.launch
```
launch有两个节点，运行launch直接会打开rviz可视化

```bash
<launch>
    <node pkg="lidar_obstacle_detection" type="lidar_obstacle_detection_node" name="lidar_obstacle_detection_node" output="screen" />
        <rosparam file="$(find lidar_obstacle_detection)/config/lidar_obstacle_detection.yaml" command="load" />
    <!-- Start lidar_obstacle_detection.rviz -->
    <node pkg="rviz" type="rviz" name="rviz" output = "screen" args="-d $(find lidar_obstacle_detection)/rviz/lidar_obstacle_detection.rviz" required="true" />
</launch>
```

用kitti数据集，效果如下，大家可以自行根改config下的launch文件里的参数进行调试
![在这里插入图片描述](https://img-blog.csdnimg.cn/0df1b01937b141b0a2e04031cdf49859.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA5Y2O5bGx5Luk54uQ5Yay44CB,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/e1bc58466d094f3dbabb49aa5f493b57.png#pic_center)
可视化MarkerArray输出包括5个信息：
- label：显示距离信息
- hull：凸多边形拟合，绿色线
- cube:蓝色的实心box
- box：红色线条围成的box
- centroid：bounding_box中心

上图为bounding_box未加方向，加方向后，效果不太好，对于道路两边的区域的姿态估计显然会不准确，可以将`isEstimatePose`字段设置为true，观测添加姿态估计后的效果


