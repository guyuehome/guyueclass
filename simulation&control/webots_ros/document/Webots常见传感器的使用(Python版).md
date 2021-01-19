@[toc]

Copyright 2020 by 罗伯特祥

# 1. GPS

```python
from controller import Robot
from controller import GPS

robot = Robot()
timeStep = getBasicTimeStep()
gps = robot.getGPS("gps_name")
gps.enable(timeStep)
gps.disable()

# 获取当前GPS测量值，3D向量（3个浮点数列表），绝对位置
gps.getValues()

# 获取速度，单位m/s
gps.getSpeed()

```

- `WorldInfo`节点中的`gpsCoordinateSystem`用于设置数据是在webots笛卡尔坐标系中表示此位置，还是使用经纬度表示此位置
- `WorldInfo`节点中的`gpsReference`用于定义GPS的参考点
# 2. Camera
```python
from controller import Robot
from controller import Camera

robot = Robot()
timeStep = getBasicTimeStep()
camera = robot.getCamera("Camera_name")
camera.enable(timeStep)
camera.disable()

camera.setFocalDistance(0.01) #设置焦距
camera.getWidth() #获取图像宽度
camera.getHeight() #获取图像高度

# 获取图像
cameraData = camera.getImage()
# 得到像素的灰度分量(5,10)及三通道值
gray = camera.imageGetGray(cameraData, camera.getWidth(), 5, 10)
r = camera.imageGetRed(cameraData,camera.getWidth(),5,10)
g = camera.imageGetGreen(cameraData,camera.getWidth(),5,10)
b = camera.imageGetBlue(cameraData,camera.getWidth(),5,10)

image = camera.getImageArray()
# 显示每个像素的组成部分
for x in range(0,camera.getWidth()):
  for y in range(0,camera.getHeight()):
    red   = image[x][y][0]
    green = image[x][y][1]
    blue  = image[x][y][2]
    gray  = (red + green + blue) / 3
    print 'r='+str(red)+' g='+str(green)+' b='+str(blue)
```
焦点设置，参考`Focus`节点



# 3. Accelerometer
加速度计单位：m/s^2
```python
from controller import Robot
from controller import Accelerometer

robot = Robot()
timeStep = getBasicTimeStep()
acc = robot.getAccelerometer("Accelerometer_name")
acc.enable(timeStep)
acc.disable()
acc.getValues() #返回3D向量（python中返回三个浮点数列表）
```
注意，会受重力加速度影响，为了获取只由运动引起的加速度，必须减去这个偏移量。
# 4. Gyro
陀螺仪单位：rad/s
```python
from controller import Robot
from controller import Gyro

robot = Robot()
timeStep = getBasicTimeStep()
gyro = robot.getGyro("Gyro_name")
gyro.enable(timeStep)
gyro.disable()
gyro.getValues()
```
# 5. InertialUnit
惯性测量单元用于测量RPY，如果测量加速度读或角速度需要使用Accelerometer和Gyro节点，InertialUnit的X轴必须同机器人前向相同，Z轴指向机器人右侧，Y轴指向机器人正上方，

![在这里插入图片描述](https://img-blog.csdnimg.cn/20200706203859962.png#pic_center)

```python
from controller import Robot
from controller import InertialUnit

robot = Robot()
timeStep = getBasicTimeStep()
Inertial = robot.getInertialUnit("InertialUnit_name")
Inertial.enable(timeStep)
Inertial.disable()
Inertial.getRollPitchYaw() #获取RPY数据
```
> 注意：在万向节锁定情况下，即，当俯仰角为-π/ 2或π/ 2时，横滚和横摆设置为NaN
# 6. DistanceSensor
距离传感器，包括激光传感器、红外线传感器、声纳/超声波传感器，默认为通用距离传感器，可以指定以上三种类型，值的单位可通过`查表`自己通过映射关系设置，默认单位为m，但其返回值不是m的单位
```python
from controller import Robot
from controller import DistanceSensor

robot = Robot()
timeStep = getBasicTimeStep()
dist = robot.getDistanceSensor("DistanceSensor_name")
dist.enable(timeStep)
dist.disable()
dist.getValue()
```

# 7. Compass
模拟数字罗盘（磁传感器），向量指向虚拟北方，虚拟北方由`WorldInfo`中的`northDirection`指定
```python
from controller import Robot
from controller import Compass

robot = Robot()
timeStep = getBasicTimeStep()
Compass = robot.getCompass("Compass_name")
Compass.enable(timeStep)
Compass.disable()
Compass.getValues()
```
# 8. Lidar

激光雷达，单位m，参数较多，详细参照手册
```python
from controller import Robot
from controller import LidarPoint
from controller import Lidar

robot = Robot()
timeStep = getBasicTimeStep()
Lidar = robot.getLidar("Lidar_name")
Lidar.enable(timeStep) #启动更新
Lidar.disable()

Lidar.enablePointCloud() #启动激光雷达点云更新
Lidar.disablePointCloud() #
Lidar.isPointCloudEnabled() #判断是否已经启动点云更新，是则返回true

Lidar.getRangeImage() # 读取激光雷达捕获的最后一个范围图像的内容，一维列表
Lidar.getRangeImageArray() #与上同，但返回的是二维列表

Lidar.getPointCloud() #获取点数组
Lidar.getNumberOfPoints() #获取总数

```
# 9. PositionSensor
单位为rad或m
```python
from controller import Robot
from controller import PositionSensor

robot = Robot()
timeStep = getBasicTimeStep()
pos = robot.getPositionSensor("PositionSensor_name")
pos.enable(timeStep)
pos.disable()
pos.getValue()
```
# 10. Radar
用于模拟雷达传感器，用来测量其他固体距离、角度和相对速度
```python
from controller import Robot
from controller import Radar

robot = Robot()
timeStep = getBasicTimeStep()
Radar = robot.getRadar("Radar_name")
Radar.enable(timeStep)
Radar.disable()
Radar.getNumberOfTargets() #获取当前目标数目

# 其他
```
# 11. RangeFinder
测距仪，深度相机那种
```python
from controller import Robot
from controller import RangeFinder

robot = Robot()
timeStep = getBasicTimeStep()
RangeFinder = robot.getRangeFinder("RangeFinder_name")
RangeFinder.enable(timeStep)
RangeFinder.disable()
RangeFinder.getWidth()
RangeFinder.getHeight()

imageData = getRangeImage()
imageArray = getRangeImageArray()
RangeFinder.rangeImageGetDepth(imageData,RangeFinder.getWidth(),5,10)
```


# 12.TouchSensor
```python
from controller import Robot
from controller import TouchSensor

robot = Robot()
timeStep = getBasicTimeStep()
TouchSensor = robot.getTouchSensor("TouchSensor_name")
TouchSensor.enable(timeStep)
TouchSensor.disable()
TouchSensor.getValues() #仅适用于force-3d的情况

```


> 注意，噪声和分辨率需要研究`lookupTable`和`resolution`


# 13.Supervisor
这不是一个实体节点，也不是一个传感器，暂时把它归类到传感器中，我个人称其为“上帝节点”，它可以代替某些传感器，本质来说，它们是一种东西，唯一不同的是，这个“上帝”可以直接对环境进行操作。

---
参考文献：
- [https://cyberbotics.com/doc/reference/index](https://cyberbotics.com/doc/reference/index)
