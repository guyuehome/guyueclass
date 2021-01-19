Copyright 2020 by 罗伯特祥

# 1. RotationalMotor

```python
# -*- coding: utf-8 -*-
"""motor_controller controller."""

from controller import Robot

# 实例化机器人
robot = Robot()

# 获取基本仿真步长
timestep = int(robot.getBasicTimeStep())

# 关联设备
motor = robot.getMotor('my_Rmotor')

# 设置电机运行模式为速度模式
motor.setPosition(float('inf'))
motor.setVelocity(1)

# Main loop:
while robot.step(timestep) != -1:
    pass
```



# 2. LinearMotor
```py
# -*- coding: utf-8 -*-
"""linear motor controller."""

from controller import Robot
import math


robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor = robot.getMotor('my_lMotor')
#motor.setPosition(float('inf'))
#motor.setVelocity(0)

# Main loop:
count = 0
while robot.step(timestep) != -1:
    #motor.setVelocity(1)
    motor.setPosition(math.sin(count)*0.5)
    count += 0.1
```

# 3. Brake
- `setDampingConstant(dampingConstant)`函数设置关节的阻尼常数（单位Ns/m或者Nms），如果`JointParameters`设置了阻尼常数，那么所产生的的`dampingConstant`系数是`JointParameters`中的`dampingConstant`与使用该函数所设置的`dampingConstant`的和。即：
$$
实际作用的dampingConstant = Joint中的dampingConstant + Function设置的dampingConstant
$$
```py
# -*- coding: utf-8 -*-
"""brake controller."""

from controller import Robot
from controller import Motor
from controller import Brake

robot = Robot()

timestep = int(robot.getBasicTimeStep())

motor = robot.getMotor('my_Rmotor')
#motor.enableTorqueFeedback(timestep)
brake = robot.getBrake('my_brake')

motor.setTorque(0)

# Main loop:
count = 0
while robot.step(timestep) != -1:
    count += 1
    print(count)
    #print("motor torque"+str(motor.getTorqueFeedback()))
    motor.setTorque(0.5)
    if count > 50:
        brake.setDampingConstant(1)
    if count >300:
        count = 0
        brake.setDampingConstant(0)
```


# 4. Propeller
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200712231231332.png#pic_center)
- 合成推力计算：$$T = t1 * |omega| * omega - t2 * |omega| * V$$
其中`thrustConstants`中定义的两个常数为式中的t1和t2，omega为电机角速度，V是推力中心沿轴心线速度的分量。推力施加在`centerOfThruse`中指定的点上
- 合成力矩计算：$$Q = q1 * |omega| * omega - q2 * |omega| * V$$
	其中q1和q2为`torqueConstants`中定义的常数
> 以上公式计算来自：Thor I. Fossen的"Guidance and Control of Ocean Vehicles"和Raymond W. Prouty的"Helicopter Performance, Stability, and Control"

几个重要节点：
- `shaftAxis`：定义沿其施加合力和扭矩的轴
- `centerOfThrust`：定义推力的施加点
- `thrustConstants`：定义推力计算公式中的两个常数，正旋与反旋只需将此字段中的数值设置为相反数。即正旋时，此字段数值为正，那么反旋则为此数值的负值即可
- `torqueConstants`：定义转矩计算公式中的两个常数
>  `thrustConstants`和`torqueConstants`中的常数，在现实中，其数值由螺旋桨的倾角和方向决定
- `fastHelixThreshold`：从`slowHelix`切换到`fastHelix`阀值，默认为24π rad/s
- `device`：旋转电机放置的设备节点
- `fastHelix`和`slowHelix`：如果不为`NULL`，那么必须使用Solid节点设置这些属性；如果$|omega|>fastHelixThreshold$，那么只有在`fastHelix`中定义的Solid是可见的，反之则只有在`slowHelix`中定义的Solid是可见的。

# 5. Pen
通常用于显示机器人的移动轨迹，笔的绘制方向与节点`-y`向重合
几个重要节点：
- `inkColor`：定义笔迹颜色，可通过函数定义/修改
- `inkDensity`：定义颜色密度，在[0,1]之间
- `leadSize`：定义笔迹宽度
- `maxDistance`：定义笔与绘制表明之间最大距离，该值≤0表示绘制距离无限
- `write`：使能笔的书写功能，可由函数控制
- `WorldInfo`节点的`inkEvaporation`控制墨水消失的速度

遗憾的是，只能在对象上绘制
# 6. LED
LED虽然在实际控制中没什么作用，但是通过LED的灯光显示我们可以传递一些信息，尤其是在实际机器人当中。


- ① 颜色设置在`LED`节点下的`color`设置
- ② `LED.set(2)`时，打开第二种颜色
	
> 注意，设置的值不能超过`color`中设置的颜色种类
	
- ③ `gradual`节点定义`LED`节点的类型。若`gradual=TRUE`，`color`列表为空，则为RGB LED，`set()`函数此时接收的值为十六进制RGB颜色值（即R8G8B8），例如红色为`0xff0000`；`gradual=FALSE`，包含只有一种颜色，则为单色LED；包含多种颜色，则为多色LED；
	

```py
"""LED_controller controller."""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

LED = robot.getLED('my_led')

# Main loop:
count = 0
while robot.step(timestep) != -1:
    count += 1
    print(count)
    print("LED状态：" + str(LED.get()))
    if count > 20:
        LED.set(1)# 打开LED，强度255
    if count > 40:
        LED.set(0)# 关闭LED，强度0
        count = 0
```
```py
# Main loop:
count = 0
while robot.step(timestep) != -1:
    count += 1
    print(count)
    print("LED状态：" + str(LED.get()))
    if count > 20:
        LED.set(0xff0000)
    if count > 40:
        LED.set(0x00ff00)
    if count > 60:
        LED.set(0)
        count = 0
```

> 如果`gradual=TRUE`，而color值又非空，那么通过十六进制设置灯光颜色时，LED会发处一个光团，且颜色为color列表的第一种
![在这里插入图片描述](https://img-blog.csdnimg.cn/20200712211118181.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MzQ1NTU4MQ==,size_16,color_FFFFFF,t_70#pic_center)
---
参考文献：
- [https://cyberbotics.com/doc/reference/index](https://cyberbotics.com/doc/reference/index)
