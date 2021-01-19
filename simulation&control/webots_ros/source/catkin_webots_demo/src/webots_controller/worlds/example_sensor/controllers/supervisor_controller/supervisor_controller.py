# _*_ coding:UTF-8 _*_
"""
supervisor_controller controller.

罗伯特祥
2020-07-20

"""


#from controller import Robot
#from controller import Node
#from controller import Supervisor
#from controller import Field
from controller import *

##################################################
MAXNUM = 200 #最大路径点数

def creatPath(object):
    '''
    添加一个Shape节点，用以创建轨迹
    '''
    pathNode = object.getFromDef("Path")
    if pathNode:
        pathNode.remove()
    code_1  = "DEF Path Shape {\n"
    code_2  = "  appearance Appearance {\n"
    code_3  = "    material Material {\n"
    code_4  = "      diffuseColor 0 1 0\n"
    code_5  = "      emissiveColor 0 1 0\n"
    code_6  = "    }\n"
    code_7  = "  }\n"
    code_8  = "  geometry DEF PathData IndexedLineSet {\n"
    code_9  = "    coord Coordinate {\n"
    code_10 = "      point [\n"
    code_11 = ""
    for i_ in range(MAXNUM):
        code_11 += "      0 0 0\n" 
    code_12 = "      ]\n"
    code_13 = "    }\n"
    code_14 = "    coordIndex [\n"
    code_15 = ""
    for i_ in range(MAXNUM):
        code_15 += "      0 0 -1\n"
    code_16 = "    ]\n"
    code_17 = "  }\n"
    code_18 = "}\n"
    code_   = ""
    for i_ in range(18):
        code_ += eval("code_"+str(i_+1))
        
    root     = robot.getRoot()
    children = root.getField("children") 
    children.importMFNodeFromString(-1,code_)
    return 
##################################################

robot    = Supervisor() # 注意，Supervisor继承自Robot类
timestep = int(robot.getBasicTimeStep())
robot.setLabel(0,"Am Label",0,0,0.1,0xff0000,0,"Arial") #3D界面标签

root     = robot.getRoot()
children = root.getField("children") # 获取子节点字段
count_   = children.getCount()
print("当前World的节点数："+str(count_))

for i_ in range(count_):
    node = children.getMFNode(i_) # 获取字段对应的node
    print("-> " + node.getTypeName())
    
print("=====================================")

creatPath(robot) # 创建路径节点

WorldInfoNode = children.getMFNode(0) # 获取World第一个子节点，即WorldInfo
gravityField  = WorldInfoNode.getField("gravity") # 获取重力加速度字段
gravityData   = gravityField.getSFVec3f() # 获取该字段的值
print("重力加速度：" + str(gravityData))

robot.step(timestep*10)# 延迟的仿真时间，单位ms，其值必须为basicTiimeStep的整数倍
boxNode  = children.getMFNode(3)
boxField = boxNode.getField("size")
size = [1,1,1]
boxField.setSFVec3f(size)

robot.step(timestep*10)
boxField.setSFVec3f([0.6,0.6,0.6])

#robot.step(timestep*10)
#boxNode.remove() # 删除节点后，是不能恢复的

ballNode = children.getMFNode(4)
#ballNode.addTorque([0,1,0],True)  # 添加一个力矩，True是相对于节点本身坐标系的
ballNode.addForceWithOffset([0,100,0],[0.1,0,0],False)
#ballNode.addForce([0,0,100],False) # 添加一个力，False是相对于世界坐标系的
#print(ballNode.getVelocity()) #获取ball的速度
#ballNode.setVelocity([0,0,0,0,0,0]) #基于世界坐标系的
#ballNode.setVelocity([0,0,10,0,0,0])
print("小球位置：" + str(ballNode.getPosition())) # 节点在世界坐标系下的位置
print("小球姿态：" + str(ballNode.getOrientation()))# 节点在世界坐标系下的姿态，3x3正交矩阵
print("=====================================")
#robot.step(timestep*100)
#robot.simulationReset() # 注意，只能重置除Supervisor和机器人控制器以外的其他过程

#robot.step(timestep*20)
#robot.simulationQuit(1) # 直接退出仿真器

#GodNode = children.getMFNode(6)
#GodNode.restartController() #重新启动机器人控制器，注意，重启也是基于当前状态的




pathNode  = robot.getFromDef("PathData")
pathField = pathNode.getField("coord")
coordinatesNode = pathField.getSFNode()
pointField = coordinatesNode.getField("point")
coordIndexField = pathNode.getField("coordIndex")

index = 0
firstStep = 1; # 标志位
while robot.step(timestep) != -1:
    # 获取小球的位置
    # 法1：
    #ballField  = ballNode.getField("translation") # 获取位置字段
    #ballPathData   = ballField.getSFVec3f() # 获取该字段的值
    # 法2：
    pathPoint = ballNode.getPosition()
    pointField.setMFVec3f(index,pathPoint) # 更新Path节点中point数据
    
    if index > 0:
        coordIndexField.setMFInt32(3 * (index - 1), (index - 1))
        coordIndexField.setMFInt32(3 * (index - 1) + 1, index)
    elif index == 0 and firstStep == 0:
        coordIndexField.setMFInt32(3 * (MAXNUM - 1), 0)
        coordIndexField.setMFInt32(3 * (MAXNUM - 1) + 1, MAXNUM-1)   
        
    coordIndexField.setMFInt32(3 * index, index)
    coordIndexField.setMFInt32(3 * index + 1, index)
    firstStep = 0
    index += 1
    index = index % MAXNUM