# _*_ coding:UTF-8 _*_
"""
four_wheels_path_controller.
Author:罗伯特祥
"""


#from controller import Robot
#from controller import Node
#from controller import Supervisor
#from controller import Field
from controller import *

MAXNUM = 1000 #最大路径点数

##################################################
###                  模板勿动                  ###
##################################################
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

root     = robot.getRoot()
children = root.getField("children") # 获取子节点字段
count_   = children.getCount()
print("当前World的节点数："+str(count_))

for i_ in range(count_):
    node = children.getMFNode(i_) # 获取字段对应的node
    print("-> ID:%s "%str(i_) + node.getTypeName())
    
print("=====================================")
##################################################


""" 此处修改目标节点 """
carNode = children.getMFNode(5) # 获取World第5个子节点，即四轮小车
carChildren = carNode.getField("children")
count_ = carChildren.getCount()
for i_ in range(count_):
    node = carChildren.getMFNode(i_)
    print("-> ID:%s "%str(i_) + node.getTypeName())

carBase = carChildren.getMFNode(4) # 获取base变换节点
baseTrans = carBase.getField("translation")



##################################################
###                  模板勿动                  ###
##################################################
creatPath(robot) # 创建路径节点
pathNode  = robot.getFromDef("PathData")
pathField = pathNode.getField("coord")
coordinatesNode = pathField.getSFNode()
pointField = coordinatesNode.getField("point")
coordIndexField = pathNode.getField("coordIndex")
index = 0
firstStep = 1; # 标志位
##################################################


while robot.step(timestep) != -1:
    """ 此处修改跟随的数据 """
    carData = carNode.getPosition()
    baseData = baseTrans.getSFVec3f()
    pathPoint = [carData[0]+baseData[0],carData[1]+baseData[1],carData[2]+baseData[2]]
    print(pathPoint)



##################################################
###                  模板勿动                  ###
##################################################
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
##################################################
