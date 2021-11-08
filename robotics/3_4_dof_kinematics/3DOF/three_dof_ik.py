# -- coding:UTF-8 --
"""
Created on Tues Aug  3 17:06:02 2021

@author: wjx
"""
from math import cos, sin, sqrt, atan2, degrees


def inverse_kinematics(x, y, z):
    l1 = 150  # 连杆1长度
    l2 = 160  # 连杆2长度
    el = 60  # 水平误差
    eh = 35  # 垂直误差
    distance = sqrt(x ** 2 + y ** 2 + z ** 2)
    if distance > (l1 + l2 + el):
        print("够不着")
        return False, None, None, None

    if x == 0 and y == 0:
        j1 = 0
    else:
        j1 = atan2(y, x)

    if x != 0:
        len = x / cos(j1)
    else:
        len = abs(y)

    a = len - el
    b = z - eh

    cos_j3 = ((pow(a, 2) + pow(b, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l2 * l1))
    sin_j3 = sqrt(1 - pow(cos_j3, 2))
    j3 = atan2(sin_j3, cos_j3)
    k1 = l1 + l2 * cos_j3
    k2 = l2 * sin_j3
    w = atan2(k2, k1)
    j2 = atan2(a, b) - w

    x1 = (l1 * sin(j2) + (l2 * sin(j2 + j3)) + el) * cos(j1)
    y1 = (l1 * sin(j2) + (l2 * sin(j2 + j3)) + el) * sin(j1)
    z1 = (l1 * cos(j2) + l2 * cos(j2 + j3)) + eh
    deg_j1 = degrees(j1)
    deg_j2 = degrees(j2)
    deg_j3 = degrees(j3) - 90 + deg_j2

    print("结果：j1: {} ,j2: {} ,j3: {} ".format(deg_j1, deg_j2, deg_j3))
    if deg_j1 > 135 or deg_j1 < -135 or deg_j2 > 100 or deg_j2 < -60 or deg_j3 > 135 or deg_j3 < 20:
        print("超出约束")
        return False, None, None, None
    print("运动学正解结果：x:%f,y:%f,z:%f\r\n" % (x1, y1, z1))

    return True, deg_j1, deg_j2, deg_j3


if __name__ == '__main__':
    has_sol, j1, j2, j3 = inverse_kinematics(300, 0, 0)
    # y = 100
    # x = 0
    # j1 = atan2(y, x)
    # print(j1, cos(j1))
