# -- coding:UTF-8 --
"""
Created on Tues Aug  3 17:06:02 2021

@author: wjx
"""
from math import cos, sin, sqrt, atan2, degrees, radians


def inverse_kinematics(x, y, z, angle=180):
    l1 = 105  # 连杆1长度
    l2 = 135  # 连杆2长度
    l3 = 130  # 连杆3长度
    eh = 80
    ANGLE_ERR = 180
    distance = sqrt(x ** 2 + y ** 2 + z ** 2)
    if distance > (l1 + l2 + l3):
        print("够不着")
        return False, None, None, None, None

    if x == 0 and y == 0:
        j1 = 0
    else:
        j1 = atan2(y, x)

    if j1 > radians(135):
        j1 = j1 - radians(180)
        angle = -angle
    if j1 < radians(-135):
        j1 = j1 + radians(180)
        angle = -angle

    if x != 0:
        len = x / cos(j1)
    else:
        len = abs(y)

    count = 0
    angle_up = angle_low = angle
    up_true_low_false = True
    is_out_range = False
    while count <= ANGLE_ERR:
        rad = radians(angle)
        a = len - l3 * sin(rad)
        b = z - eh - l3 * cos(rad)

        cos_j3 = ((pow(a, 2) + pow(b, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l2 * l1))
        if abs(cos_j3) <= 1:
            sin_j3 = sqrt(1 - pow(cos_j3, 2))
            j3 = atan2(sin_j3, cos_j3)
            k1 = l1 + l2 * cos_j3
            k2 = l2 * sin_j3
            w = atan2(k2, k1)
            j2 = atan2(a, b) - w

            j4 = rad - j2 - j3

            x1 = (l1 * sin(j2) + l2 * sin(j2 + j3) + l3 * sin(j2 + j3 + j4)) * cos(j1)
            y1 = (l1 * sin(j2) + l2 * sin(j2 + j3) + l3 * sin(j2 + j3 + j4)) * sin(j1)
            z1 = l1 * cos(j2) + l2 * cos(j2 + j3) + l3 * cos(j2 + j3 + j4) + eh
            deg_j1 = degrees(j1)
            deg_j2 = degrees(j2)
            deg_j3 = degrees(j3)
            deg_j4 = degrees(j4) - 15

            print("结果：j1: {} ,j2: {} ,j3: {} ,j4: {} ,angle: {} ".format(deg_j1, deg_j2, deg_j3, deg_j4, angle))
            print("运动学正解结果：x:%f,y:%f,z:%f\r\n" % (x1, y1, z1))

            if deg_j1 <= 135 and deg_j1 >= -135 and deg_j2 <= 90 and deg_j2 >= -90 and deg_j3 <= 135 and deg_j3 >= -135 and deg_j4 <= 90 and deg_j4 >= -135:
                return True, deg_j1, deg_j2, deg_j3, deg_j4
            else:
                print("超出约束")
                is_out_range = True

        # 试试两边的角度有没有解
        if abs(cos_j3) > 1 or is_out_range == True:
            if up_true_low_false == True:
                angle_up = angle_up + 1
                angle = angle_up

            if up_true_low_false == False:
                angle_low = angle_low - 1
                angle = angle_low
                count += 1
            up_true_low_false = bool(1 - up_true_low_false)
    if count > ANGLE_ERR:
        print("无解")
        return False, None, None, None, None


if __name__ == '__main__':
    has_sol, j1, j2, j3, j4 = inverse_kinematics(300, 0, 0, 180)
