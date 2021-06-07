from UR5_Controller import UR5_Controller
import math

PI = 3.1415926

ur5 = UR5_Controller()
target = [0, -PI/2, 0, 0, 0, 0]
ur5.move_group_to_joint_target('Arm', target, plot=True)
ur5.stay(3000)
target = [0, 0, 0, 0, 0, 0]
ur5.move_group_to_joint_target('Arm', target, plot=True)
ur5.stay(3000)

def sin_cal(A, w, t):
    return A * math.sin(w * t)

trajectory = []
precision = 0.001

A = []
w = []
for i in range(6):
    A.append((7 - i) * 0.2)
    w.append((i + 1) * 0.5)

for i in range(10000):
    tmp = []
    for k in range(6):
        tmp.append([sin_cal(A[k], w[k], i * precision)])
    trajectory.append(tmp)

result = ur5.move_group_along_trajectory(group='Arm', target=trajectory, plot=True)
print("result: {}".format(result))
ur5.stay(5000)
