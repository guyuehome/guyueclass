import mujoco_py as mp
from joint_ipd import torque_joint
from cartesian_ipd import torque_cartesian
import func
import numpy as np
import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

end_x = [0] * 300
end_y = [0] * 300
end_z = [0] * 300

def update_x():
    global end_x, curve_x
    end_x[:-1] = end_x[1:]
    end_x[-1] = sim.data.get_site_xpos('ee')[0]
    curve_x.setData(end_x)


def update_y():
    global end_y, curve_y
    end_y[:-1] = end_y[1:]
    end_y[-1] = sim.data.get_site_xpos('ee')[1]
    curve_y.setData(end_y)


def update_z():
    global end_z, curve_z
    end_z[:-1] = end_z[1:]
    end_z[-1] = sim.data.get_site_xpos('ee')[2]
    curve_z.setData(end_z)


def update_xy():
    curve_xy.setData(end_x, end_y)


if __name__ == '__main__':
    model = mp.load_model_from_path('ur5.xml')
    sim = mp.MjSim(model)
    viewer = mp.MjViewer(sim)
    Rbt = func.Robot(sim, 'ee')
    init = [-3.1, -1.6, 1.6, -1.6, -1.6, 0]
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    for i in range(6):
        sim.data.set_joint_qpos(joint_names[i], init[i])
    sim.forward()
    pos, ori = Rbt.fk()
    desired_pos = pos.copy()
    desired_ori = ori.copy()
    kj, dj = np.array([20] * 6, dtype=np.float32), np.array([100] * 6, dtype=np.float32)
    kc = np.array([300, 300, 300, 600, 600, 600], dtype=np.float32)
    dc = np.array([120, 120, 120, 70, 70, 70], dtype=np.float32)
    last_tau = 0
    steps = 0
    body_id = sim.model.body_name2id('wrist_3_link')

    app = QtGui.QApplication([])
    win = pg.GraphicsLayoutWidget(show=True, title="End Effector Position")
    win.resize(800, 200)
    win.setWindowTitle('End Effector Position')
    pg.setConfigOptions(antialias=True)

    timer = QtCore.QTimer()

    px = win.addPlot(title="Position-X")
    curve_x = px.plot(pen='y')
    py = win.addPlot(title="Position-Y")
    curve_y = py.plot(pen='y')
    pz = win.addPlot(title="Position-Z")
    curve_z = pz.plot(pen='y')
    circle = win.addPlot(title='Position-XY')
    curve_xy = circle.plot(pen='r')

    timer.timeout.connect(update_x)
    timer.timeout.connect(update_y)
    timer.timeout.connect(update_z)
    timer.timeout.connect(update_xy)
    timer.start(1)

    while 1:
        desired_pos[0] = pos[0] + 0.1 * math.cos(steps / 180 * np.pi)
        desired_pos[1] = pos[1] + 0.1 * math.sin(steps / 180 * np.pi)
        tau, ok = torque_joint(Rbt, sim, kj, dj, desired_pos, desired_ori, last_tau)
        # tau = torque_cartesian(Rbt, sim, kc, dc, 'ee', desired_pos, desired_ori)
        if 1000 <= steps < 1100:
            sim.data.xfrc_applied[body_id][0] = 100
        else:
            sim.data.xfrc_applied[body_id][0] = 0
        sim.data.ctrl[:] = tau
        last_tau = tau.copy()
        sim.step()
        steps += 1
        viewer.render()
