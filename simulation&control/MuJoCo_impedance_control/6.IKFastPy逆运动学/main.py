import mujoco_py as mp
import utils
import numpy as np
import ikfastpy
import func_kdl

solver = ikfastpy.PyKinematics()
n_joints = solver.getDOF()


def ik(rbt, init, pos, ori):
    pos, ori, ok = utils.check_pos_ori_valid(pos, ori)
    if not ok:
        return []
    ee_pos = np.append(ori, pos, axis=1)
    joint_configs = solver.inverse(ee_pos.reshape(-1).tolist())
    n_solutions = int(len(joint_configs) / n_joints)
    joint_configs = np.asarray(joint_configs).reshape(n_solutions, n_joints)
    min_norm = np.inf
    res = []
    for i in range(n_solutions):
        norm = np.linalg.norm(joint_configs[i] - init)
        if norm < min_norm:
            res = joint_configs[i]
            min_norm = norm
    if n_solutions == 0:
        res = rbt.ik(init, pos, ori)
        if sum(abs(res - init)) > 3 / 180 * np.pi:
            res = []
    return res


if __name__ == '__main__':
    model = mp.load_model_from_path('ur5.xml')
    sim = mp.MjSim(model)
    RbtKdl = func_kdl.RobotKdl(sim)
    init = [-3.1, -1.6, 1.6, -1.6, -1.6, 0]
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    for i in range(6):
        sim.data.set_joint_qpos(joint_names[i], init[i])
    sim.forward()
    pos, ori = RbtKdl.fk()
    pos, ori = np.array(pos), np.array(ori)
    init = [-3.1, -1.5, 1.6, -1.6, -1.6, 0]
    print(ik(RbtKdl, init, pos, ori))

