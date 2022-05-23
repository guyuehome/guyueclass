import utils
import numpy as np
import ikfastpy


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


def torque_joint(robot, sim, k, d, desired_pos, desired_ori, tau_last):
    q = np.array(sim.data.qpos[:])
    qd = np.array(sim.data.qvel[:])
    q_target = ik(robot, q, desired_pos, desired_ori)
    M = robot.mass_matrix()
    ok = False
    tau = tau_last
    if len(q_target) > 0:
        tau = np.multiply(k, q_target - q) - np.multiply(d, qd)
        tau = np.dot(M, tau)
        tau += robot.coriolis_gravity()
        ok = True
    return tau, ok