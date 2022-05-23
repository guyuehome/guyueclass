import numpy as np


def orientation_error(desired, current):
    rc1 = current[0:3, 0]
    rc2 = current[0:3, 1]
    rc3 = current[0:3, 2]
    rd1 = desired[0:3, 0]
    rd2 = desired[0:3, 1]
    rd3 = desired[0:3, 2]
    error = 0.5 * (np.cross(rc1, rd1) + np.cross(rc2, rd2) + np.cross(rc3, rd3))
    return error


def torque_cartesian(robot, sim, k, d, eef_name, desired_pos, desired_ori):
    M = robot.mass_matrix()
    qd = np.array(sim.data.qvel[:])
    J = robot.jacobian()
    J_inv = np.linalg.inv(J)
    Jd = robot.jacobian_dot()
    M_inv = np.linalg.inv(M)
    Md_inv = np.dot(np.dot(J, M_inv), J.T)
    Md = np.linalg.inv(Md_inv)
    tau = sim.data.qfrc_bias[:].copy()

    x_pos = np.array(sim.data.get_site_xpos(eef_name))
    x_ori = np.array(sim.data.site_xmat[sim.model.site_name2id(eef_name)].reshape([3, 3]))
    x_pos_vel = np.array(sim.data.site_xvelp[sim.model.site_name2id(eef_name)])
    x_ori_vel = np.array(sim.data.site_xvelr[sim.model.site_name2id(eef_name)])

    coef = np.dot(M, J_inv)
    xd_error = np.concatenate([-x_pos_vel, -x_ori_vel])
    sum = np.multiply(d, xd_error)
    pos_error = desired_pos - x_pos
    ori_error = orientation_error(desired_ori, x_ori)
    x_error = np.concatenate([pos_error, ori_error])

    sum += np.multiply(k, x_error)
    sum -= np.dot(np.dot(Md, Jd), qd)
    tau += np.dot(coef, sum)

    return tau
