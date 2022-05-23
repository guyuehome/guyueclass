import numpy as np


class RobotMj:
    def __init__(self, sim, ee):
        self.sim = sim
        self.ee_name = ee
        self.index = np.arange(0, 6)

    def fk(self):
        pos = self.sim.data.get_site_xpos(self.ee_name)
        ori = np.array(self.sim.data.site_xmat[self.sim.model.site_name2id(self.ee_name)].reshape([3, 3]))
        return pos, ori

    def jacobian(self):
        J_pos = np.array(self.sim.data.get_site_jacp(self.ee_name).reshape((3, -1))[:, self.index])
        J_ori = np.array(self.sim.data.get_site_jacr(self.ee_name).reshape((3, -1))[:, self.index])
        J_full = np.array(np.vstack([J_pos, J_ori]))
        return J_full

