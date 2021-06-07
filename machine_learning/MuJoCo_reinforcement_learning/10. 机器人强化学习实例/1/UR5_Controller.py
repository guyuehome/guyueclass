import mujoco_py as mp
import numpy as np
from gym import spaces


class UR5_Controller(object):
    def __init__(self, target):
        self.model = mp.load_model_from_path('ur5.xml')
        self.sim = mp.MjSim(self.model)
        self.viewer = mp.MjViewer(self.sim)
        self.state = [0] * 12
        self.action_high = np.array([2, 2, 2, 1, 1, 1], dtype=np.float32)
        self.action_space = spaces.Box(-self.action_high, self.action_high, dtype=np.float32)
        self.observation_high = np.array([np.finfo(np.float32).max] * 12)
        self.observation_space = spaces.Box(-self.observation_high, self.observation_high, dtype=np.float32)
        self.target = target

    def step(self, action):
        for i in range(6):
            self.sim.data.ctrl[i] = action[i]
        self.sim.step()
        r = 0
        for i in range(6):
            r -= abs(self.sim.data.qpos[i] - self.target[i])
            # r -= abs(self.sim.data.qvel[i])
            self.state[i] = self.sim.data.qpos[i]
            self.state[i + 6] = self.sim.data.qvel[i]
        done = False
        # if r < -50:
        #     done = True
        # print(r)
        return np.array(self.state, dtype=np.float32), r, done, {}

    def render(self):
        self.viewer.render()

    def reset(self):
        self.sim.reset()
        for i in range(6):
            self.sim.data.qpos[i] = 0
            self.sim.data.qvel[i] = 0
        self.state = [0] * 12
        return np.array(self.state, dtype=np.float32)

    def close(self):
        pass