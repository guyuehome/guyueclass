import numpy as np
import gym
from gym.utils import seeding
from gym import spaces, logger
import time

import sys
sys.path.append('../VREP_RemoteAPIs')
import sim as vrep_sim

from CartPoleModel import CartPoleModel

class CartPoleEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, action_type='discrete'):
        super(CartPoleEnv, self).__init__()
        self.action_type = action_type
        self.push_force = 0
        self.q = [0.0, 0.0]
        self.q_last = [0.0, 0.0]

        self.theta_max = 40*np.pi / 360
        self.cart_pos_max = 0.8

        high = np.array(
            [
                self.cart_pos_max,
                self.theta_max,
                1000000.0,
                1000000.0
            ],
            dtype=np.float32,
        )

        if self.action_type == 'discrete':
            self.action_space = spaces.Discrete(3)
        elif self.action_type == 'continuous':
            self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        else:
            assert 0, "The action type \'" + self.action_type + "\' can not be recognized"

        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

        self.seed()
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
        self.counts = 0
        self.steps_beyond_done = None

        # Connect to VREP (CoppeliaSim)
        vrep_sim.simxFinish(-1) # close all opened connections
        while True:
            client_ID = vrep_sim.simxStart('127.0.0.1', 19997, True, False, 5000, 5) # Connect to CoppeliaSim
            if client_ID > -1: # connected
                print('Connect to remote API server.')
                break
            else:
                print('Failed connecting to remote API server! Try it again ...')

        # Open synchronous mode
        vrep_sim.simxSynchronous(client_ID, True) 
        vrep_sim.simxStartSimulation(client_ID, vrep_sim.simx_opmode_oneshot)
        vrep_sim.simxSynchronousTrigger(client_ID)

        self.cart_pole_sim_model = CartPoleModel()
        self.cart_pole_sim_model.initializeSimModel(client_ID)
        vrep_sim.simxSynchronousTrigger(client_ID)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        if self.action_type == 'discrete':
            assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))

        q = [0.0, 0.0]
        q[0] = self.cart_pole_sim_model.getJointPosition('joint_1')
        q[1] = self.cart_pole_sim_model.getJointPosition('joint_2')
        self.q_last = self.q
        self.q = q

        if self.action_type == 'discrete':
            if action == 0:
                self.push_force = 0
            elif action == 1:
                self.push_force = 1.0
            elif action == 2:
                self.push_force = -1.0
        elif self.action_type == 'continuous':
            self.push_force = action*2.0 # The action is in [-1.0, 1.0], therefore the force is in [-2.0, 2.0]
        else:
            assert 0, "The action type \'" + self.action_type + "\' can not be recognized"

        # set action
        self.cart_pole_sim_model.setJointTorque(self.push_force)
        
        done = (q[0] <= -self.cart_pos_max) or (q[0] >= self.cart_pos_max) or (q[1] < -self.theta_max) or (q[1] > self.theta_max)
        done = bool(done)

        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0

        dt = 0.005
        self.v = [(self.q[0] - self.q_last[0])/dt, (self.q[1] - self.q_last[1])/dt]
        self.state = (self.q[0], self.q[1], self.v[0], self.v[1])
        self.counts += 1

        vrep_sim.simxSynchronousTrigger(self.cart_pole_sim_model.client_ID)
        vrep_sim.simxGetPingTime(self.cart_pole_sim_model.client_ID)

        return np.array(self.state), reward, done, {}
    
    def reset(self):
        # print('Reset the environment after {} counts'.format(self.counts))
        self.counts = 0
        self.push_force = 0
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
        self.steps_beyond_done = None

        vrep_sim.simxStopSimulation(self.cart_pole_sim_model.client_ID, vrep_sim.simx_opmode_blocking) # stop the simulation
        vrep_sim.simxGetPingTime(self.cart_pole_sim_model.client_ID)
        time.sleep(0.01) # ensure the coppeliasim is stopped
        vrep_sim.simxStartSimulation(self.cart_pole_sim_model.client_ID, vrep_sim.simx_opmode_oneshot)
        self.cart_pole_sim_model.setJointTorque(0)
        vrep_sim.simxSynchronousTrigger(self.cart_pole_sim_model.client_ID)
        vrep_sim.simxGetPingTime(self.cart_pole_sim_model.client_ID)

        return np.array(self.state)
    
    def render(self):
        return None

    def close(self):
        vrep_sim.simxStopSimulation(self.cart_pole_sim_model.client_ID, vrep_sim.simx_opmode_blocking) # stop the simulation
        vrep_sim.simxFinish(-1)  # Close the connection
        print('Close the environment')
        return None

if __name__ == "__main__":
    env = CartPoleEnv()
    env.reset()

    for _ in range(500):
        action = env.action_space.sample() # random action
        env.step(action)
        print(env.state)

    env.close()
