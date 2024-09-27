from stable_baselines3.common.env_util import make_atari_env
from stable_baselines3.common.vec_env import VecFrameStack
from stable_baselines3 import PPO
from easy_logx.easy_logx import EasyLog
import os
import time
import gym

from typing import Any, List, Tuple
from easy_logx.easy_logx import EasyLog
import pybullet as p
import pybullet_data
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
from math import sqrt
import random
import time
import math
import cv2
import torch
import os
from stable_baselines3.common.vec_env import VecVideoRecorder
from stable_baselines3.common.vec_env.vec_monitor import VecMonitor
import matplotlib.pyplot as plt
from stable_baselines3.common import results_plotter
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
import pathlib
import torch as th
import torch.nn as nn
from typing import Tuple, Any, Dict, List, Type, Optional, Union, Callable
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticPolicy
import imageio
from kuka_visual_reach import KukaVisualReachEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv,SubprocVecEnv,VecEnv
import logging


# change log
# skip=2


class CustomCNN(BaseFeaturesExtractor):
    """
    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256):
        super(CustomCNN, self).__init__(observation_space, features_dim)
        # We assume CxHxW images (channels first)
        # Re-ordering will be done by pre-preprocessing or wrapper
        n_input_channels = observation_space.shape[0]
        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),

            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            n_flatten = self.cnn(
                th.as_tensor(observation_space.sample()[None]).float()
            ).shape[1]

        self.linear = nn.Sequential(nn.Linear(n_flatten, features_dim), nn.ReLU())

    def forward(self, observations: th.Tensor) -> th.Tensor:
        return self.linear(self.cnn(observations))


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """

    def __init__(self, check_freq: int, log_dir: str, verbose=1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:

            # Retrieve training reward
            x, y = ts2xy(load_results(self.log_dir), 'timesteps')
            if len(x) > 0:
                # Mean training reward over the last 100 episodes
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    print("Num timesteps: {}".format(self.num_timesteps))
                    print(
                        "Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(self.best_mean_reward,
                                                                                                 mean_reward))

                # New best model, you could save the agent here
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    # Example for saving best model
                    if self.verbose > 0:
                        print("Saving new best model to {}".format(self.save_path))
                    self.model.save(self.save_path)

        return True


def make_env(seed):
    def _init():
        env = KukaVisualReachEnv(is_render=False,skip=2)
        env.seed(seed)
        return env

    return _init



if __name__ == '__main__':

    logger = EasyLog(log_level=logging.INFO)
    logger.add_filehandler()

    timesteps = 1e7

    current_file_name = pathlib.Path(__file__).stem
    # Create log dir
    log_dir = "models/" + current_file_name
    os.makedirs(log_dir, exist_ok=True)

    env=SubprocVecEnv([make_env(None) for _ in range(12)])
    env=VecMonitor(env,log_dir)

    policy_kwargs = dict(features_extractor_class=CustomCNN,
                         features_extractor_kwargs=dict(features_dim=512),
                         net_arch=[125],
                         activation_fn=th.nn.ReLU)

    callback = SaveOnBestTrainingRewardCallback(check_freq=100, log_dir=log_dir)

    start_time=time.time()

    model = PPO('CnnPolicy', env, verbose=1, batch_size=512, policy_kwargs=policy_kwargs)
    model.learn(int(timesteps),callback=callback)

    elapsed_time=(time.time()-start_time)/3600
    logger.info(f'elapsed time={elapsed_time}')
