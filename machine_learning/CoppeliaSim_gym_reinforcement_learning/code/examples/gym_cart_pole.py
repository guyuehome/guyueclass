import time
import gym
from stable_baselines3 import A2C

import os
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CallbackList
from stable_baselines3.common.callbacks import EvalCallback

import sys
sys.path.append("../utils")
from callbackFunctions import VisdomCallback

# ---------------- create environment
env = gym.make('CartPole-v1')

# ---------------- callback functions
log_dir = "../CartPole/saved_models/tmp"
os.makedirs(log_dir, exist_ok=True)
env = Monitor(env, log_dir)

callback_visdom = VisdomCallback(name='visdom_simple_cart_pole_rl', check_freq=100, log_dir=log_dir)
callback_save_best_model = EvalCallback(env, best_model_save_path=log_dir, log_path=log_dir, eval_freq=500, deterministic=True, render=False)
callback_list = CallbackList([callback_visdom, callback_save_best_model])


# ---------------- model learning
print('Learning the model')
model = A2C('MlpPolicy', env, verbose=True)
model.learn(total_timesteps=20000, callback=callback_list) # 'MlpPolicy' = Actor Critic Policy
print('Learning finished')

del model


# ---------------- prediction
print('Prediction')
model = A2C.load("../CartPole/saved_models/tmp/best_model", env=env)


observation = env.reset()
for i in range(1000):
    action, _state = model.predict(observation, deterministic=True)
    observation, reward, done, info = env.step(action)
    env.render()
    if done:
        observation = env.reset()

    time.sleep(0.01)

env.close()
