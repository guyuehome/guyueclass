from stable_baselines3 import A2C
from stable_baselines3.common.env_checker import check_env

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CallbackList
from stable_baselines3.common.callbacks import EvalCallback

import sys
sys.path.append("../utils")
from callbackFunctions import VisdomCallback

sys.path.append("../CartPole")
from CartPoleEnv import CartPoleEnv

import os


# ---------------- Create environment
env = CartPoleEnv(action_type='discrete') # action_type can be set as discrete or continuous
check_env(env)

# ---------------- Callback functions
log_dir = "../CartPole/saved_models/tmp"
os.makedirs(log_dir, exist_ok=True)

env = Monitor(env, log_dir)

callback_visdom = VisdomCallback(name='visdom_cart_pole_rl', check_freq=100, log_dir=log_dir)
callback_save_best_model = EvalCallback(env, best_model_save_path=log_dir, log_path=log_dir, eval_freq=500, deterministic=True, render=False)
callback_list = CallbackList([callback_visdom, callback_save_best_model])


# ---------------- Model
# Option 1: create a new model
# print("create a new model")
# model = A2C(policy='MlpPolicy', env=env, learning_rate=1e-3, verbose=True)

# Option 2: load the model from files (note that the loaded model can be learned again)
# print("load the model from files")
# model = SAC.load("../CartPole/saved_models/tmp/best_model", env=env)
# model.learning_rate = 1e-3

# Option 3: load the pre-trained model from files
print("load the pre-trained model from files")
if env.action_type == 'discrete':
    model = A2C.load("../CartPole/saved_models/best_model_discrete", env=env)
else:
    model = A2C.load("../CartPole/saved_models/best_model_continuous", env=env)


# ---------------- Learning
# print('Learning the model')
# model.learn(total_timesteps=400000, callback=callback_list) # 'MlpPolicy' = Actor Critic Policy
# print('Finished')
# del model # delete the model and load the best model to predict
# model = A2C.load("../CartPole/saved_models/tmp/best_model", env=env)

# ---------------- Prediction
for _ in range(10):
    observation, done = env.reset(), False
    episode_reward = 0.0

    while not done:
        action, _state = model.predict(observation, deterministic=True)
        observation, reward, done, info = env.step(action)
        episode_reward += reward
    
    print([episode_reward, env.counts])

env.close()
