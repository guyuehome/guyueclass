from spinup import ppo_pytorch as ppo
from UR5_Controller import UR5_Controller
from spinup.utils.test_policy import load_policy_and_env, run_policy
import torch

TRAIN = 0

target = [0, -1.57, 1.57, 0, 0, 0]
env = lambda : UR5_Controller(target)

if TRAIN:
    ac_kwargs = dict(hidden_sizes=[64,64], activation=torch.nn.ReLU)
    logger_kwargs = dict(output_dir='log', exp_name='ur5_goToTarget')

    ppo(env, ac_kwargs=ac_kwargs, logger_kwargs=logger_kwargs,
        steps_per_epoch=5000, epochs=4000)

else:
    _, get_action = load_policy_and_env('log')
    env_test = UR5_Controller(target)
    run_policy(env_test, get_action)
