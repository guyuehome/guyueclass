from spinup import ppo_pytorch as ppo
from spinup.utils.test_policy import load_policy_and_env, run_policy
import torch
import gym

TRAIN = 0

# env = lambda : gym.make('Reacher-v2')
# env = lambda : gym.make('Ant-v3')
env = lambda : gym.make('Thrower-v2')


if TRAIN:
    ac_kwargs = dict(hidden_sizes=[64,64], activation=torch.nn.ReLU)
    # logger_kwargs = dict(output_dir='log', exp_name='reacher')
    # logger_kwargs = dict(output_dir='log2', exp_name='ant')
    logger_kwargs = dict(output_dir='log3', exp_name='hand')

    ppo(env, ac_kwargs=ac_kwargs, logger_kwargs=logger_kwargs,
        steps_per_epoch=5000, epochs=4000)

else:
    # _, get_action = load_policy_and_env('log')
    # _, get_action = load_policy_and_env('log2')
    _, get_action = load_policy_and_env('log3')
    # env_test = gym.make('Reacher-v2')
    # env_test = gym.make('Ant-v3')
    env_test = gym.make('Thrower-v2')
    run_policy(env_test, get_action)
