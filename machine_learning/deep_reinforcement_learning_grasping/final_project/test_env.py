from kuka_visual_reach import KukaVisualReachEnv
from stable_baselines3.common.env_checker import check_env

env=KukaVisualReachEnv(is_render=True,is_good_view=True)
assert check_env(env)==None
assert env.observation_space.shape==(4,84,84)
assert env.action_space.shape==(3,)

obs=env.reset()
for _ in range(1000):
    action=env.action_space.sample()
    state_,reward,done,info=env.step(action)
    if done:
        obs=env.reset()
