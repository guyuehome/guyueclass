from stable_baselines import DQN
from stable_baselines.common.evaluation import evaluate_policy
import gym
import time

# env = gym.make('CartPole-v0')  # 倒立摆
env = gym.make('GridWorld-v0') # 机器人找金币
TRAIN = 0

if TRAIN:
    model = DQN('MlpPolicy', env, learning_rate=1e-3, prioritized_replay=True, verbose=1)
    model.learn(total_timesteps=int(1e5))
    # model.save("dqn_cartpole")
    model.save("dqn_gridworld")
    del model

else:
    # model = DQN.load("dqn_cartpole", env)
    model = DQN.load("dqn_gridworld", env)
    mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
    # print(mean_reward, std_reward)
    obs = env.reset()
    # print(obs)
    for i in range(1000):
        action, _states = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        env.render()
        time.sleep(2) # for showing render()