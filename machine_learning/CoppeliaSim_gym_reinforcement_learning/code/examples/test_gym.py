import gym

env = gym.make("CartPole-v1")
observation = env.reset()

for _ in range(10000):
  env.render()
  action = env.action_space.sample() # your agent here (this takes random actions)
  observation, reward, done, info = env.step(action)

  if done:
    observation = env.reset()

env.close()
