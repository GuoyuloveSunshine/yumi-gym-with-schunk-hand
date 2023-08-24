import gym, yumi_gym


# env = gym.make('yumi-inspire')
env = gym.make('yumi-schunk')
observation = env.reset()
env.render()


while True:
    env.render()

    action = env.action_space.sample()

    observation, reward, done, info = env.step(action)
