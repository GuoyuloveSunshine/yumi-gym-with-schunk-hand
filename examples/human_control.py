import gym, yumi_gym
import pybullet as p

# env = gym.make('yumi-inspire's)
env = gym.make('yumi-schunk')
observation = env.reset()
env.render()


motorsIds = []
for joint in env.joints:
    motorsIds.append(p.addUserDebugParameter(joint, env.joint2Limits[joint][0], env.joint2Limits[joint][1], 0))
    # motorsIds.append(p.addUserDebugParameter(joint, -1, 1, 0))

while True:
    env.render()

    action = []
    for motorId in motorsIds:
        action.append(p.readUserDebugParameter(motorId))
    
    observation, reward, done, info = env.step(action)
