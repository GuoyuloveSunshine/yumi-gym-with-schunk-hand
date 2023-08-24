import gym, yumi_gym
import pybullet as p
import numpy as np
import h5py
import time


hf = h5py.File('./data/inference_jiqiren.h5', 'r')
key = "group1"
# hf = h5py.File('./data/mocap_data_YuMi_affine_execute.h5', 'r')
# key = 'fengren.bag'
# hf = h5py.File('./data/mocap_data_YuMi_affine_execute.h5', 'r')
# key = 'fengren.bag'
group1 = hf.get(key)
print(group1.keys())
l_joint_angle = group1.get('l_joint_angle')
r_joint_angle = group1.get('r_joint_angle')
l_hand_angle = group1.get('l_glove_angle')
r_hand_angle = group1.get('r_glove_angle')

# hf = h5py.File('./data/mocap_ik_results_YuMi_g2o_similarity.h5', 'r')
# key = 'fengren_1'
# arm_traj = hf[key+'/arm_traj_1'][:]
# l_joint_angle = arm_traj[:, :7]
# r_joint_angle = arm_traj[:, 7:14]
# l_hand_angle = arm_traj[:, 14:26]
# r_hand_angle = arm_traj[:, 26:38]


total_frames = l_joint_angle.shape[0]
print(l_joint_angle.shape, r_joint_angle.shape)
env = gym.make('yumi-inspire')
observation = env.reset()
env.render()

flag = True

while flag:
    env.render()
    for t in range(total_frames):
        for i in range(1):
            # print(t, l_hand_angle.shape, l_hand_angle[t] * 180 / np.pi)
            print("frame: ", t)
            action = l_joint_angle[t].tolist() + r_joint_angle[t].tolist() + l_hand_angle[t].tolist() + r_hand_angle[t].tolist()
            observation, reward, done, info = env.step(action)
            time.sleep(0.02)
    flag = False
