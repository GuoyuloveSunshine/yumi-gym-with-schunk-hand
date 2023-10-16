import numpy as np
import matplotlib.pyplot as plt
import h5py
import cv2
import os

folder = "schunk/woHands/"
action = "source_上楼梯-shanglouti.h5"
hf = h5py.File('./data/'+folder+action, 'r')
key = "group1"
group1 = hf.get(key)
print(group1.keys())
l_joint_q = group1.get('l_joint_q')
r_joint_q = group1.get('r_joint_q')
l_hand_pos = group1.get('l_hand_pos')
r_hand_pos = group1.get('r_hand_pos')

# print(l_hand_pos[0])
print(l_joint_q.shape)
total_frames = l_joint_q.shape[0]
# print(l_joint_q.shape, r_joint_q.shape)

folder2 = "schunk/woHands/"
action2 = "source_交通-jiaotong.h5"
hf2 = h5py.File('./data/'+folder2+action2, 'r')
key2 = "group1"
group12 = hf2.get(key)
print(group12.keys())
l_joint_q2 = group12.get('l_joint_q')
r_joint_q2 = group12.get('r_joint_q')
l_hand_pos2 = group12.get('l_hand_pos')
r_hand_pos2 = group12.get('r_hand_pos')
print(l_joint_q2.shape)
total_frames2 = l_joint_q2.shape[0]

for t in range(total_frames):
        # print(t, l_hand_pos.shape, l_hand_pos[t] * 180 / np.pi)
    print("frame: ", t)
    print(l_joint_q[t],l_joint_q2[t])
