import numpy as np
import matplotlib.pyplot as plt
import h5py
import cv2
import os

def draw_human_3d(l_joint_pos, r_joint_pos, l_hand_pos, r_hand_pos, save_path):
    pair_arm = np.array([[0, 1],
                         [1, 2]])
    pair_hand = np.array([[0, 1, 2, 3, 0, 5, 6, 7, 0, 9 , 10, 11, 0 , 13, 14, 15, 0 , 17, 18, 19],
                          [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]])
    pair_arm = pair_arm.transpose(1,0)
    pair_hand = pair_hand.transpose(1, 0)
    num_arm_joint = l_joint_pos.shape[0]
    num_hand_joint = l_hand_pos.shape[0]

    pair_total = np.concatenate((pair_arm, pair_arm+num_arm_joint,pair_hand+2*num_arm_joint, pair_hand+2*num_arm_joint+num_hand_joint), axis=0)

    # print(pair_total)
    # print(l_joint_pos)
    # print(r_joint_pos)
    l_hand_pos = l_hand_pos + l_joint_pos[2]
    r_hand_pos = r_hand_pos + r_joint_pos[2]

    fig = plt.figure()
    ax = plt.axes(projection = "3d")
    
    X = np.concatenate((l_joint_pos[... , 0], r_joint_pos[..., 0], l_hand_pos[..., 0], r_hand_pos[..., 0]), axis = 0)
    Y = np.concatenate((l_joint_pos[..., 1], r_joint_pos[... , 1], l_hand_pos[... , 1], r_hand_pos[... , 1]), axis = 0)
    Z = np.concatenate((l_joint_pos[..., 2], r_joint_pos[... , 2], l_hand_pos[... , 2], r_hand_pos[... , 2]), axis = 0)
    ax.scatter3D(X,Y,Z,c="red")
    for pair in pair_total:
        xPair = [X[pair[0]], X[pair[1]]]
        yPair = [Y[pair[0]], Y[pair[1]]]
        zPair = [Z[pair[0]], Z[pair[1]]]

        ax.plot(xPair, yPair, zPair, color = 'green', linewidth = 5)
    ax.set_xlim([0,0.5])
    ax.set_ylim([-0.3,0.3])
    ax.set_zlim([0.5,1])
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.view_init(30,0,0)
    if save_p:
        plt.savefig(save_path)
    # plt.show()
    # fig.canvas.draw()
    # img = np.fromstring(fig.canvas.tostring_rgb(), dtype = np.uint8, sep="")
    # return img


# action = "source_吃饭-chifan.h5"
# action = "source_睡觉-shuijiao.h5"
folder = "schunk/woHands/"
action = "source_交通-jiaotong.h5"
hf = h5py.File('./data/'+folder+action, 'r')
key = "group1"
group1 = hf.get(key)
print(group1.keys())
l_joint_pos = group1.get('l_joint_pos')
r_joint_pos = group1.get('r_joint_pos')
l_hand_pos = group1.get('l_hand_pos')
r_hand_pos = group1.get('r_hand_pos')

# print(l_hand_pos[0])
print(l_joint_pos.shape)
total_frames = l_joint_pos.shape[0]
# print(l_joint_pos.shape, r_joint_pos.shape)


flag = True

folder = "results"
stock_action = action.split(".")[0]
stock_action = stock_action.split("-")[1]
os.makedirs(folder, exist_ok = True)
os.makedirs(os.path.join(folder, stock_action), exist_ok = True)
while flag:
    for t in range(total_frames):
            # print(t, l_hand_pos.shape, l_hand_pos[t] * 180 / np.pi)
        print("frame: ", t)
        save_p = os.path.join(folder,stock_action,str(t)+".png")
        draw_human_3d(l_joint_pos[t], r_joint_pos[t], l_hand_pos[t], r_hand_pos[t], save_p)
        # cv2.imshow("human", fig)
        # cv2.waitKey(0)
    flag = False
