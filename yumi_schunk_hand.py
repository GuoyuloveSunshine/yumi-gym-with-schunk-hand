import pybullet as p
import pybullet_data
import os

def left_schunk_hand():
    p.connect(p.GUI) #连接到仿真环境，p.DIREACT是不显示仿真界面,p.GUI则为显示仿真界面
    p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=180, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.1])
    p.setGravity(0,0,-10) #设定重力
    p.resetSimulation() #重置仿真环境
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #添加pybullet的额外数据地址，使程序可以直接调用到内部的一些模型
    LShandId = p.loadURDF(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                       "yumi_gym","envs","assets","schunk_left_hand.urdf"),
                          useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
    joints = ['LThumb_Opposition', 'LThumb_Flexion', 'LThumb_3', 'LThumb_4',
            'LIndex_Spread', 'LIndex_Proximal', 'LIndex_3', 'LIndex_4', 
            'LMiddle_Spread_Dummy', 'LMiddle_Proximal', 'LMiddle_3', 'LMiddle_4',
            'LRing_Spread', 'LRing_Proximal', 'LRing_3', 'LRing_4',
            'LPinky_Spread', 'LPinky_Proximal', 'LPinky_3', 'LPinky_4', 'LPart_Hand']
    joint2Index = {}
    for i in range(p.getNumJoints(LShandId)):
        joint2Index[p.getJointInfo(LShandId, i)[1].decode('utf-8')] = i
    print("joint2Index: ", joint2Index)


    motorsIds = []
    for joint in joints:
        motorsIds.append(p.addUserDebugParameter(joint, -1, 1, 0))


    while 1:
        action = []
        for motorId in motorsIds:
            action.append(p.readUserDebugParameter(motorId))

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        p.setJointMotorControlArray(LShandId, [joint2Index[joint] for joint in joints], p.POSITION_CONTROL, action)
        p.stepSimulation()



def right_schunk_hand():
    p.connect(p.GUI) #连接到仿真环境，p.DIREACT是不显示仿真界面,p.GUI则为显示仿真界面
    p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=180, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.1])
    p.setGravity(0,0,-10) #设定重力
    p.resetSimulation() #重置仿真环境
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #添加pybullet的额外数据地址，使程序可以直接调用到内部的一些模型
    RShandId = p.loadURDF(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                       "yumi_gym","envs","assets","schunk_right_hand.urdf"),
                          useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

    joints = ['RThumb_Opposition', 'RThumb_Flexion', 'RThumb_3', 'RThumb_4',
            'RIndex_Spread', 'RIndex_Proximal', 'RIndex_3', 'RIndex_4', 
            'RMiddle_Spread_Dummy', 'RMiddle_Proximal', 'RMiddle_3', 'RMiddle_4',
            'RRing_Spread', 'RRing_Proximal', 'RRing_3', 'RRing_4',
            'RPinky_Spread', 'RPinky_Proximal', 'RPinky_3', 'RPinky_4', 'RPart_Hand']
    joint2Index = {}
    for i in range(p.getNumJoints(RShandId)):
        joint2Index[p.getJointInfo(RShandId, i)[1].decode('utf-8')] = i
    print("joint2Index: ", joint2Index)


    motorsIds = []
    for joint in joints:
        motorsIds.append(p.addUserDebugParameter(joint, -1, 1, 0))


    while 1:
        action = []
        for motorId in motorsIds:
            action.append(p.readUserDebugParameter(motorId))

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        p.setJointMotorControlArray(RShandId, [joint2Index[joint] for joint in joints], p.POSITION_CONTROL, action)
        p.stepSimulation()


def yumi_arm():
    p.connect(p.GUI) #连接到仿真环境，p.DIREACT是不显示仿真界面,p.GUI则为显示仿真界面
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.1])
    p.setGravity(0,0,-10) #设定重力
    p.resetSimulation() #重置仿真环境
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #添加pybullet的额外数据地址，使程序可以直接调用到内部的一些模型
    LShandId = p.loadURDF(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                       "yumi_gym","envs","assets","yumi_arm.urdf"),
                          useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

    joints = ['yumi_joint_1_l',
                'yumi_joint_2_l',
                'yumi_joint_7_l',
                'yumi_joint_3_l',
                'yumi_joint_4_l',
                'yumi_joint_5_l',
                'yumi_joint_6_l',
                'yumi_joint_1_r',
                'yumi_joint_2_r',
                'yumi_joint_7_r',
                'yumi_joint_3_r',
                'yumi_joint_4_r',
                'yumi_joint_5_r',
                'yumi_joint_6_r']
    joint2Index = {}
    joint2Limits = {}
    for i in range(p.getNumJoints(LShandId)):
        joint2Index[p.getJointInfo(LShandId, i)[1].decode('utf-8')] = i
        joint2Limits[p.getJointInfo(LShandId, i)[1].decode('utf-8')] = [float(p.getJointInfo(LShandId, i)[8]), float(p.getJointInfo(LShandId, i)[9])]
    print("joint2Index: ", joint2Index)
    print("joint2Limit: ", joint2Limits)


    motorsIds = []
    for joint in joints:
        motorsIds.append(p.addUserDebugParameter(joint, joint2Limits[joint][0], joint2Limits[joint][1], 0))


    while 1:
        action = []
        for motorId in motorsIds:
            action.append(p.readUserDebugParameter(motorId))

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        p.setJointMotorControlArray(LShandId, [joint2Index[joint] for joint in joints], p.POSITION_CONTROL, action)
        p.stepSimulation()

def yumi_with_schunk_hands():
    p.connect(p.GUI) #连接到仿真环境，p.DIREACT是不显示仿真界面,p.GUI则为显示仿真界面
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.1])
    p.setGravity(0,0,-10) #设定重力
    p.resetSimulation() #重置仿真环境
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #添加pybullet的额外数据地址，使程序可以直接调用到内部的一些模型
    LShandId = p.loadURDF(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                       "yumi_gym","envs","assets","yumi_with_schunk_hands.urdf"),
                          useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

    joints = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l',
              'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l',
              'LThumb_Opposition', 'LThumb_Flexion', 'LThumb_3', 'LThumb_4',
              'LIndex_Spread', 'LIndex_Proximal', 'LIndex_3', 'LIndex_4', 
              'LMiddle_Spread_Dummy', 'LMiddle_Proximal', 'LMiddle_3', 'LMiddle_4',
              'LRing_Spread', 'LRing_Proximal', 'LRing_3', 'LRing_4',
              'LPinky_Spread', 'LPinky_Proximal', 'LPinky_3', 'LPinky_4', 'LPart_Hand',

              'yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r',
              'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r',
              'RThumb_Opposition', 'RThumb_Flexion', 'RThumb_3', 'RThumb_4',
              'RIndex_Spread', 'RIndex_Proximal', 'RIndex_3', 'RIndex_4', 
              'RMiddle_Spread_Dummy', 'RMiddle_Proximal', 'RMiddle_3', 'RMiddle_4',
              'RRing_Spread', 'RRing_Proximal', 'RRing_3', 'RRing_4',
              'RPinky_Spread', 'RPinky_Proximal', 'RPinky_3', 'RPinky_4', 'RPart_Hand']
    
    joint2Index = {}
    joint2Limits = {}
    for i in range(p.getNumJoints(LShandId)):
        joint2Index[p.getJointInfo(LShandId, i)[1].decode('utf-8')] = i
        joint2Limits[p.getJointInfo(LShandId, i)[1].decode('utf-8')] = [float(p.getJointInfo(LShandId, i)[8]), float(p.getJointInfo(LShandId, i)[9])]
    print("joint2Index: ", joint2Index)


    motorsIds = []
    for joint in joints:
        # motorsIds.append(p.addUserDebugParameter(joint, -1, 1, 0))
        motorsIds.append(p.addUserDebugParameter(joint, joint2Limits[joint][0], joint2Limits[joint][1], 0))


    while 1:
        action = []
        for motorId in motorsIds:
            action.append(p.readUserDebugParameter(motorId))

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        p.setJointMotorControlArray(LShandId, [joint2Index[joint] for joint in joints], p.POSITION_CONTROL, action)
        p.stepSimulation()

if __name__ =="__main__":
    # right_schunk_hand()
    # left_schunk_hand()
    # yumi_arm()
    yumi_with_schunk_hands()