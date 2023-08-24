import os
import gym
from gym import spaces
import pybullet as p
import numpy as np

class YumiSchunkEnv(gym.Env):
    """docstring for YumiSchunkEnv"""
    def __init__(self):
        super(YumiSchunkEnv, self).__init__()
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.1])
        self.step_counter = 0
        self.joints = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l',
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
        self.action_space = spaces.Box(np.array([-1]*len(self.joints), dtype=np.float32), np.array([1]*len(self.joints), dtype=np.float32))
        self.observation_space = spaces.Box(np.array([-1]*len(self.joints), dtype=np.float32), np.array([1]*len(self.joints), dtype=np.float32))
        print("Make environment!\n")

    def step(self, action, custom_reward=None, **kwargs):
        # print("Hello")
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        p.setJointMotorControlArray(self.yumiUid, [self.joint2Index[joint] for joint in self.joints], p.POSITION_CONTROL, action)
        p.stepSimulation()
        # get joint states
        jointStates = {}
        for joint in self.joints:
            jointStates[joint] = p.getJointState(self.yumiUid, self.joint2Index[joint]) + p.getLinkState(self.yumiUid, self.joint2Index[joint])
        # recover color
        for joint, index in self.joint2Index.items():
            if joint in self.jointColor and joint != 'world_joint':
                p.changeVisualShape(self.yumiUid, index, rgbaColor=self.jointColor[joint])
        # check collision
        collision = False
        for joint in self.joints:
            if len(p.getContactPoints(bodyA=self.yumiUid, linkIndexA=self.joint2Index[joint])) > 0:
                collision = True
                for contact in p.getContactPoints(bodyA=self.yumiUid, linkIndexA=self.joint2Index[joint]):
                    print("Collision Occurred in Joint {} & Joint {}!!!".format(contact[3], contact[4]))
                    p.changeVisualShape(self.yumiUid, contact[3], rgbaColor=[1, 0, 0, 1])
                    p.changeVisualShape(self.yumiUid, contact[4], rgbaColor=[1, 0, 0, 1])
        
        self.step_counter += 1

        if custom_reward is None:
            # default reward
            reward = 0
            done = False
        else:
            # custom reward
            reward, done = custom_reward(jointStates=jointStates, collision=collision, step_counter=self.step_counter)

        info = {'collision': collision}
        observation = [jointStates[joint][0] for joint in self.joints]
        return observation, reward, done, info

    def reset(self):
        p.resetSimulation()
        self.step_counter = 0
        self.yumiUid = p.loadURDF(os.path.join(os.path.dirname(os.path.realpath(__file__)),
            "assets/yumi_with_schunk_hands.urdf"), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
        # self.tableUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),
        #     "table/table.urdf"), basePosition=[0,0,-0.65])
        p.setGravity(0, 0, -10)
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(1./240.)
        self.joint2Index = {}  # jointIndex map to jointName
        for i in range(p.getNumJoints(self.yumiUid)):
            self.joint2Index[p.getJointInfo(self.yumiUid, i)[1].decode('utf-8')] = i
        self.jointColor = {}  # jointName map to jointColor
        for data in p.getVisualShapeData(self.yumiUid):
            self.jointColor[p.getJointInfo(self.yumiUid, data[1])[1].decode('utf-8')] = data[7]
        # recover color
        for joint, index in self.joint2Index.items():
            if joint in self.jointColor and joint != 'world_joint':
                p.changeVisualShape(self.yumiUid, index, rgbaColor=self.jointColor[joint])

    def render(self, mode='human'):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.5, 0, 0.5],
                                                          distance=.7,
                                                          yaw=90,
                                                          pitch=0,
                                                          roll=0,
                                                          upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                   aspect=float(960)/720,
                                                   nearVal=0.1,
                                                   farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                            height=720,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720, 960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        p.disconnect()
