import os
import gym
from gym import spaces
import pybullet as p
import numpy as np

class YumiInspireEnv(gym.Env):
    """docstring for YumiInspireEnv"""
    def __init__(self):
        super(YumiInspireEnv, self).__init__()
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.1])
        self.step_counter = 0
        self.joints = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l',
                        'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l',
                        "link1", "link11", "link2", "link22", "link3", "link33",
                        "link4", "link44", "link5", "link51", "link52", "link53",

                        'yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r',
                        'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r',
                        "Link1", "Link11", "Link2", "Link22", "Link3", "Link33",
                        "Link4", "Link44", "Link5", "Link51", "Link52", "Link53"]
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
            "assets/yumi_with_inspire_hands.urdf"), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
        p.setGravity(0, 0, -10)
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setTimeStep(1./240.)

        self.joint2Index = {}  # jointIndex map to jointName
        self.joint2Limits = {} # jointLimits map to jointName
        for i in range(p.getNumJoints(self.yumiUid)):
            self.joint2Index[p.getJointInfo(self.yumiUid, i)[1].decode('utf-8')] = i
            self.joint2Limits[p.getJointInfo(self.yumiUid, i)[1].decode('utf-8')] = [float(p.getJointInfo(self.yumiUid, i)[8]), float(p.getJointInfo(self.yumiUid, i)[9])]
        self.jointColor = {}  # jointName map to jointColor
        for data in p.getVisualShapeData(self.yumiUid):
            self.jointColor[p.getJointInfo(self.yumiUid, data[1])[1].decode('utf-8')] = data[7]

        self.joints_limit_lower = np.array([self.joint2Limits[joint][0] for joint in self.joints],  dtype=np.float32)
        self.joints_limit_upper = np.array([self.joint2Limits[joint][1] for joint in self.joints],  dtype=np.float32)
        # print(self.joints_limit_lower)
        # print(self.joints_limit_upper)
        self.action_space = spaces.Box(self.joints_limit_lower, self.joints_limit_upper)
        self.observation_space = spaces.Box(self.joints_limit_lower, self.joints_limit_upper)
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
