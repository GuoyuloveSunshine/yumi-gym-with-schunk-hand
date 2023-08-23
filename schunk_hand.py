import pybullet as p
import pybullet_data #pybullet自带的一些模型
p.connect(p.GUI) #连接到仿真环境，p.DIREACT是不显示仿真界面,p.GUI则为显示仿真界面
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=180, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.1])
p.setGravity(0,0,-10) #设定重力
p.resetSimulation() #重置仿真环境
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #添加pybullet的额外数据地址，使程序可以直接调用到内部的一些模型
# LShandId = p.loadURDF('yumi_schunck_gym/assets/schunk_left_hand.urdf',useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
LShandId = p.loadURDF('yumi_schunck_gym/assets/schunk_right_hand.urdf',useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION+p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)

joints = []
joint2Index = {}
for i in range(p.getNumJoints(LShandId)):
    joints.append(p.getJointInfo(LShandId,i)[1].decode('utf-8'))
    joint2Index[p.getJointInfo(LShandId, i)[1].decode('utf-8')] = i
    # print(p.getJointInfo(LShandId,i)[1].decode('utf-8'))


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