import mujoco
import mujoco.viewer
import numpy as np


class YumiSchunk:
    def __init__(self, render = True) -> None:
        self.model = mujoco.MjModel.from_xml_path('yumi_gym/envs/assets/yumi_with_schunk_hands.xml')
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.need_render = render
        if not self.need_render:
            self.viewer.close()

        # self.js = XboxController()

    def step(self,ctrl):
        self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)

    def render(self):
        if self.need_render:
            if self.viewer.is_running():
                self.viewer.sync()

    def get_current_ja_arm(self):
        current_ja = self.data.qpos[:7]
        return current_ja

    def get_current_ja_grasp(self):
        current_ja = self.data.qpos[7:]
        return current_ja

    def get_current_jv_arm(self):
        current_jv = self.data.qvel[:7]
        return current_jv

    def get_current_jv_grasp(self):
        current_jv = self.data.qvel[7:]
        return current_jv

    def joint_pos_pd_control_arm(self,joint_ang):
        kp = 5
        kd = 0.005
        current_ja = self.get_current_ja_arm()
        current_jv = self.get_current_jv_arm()
        ctrl = self.data.ctrl[:]
        ctrl[:7] = kp*(joint_ang-current_ja) - kd * current_jv
        
        self.step(ctrl)
        self.render()

    def direct_ja_control(self,joint_ang):
        self.data.qpos[:7] = joint_ang
        ctrl = np.zeros(9)
        self.step(ctrl)
        self.render()
    
    def direct_jv_control(self,joint_velo):
        self.data.qvel[:7] = joint_velo
        ctrl = np.zeros(9)
        self.step(ctrl)
        self.render()

    def ik_velo(self,target_tip_velo):
        current_q = self.get_current_ja_arm()
        jv = KinovaGen3.inverse_kinematics(current_q,target_tip_velo)
        self.direct_jv_control(jv)

    def ik_multicriteria_damped_velo(self,target_tip_velo):
        current_q = self.get_current_ja_arm()
        jv = KinovaGen3.multicriteria_ik_damped(current_q,target_tip_velo)
        self.direct_jv_control(jv)

    def grasp(self,grasp_pos,mag_force):
        current_q = self.get_current_ja_arm()
        jv = KinovaGen3.multicriteria_ik_damped(current_q,np.zeros(6))
        self.data.qvel[:7] = jv
        ctrl = np.zeros(9)
        ctrl[7] = grasp_pos
        ctrl[8] = mag_force
        self.step(ctrl)
        self.render()

    def move_after_grasp(self,target_tip_velo,grasp_pos,mag_force):
        current_q = self.get_current_ja_arm()
        jv = KinovaGen3.multicriteria_ik_damped(current_q,target_tip_velo)
        self.data.qvel[:7] = jv
        ctrl = np.zeros(9)
        ctrl[7] = grasp_pos
        ctrl[8] = mag_force
        self.step(ctrl)
        self.render()

    def js_control(self):
        pass



    def simulate(self):
        #initial state
        home_angles_arm = [0, 0.4, np.pi, -np.pi+1.4, 0, -1, np.pi/2]
        for i in range(50):
            self.direct_ja_control(home_angles_arm)
        # print("ja: ", self.get_current_ja_arm())
        print("end effector position: ", KinovaGen3.forward_kinematics(self.get_current_ja_arm()))
        # move towards hammer
        current_tip_pos = KinovaGen3.forward_kinematics(self.get_current_ja_arm())[0]
        current_tip_ang = np.array([0,0,0])
        # target_tip_pos = np.array([0.5, 0, 0.18])
        target_tip_pos = np.array([0.55,0.077,0.175])
        target_tip_ang = np.array([0,0,np.pi/2])
        distance_pos = target_tip_pos - current_tip_pos
        distance_ang = target_tip_ang - current_tip_ang
        frame = 100
        for i in range(frame):
            target_tip_v = np.concatenate([distance_pos, distance_ang])/(frame*0.002)
            self.ik_multicriteria_damped_velo(target_tip_v)
        print("end effector position: ", KinovaGen3.forward_kinematics(self.get_current_ja_arm()))
        grasp_pos = 215
        mag_force = 0
        self.model.opt.gravity[-1] = -5
        for i in range(200): 
            self.grasp(grasp_pos,mag_force)

        # raise the hammer
        current_tip_pos = KinovaGen3.forward_kinematics(self.get_current_ja_arm())[0]
        current_tip_ang = np.array([0,0,np.pi/2])
        target_tip_pos = current_tip_pos.copy()
        target_tip_pos[2] = 0.3
        target_tip_ang = np.array([0,-np.pi/2,np.pi/2])
        distance_pos = target_tip_pos - current_tip_pos
        distance_ang = target_tip_ang - current_tip_ang
        frame = 500
        for i in range(frame):
            target_tip_v = np.concatenate([distance_pos, distance_ang])/(frame*0.002)
            self.move_after_grasp(target_tip_v,grasp_pos,mag_force)
        print("end effector position: ", KinovaGen3.forward_kinematics(self.get_current_ja_arm()))
        
        # move to the nail
        current_tip_pos = KinovaGen3.forward_kinematics(self.get_current_ja_arm())[0]
        current_tip_ang = np.array([0,-np.pi/2,np.pi/2])
        target_tip_pos = np.array([0.75,-0.15,0.15])
        target_tip_ang = np.array([0,-np.pi/2,np.pi/2])
        distance_pos = target_tip_pos - current_tip_pos
        distance_ang = target_tip_ang - current_tip_ang
        frame = 500
        for i in range(frame):
            target_tip_v = np.concatenate([distance_pos, distance_ang])/(frame*0.002)
            self.move_after_grasp(target_tip_v,grasp_pos,mag_force)
            
        # print("end effector position: ", KinovaGen3.forward_kinematics(self.get_current_ja_arm()))
        # for i in range(100):
        #     self.grasp(0)

        # armq = np.copy(self.data.qpos[:7])
        # for i in range(200):
        #     target_grasp_v = 0.1
        #     self.claw_grasp(armq,target_grasp_v)
        # for i in range(200):
        #     target_grasp_v = 0.1
        #     self.claw_release(armq,target_grasp_v)

    

if __name__ == "__main__":
    gen3 = YumiSchunk()
    gen3.simulate()