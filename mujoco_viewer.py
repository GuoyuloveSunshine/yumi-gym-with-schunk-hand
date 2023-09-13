import mujoco as mj
import mujoco.viewer as viewer
import numpy as np
import time


class YumiSchunk:
    def __init__(self, render = True) -> None:
        self.model = mj.MjModel.from_xml_path('yumi_gym/envs/assets/yumi_with_schunk_hands.xml')
        # self.model = mj.MjModel.from_xml_path('yumi_gym/envs/assets/yumi_with_inspire_hands.xml')
        self.data = mj.MjData(self.model)
        self.viewer = viewer.launch_passive(self.model, self.data)
        self.need_render = render
        if not self.need_render:
            self.viewer.close()

        # self.js = XboxController()

    def step(self,ctrl):
        # self.data.ctrl[:] = ctrl
        mj.mj_step(self.model, self.data)

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
        self.data.qpos[:] = joint_ang
        # print(joint_ang)
        ctrl = np.zeros(joint_ang.shape)
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
        # #### body part ####
        # for i in range(self.model.nbody):
        #     print(i)
        #     name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
        #     print(name)
        #     print("world position: ", self.data.xpos[i])
        # #### joint part ####
        # for i in range(self.model.nq):
        #     print(i)
        #     name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, i)
        #     print(name)
        #     # print("q position: ", self.data.qpos[i])

        #     # print(self.data.efc_pos)
        #     # mj.mjtConstraint.mjCNSTR_LIMIT_JOINT(self.model))
        for jj in range(1000):
            input_data = np.zeros(self.data.qpos.shape)
            if jj != 0:
                
                #### schunk hand ####
                hand_data = np.random.rand(self.data.qpos.shape[0])
                # input_data[7:27] = hand_data[7:27]
                # input_data[34:] = hand_data[34:]
                # #### inspire hand ####
                # hand_data = -np.random.rand(self.data.qpos.shape[0])*1.5
                # input_data[7:19] = hand_data[7:19]
                # input_data[26:] = hand_data[26:]
            self.direct_ja_control(input_data)
            # print(self.data.contact)
            # print(self.data.contact.geom1.shape, self.data.contact.geom1)
            couple = [(0, 1),(0, 21),(8, 18),(28, 38)]
            for j in range(self.data.contact.geom1.shape[0]):
                print("collision geom 1, body part {}: {}".format(self.data.contact.geom1[j], mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, self.data.contact.geom1[j])))
                print("collision geom 2, body part {}: {}".format(self.data.contact.geom2[j], mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, self.data.contact.geom2[j])))
            print()
            time.sleep(5)
        

    

if __name__ == "__main__":
    gen3 = YumiSchunk()
    gen3.simulate()