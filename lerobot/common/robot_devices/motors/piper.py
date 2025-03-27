import time
from typing import Dict
from piper_sdk import *
from lerobot.common.robot_devices.motors.configs import PiperMotorsBusConfig

class PiperMotorsBus:
    """
        对Piper SDK的二次封装
    """
    def __init__(self, 
                 config: PiperMotorsBusConfig):
        self.piper = C_PiperInterface_V2(config.can_name,can_auto_init=False)
        self.piper.ConnectPort()
        self.motors = config.motors
        self.init_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # [6 joints + 1 gripper] * 0.0
        self.safe_disable_position = [0.0, 0.0, 0.0, 0.0, 0.52, 0.0, 0.0]
        # self.pose_factor = 1000 # 单位 0.001mm
        # self.joint_factor = 57324.840764 # 1000*180/3.14， rad -> 度（单位0.001度）
        self.pose_factor = 1 # 单位 0.001mm
        self.joint_factor = 1 #（单位0.001度）
        self.gripper_factor = 1
        
    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]


    def connect(self, enable:bool) -> bool:
        '''
            使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
        '''
        enable_flag = False
        loop_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            print(f"--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
            if(enable):
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0,1000,0x01, 0)
            else:
                # move to safe disconnect position
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0,1000,0x02, 0)
            print(f"使能状态: {enable_flag}")
            print(f"--------------------")
            if(enable_flag == enable):
                loop_flag = True
                enable_flag = True
            else: 
                loop_flag = False
                enable_flag = False
            # 检查是否超过超时时间
            # if elapsed_time > timeout:
            #     print(f"超时....")
            #     enable_flag = False
            #     loop_flag = True
            #     break
            time.sleep(0.5)
        resp = enable_flag
        print(f"Returning response: {resp}")
        return resp
    
    def motor_names(self):
        return

    def set_calibration(self):
        return
    
    def revert_calibration(self):
        return

    def apply_calibration(self):
        """
            移动到初始位置
        """
        self.write(target_joint=self.init_joint_position)

    def write(self, target_joint:list):
        """
            Joint control
            - target joint: in radians
                joint_1 (float): 关节1角度 (-92000~92000) / 57324.840764
                joint_2 (float): 关节2角度 -1300 ~ 90000 / 57324.840764
                joint_3 (float): 关节3角度 2400 ~ -80000 / 57324.840764
                joint_4 (float): 关节4角度 -90000~90000 / 57324.840764
                joint_5 (float): 关节5角度 19000~-77000 / 57324.840764
                joint_6 (float): 关节6角度 -90000~90000 / 57324.840764
                gripper_range: 夹爪角度 0~0.08
        """
        joint_0 = round(target_joint[0]*self.joint_factor)
        joint_1 = round(target_joint[1]*self.joint_factor)
        joint_2 = round(target_joint[2]*self.joint_factor)
        joint_3 = round(target_joint[3]*self.joint_factor)
        joint_4 = round(target_joint[4]*self.joint_factor)
        joint_5 = round(target_joint[5]*self.joint_factor)
        # gripper_range = round(target_joint[6]*1000*1000)
        gripper_range = round(target_joint[6]*self.gripper_factor)

        self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00) # joint control
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(abs(gripper_range), 1000, 0x01, 0) # 夹爪角度,单位 0.001° 夹爪力矩,单位 0.001N/m,范围0-5000,对应0-5N/m
    

    def read(self, is_follower_arm=True) -> Dict:
        """
            - 机械臂关节消息,单位0.001度
            - 机械臂夹爪消息
        """
        
        if is_follower_arm:
            joint_msg = self.piper.GetArmJointMsgs()    # 获取从动机械臂关节消息(原始消息)
            joint_state = joint_msg.joint_state

            gripper_msg = self.piper.GetArmGripperMsgs()    # 获取从动机械臂夹爪消息(原始消息)
            gripper_state = gripper_msg.gripper_state
        
        else:
            joint_msg = self.piper.GetArmJointCtrl()    # 获取主动机械臂关节消息(原始消息)
            joint_state = joint_msg.joint_ctrl

            gripper_msg = self.piper.GetArmGripperCtrl()    # 获取主动机械臂夹爪消息(原始消息)
            gripper_state = gripper_msg.gripper_ctrl
        
        return {
            "joint_1": joint_state.joint_1,
            "joint_2": joint_state.joint_2,
            "joint_3": joint_state.joint_3,
            "joint_4": joint_state.joint_4,
            "joint_5": joint_state.joint_5,
            "joint_6": joint_state.joint_6,
            "gripper": gripper_state.grippers_angle
        }
    
    def safe_disconnect(self):
        """ 
            Move to safe disconnect position
        """
        self.write(target_joint=self.safe_disable_position)