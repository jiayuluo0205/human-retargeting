import time
import numpy as np
import torch 
from xarm6_interface import XARM6_IP
from xarm.wrapper import XArmAPI
from xarm6_interface.arm_mplib import min_jerk_interpolator_with_alpha 

class XArm6RealWorld:
    def __init__(self, ip=XARM6_IP,is_radian=True, default_speed=0.5):
        self.arm = XArmAPI(ip, is_radian=is_radian)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_speed(5000)

        # self.arm.reset(wait=True)
        self.default_is_radian = is_radian
        self.default_speed = default_speed
        self.default_cmd_timestep = 1.0 / 500.0
        self.default_joint_values = np.array(
            [0, -67, -40, 0, 65, 0]
        ) * np.pi / 180
        self.set_joint_values(self.default_joint_values, speed=default_speed, is_radian=True, wait=True)

    def set_joint_values(self, joint_values, speed=None, is_radian=None, wait=True):
        if speed is None:
            speed = self.default_speed
        if is_radian is None:
            is_radian = self.default_is_radian
        self.arm.set_servo_angle(angle=self.to_list(joint_values), speed=speed, wait=wait, is_radian=is_radian)

    def set_joint_values_sequence(self, way_point_positions, planning_timestep=0.05, cmd_timestep=None, speed=None, is_radian=None):
        if speed is None:
            speed = self.default_speed
        if is_radian is None:
            is_radian = self.default_is_radian
        if cmd_timestep is None:
            cmd_timestep = self.default_cmd_timestep
        self.arm.set_mode(6)
        self.arm.set_state(0)
        
        joint_values_seq = min_jerk_interpolator_with_alpha(
            way_point_positions, planning_timestep, cmd_timestep
        )
        for joint_values in joint_values_seq:
            self.arm.set_servo_angle(angle=self.to_list(joint_values), speed=speed, is_radian=is_radian)
            time.sleep(1.5*cmd_timestep)  # 1.5 is a magic number
        self.arm.set_servo_angle(angle=self.to_list(joint_values_seq[-1]), speed=speed, is_radian=is_radian)
        time.sleep(2)  # 1.5 is a magic number
                
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def get_joint_values(self, is_radian=None):
        if is_radian is None:
            is_radian = self.default_is_radian
        state, joint_values = self.arm.get_servo_angle(is_radian=is_radian)
        if state != 0:
            raise ValueError("Failed to get joint values")
        return joint_values[:6]

    def to_list(self, joint_values):
        if isinstance(joint_values, list):
            return joint_values
        if isinstance(joint_values, np.ndarray):
            return joint_values.tolist()
        elif isinstance(joint_values, torch.Tensor):
            return joint_values.flatten().cpu().detach().numpy().tolist()

    def close(self):
        self.arm.disconnect()
        self.arm.reset(wait=True)

if __name__ == "__main__":
    from loguru import logger as lgr
    xarm = XArm6RealWorld()
    lgr.info("Current joint values: {}".format(xarm.get_joint_values()))

    xarm.close()