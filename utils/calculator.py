# DefaultCoordinate; global rotation;
# x为左，y为上，z为后
# TPose为初始状态，所有旋转均是相对于TPose的旋转，以“Pelvis”为根节点
# 如果一个关节没有相对于其父关节进行旋转，q应为0
# 需要存储上一timestep其四元数信息，如果四元数未改变，则此timestep此关节旋转为0，如果改变，则此timestep此关节旋转角度为新四元数计算的q-旧四元数计算的q
# 如果一个关节有相对于其父关节的旋转，四元数代表相对于TPose一个轴和沿着这个轴旋转的角度
# 存在一个由rebocap坐标系向机器人坐标系转换的问题
# 需要设置初始状态下机器人姿势为TPose

import pybullet as p
from typing import List, Dict
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

# "joint_name_rebocap": ([joint_angle(s)], [joint_indices], [joint_bounds])

joint_memory = {
    "L_Hip": ([0.0, 0.0, 0.0], [0, 1, 2], [(-0.43, 0.43), (-0.43, 0.43), (-3.14, 2.53)]), # left_hip_yaw_joint, left_hip_row_joint, left_hip_pitch_joint
    "R_Hip": ([0.0, 0.0, 0.0], [5, 6, 7], [(-0.43, 0.43), (-0.43, 0.43), (-3.14, 2.53)]), # right_hip_yaw_joint, right_hip_row_joint, right_hip_pitch_joint
    "L_Shoulder": ([0.0, 1.55, 0.0], [11, 12, 13], [(-2.87, 2.87), (-0.34, 3.11), (-1.3, 4.45)]), # left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint
    "R_Shoulder": ([0.0, -1.55, 0.0], [15, 16, 17], [(-2.87, 2.87), (-3.11, 0.34), (-4.45, 1.3)]), # right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint
    "L_Knee": ([0.0], [3], [(-0.26, 2.05)]), # left_knee_joint
    "R_Knee": ([0.0], [8], [(-0.26, 2.05)]), # right_knee_joint
    "L_Ankle": ([0.0], [4], [(-0.87, 0.52)]), # left_ankle_joint
    "R_Ankle": ([0.0], [9], [(-0.87, 0.52)]), # right_ankle_joint
    "L_Elbow": ([1.55], [14], [(-1.25, 2.61)]), # left_elbow_joint
    "R_Elbow": ([1.55], [18], [(-1.25, 2.61)]), # right_elbow_joint
    "Spine1": ([0.0], [10], [(-2.35, 2.35)]) # torso_joint
}
# init: joint_memory should store init quaternion

def calculate_joint(robotID: int, quaternion: List[float], joint_memory: Dict, joint_name: str):
    axis, angle = p.getAxisAngleFromQuaternion(quaternion) # x:left y:up z:backward
    axis_robot = [-axis[2], axis[0], axis[1]] # x:forward y:left z:up
    quaternion_robot = p.getQuaternionFromAxisAngle(axis_robot, angle)

    def objective(angles, quaternion_ee):
        
        p.setJointMotorControlArray(robotID, joint_memory[f"{joint_name}"][1], p.POSITION_CONTROL, targetPositions=angles)

        quaternion_combined = p.getLinkState(robotID, joint_memory[f"{joint_name}"][1][-1], computeForwardKinematics=True)[5]
        
        error = np.linalg.norm(quaternion_combined - quaternion_ee) # L2

        return error

    initial_angles = joint_memory[f"{joint_name}"][0]
    result = minimize(objective, initial_angles, args=(quaternion_robot,), bounds=joint_memory[f"{joint_name}"][2])

    qs_opt = np.array(result.x) - np.array(joint_memory[f"{joint_name}"][0])

    joint_memory[f"{joint_name}"][0] = list(result.x)

    return list(qs_opt)





# for elbow, knee, ankle, torso
def calculate_normal_joint(quaternion: List[float], joint_memory: Dict, joint_name: str):
    new_axis, new_angle = p.getAxisAngleFromQuaternion(quaternion)
    old_angle = joint_memory[f'{joint_name}'][1]
    joint_memory[f'{joint_name}'][0], joint_memory[f'{joint_name}'][1] = new_axis, new_angle
    return new_angle - old_angle
    # afterwards, setJointMotor2()

# for hip, shoulder
# in h1, these joints are combined using yaw joint, roll joint, pitch joint
# given a pose of end-effector, calculate the angle of each joint
def calculate_fusion_joint(robotID, quaternion: List[float], joint_memory: Dict, joint_name: str):
    axis, angle = p.getAxisAngleFromQuaternion(quaternion) # x:left y:up z:backward
    axis_robot = [-axis[2], axis[0], axis[1]] # x:forward y:left z:up
    quaternion_robot = p.getQuaternionFromAxisAngle(axis_robot, angle)

    def objective(angles, quaternion_ee):
        q1, q2, q3 = angles

        p.setJointMotorControlArray(robotID, [11, 12, 13], p.POSITION_CONTROL, targetPositions=[q1, q2, q3])

        quaternion_combined = p.getLinkState(robotID, 13, computeForwardKinematics=True)[5]
        
        error = np.linalg.norm(quaternion_combined - quaternion_ee) # L2

        return error

    initial_angles = [joint_memory[f"{joint_name}"][0], joint_memory[f"{joint_name}"][1], joint_memory[f"{joint_name}"][2]]
    result = minimize(objective, initial_angles, args=(quaternion_robot,), bounds=[(-2.87, 2.87), (-0.34, 3.11), (-1.3, 4.45)])

    q1_opt, q2_opt, q3_opt = result.x
    q1_opt -= joint_memory[f"{joint_name}"][0]
    q2_opt -= joint_memory[f"{joint_name}"][1]
    q3_opt -= joint_memory[f"{joint_name}"][2]

    joint_memory[f"{joint_name}"][0], joint_memory[f"{joint_name}"][1], joint_memory[f"{joint_name}"][2] = result.x

    return q1_opt, q2_opt, q3_opt


# def calculate_left_shoulder(quaternion: List[float], joint_memory: Dict):
#     axis, angle = p.getAxisAngleFromQuaternion(quaternion) # x:left y:up z:backward
#     axis_robot = [-axis[2], axis[0], axis[1]] # x:forward y:left z:up
#     quaternion_robot = p.getQuaternionFromAxisAngle(axis_robot, angle)

#     def apply_joint_transform(rpy, axis, angle):
#         # 1. 应用关节的 RPY 变换（将局部坐标系对齐全局）
#         rpy_rot = Rotation.from_euler('xyz', rpy).as_matrix()
        
#         # 2. 关节的旋转矩阵（基于旋转轴）
#         joint_rot = Rotation.from_rotvec(axis * angle).as_matrix()
        
#         # 3. 在关节的局部坐标系下进行旋转
#         return np.dot(rpy_rot, joint_rot)

#     def objective(angles, quaternion_ee):
#         q1, q2, q3 = angles

#         rpy1, axis1 = [0.43633, 0, 0], np.array([0, 1, 0])  # left_shoulder_pitch_joint
#         rpy2, axis2 = [-0.43633, 0, 0], np.array([1, 0, 0])  # left_shoulder_roll_joint
#         rpy3, axis3 = [0, 0, 0], np.array([0, 0, 1])  # left_shoulder_yaw_joint

#         R1 = apply_joint_transform(rpy1, axis1, q1)
#         R2 = apply_joint_transform(rpy1+rpy2, axis2, q2)
#         R3 = apply_joint_transform(rpy1+rpy2+rpy3, axis3, q3)

#         R_combined = R3 @ R2 @ R1
#         quaternion_combined = Rotation.from_matrix(R_combined).as_quat()
        
#         error = np.linalg.norm(quaternion_combined - quaternion_ee) # L2

#         return error

#     initial_angles = [0.0, 0.0, 0.0]
#     result = minimize(objective, initial_angles, args=(quaternion_robot,), bounds=[(-2.87, 2.87), (-0.34, 3.11), (-1.3, 4.45)])
#     q1_opt, q2_opt, q3_opt = result.x

#     return q1_opt, q2_opt, q3_opt

# print(calculate_normal_joint([0, -0, -0, 1], joint_memory, 'left_elbow_joint'))