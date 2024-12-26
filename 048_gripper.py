from __future__ import annotations

import time
from typing import Literal

import math
import numpy as np
import tyro
import torch
import pygame

import pyrender
from pathlib import Path
import contextlib
import cv2
import pyrealsense2 as rs
from robotpy_apriltag import AprilTag, AprilTagPoseEstimator, AprilTagDetector
from scipy.spatial.transform import Rotation as R
from xarm.wrapper import XArmAPI

from diff_robot_hand.hand_model import LeapHandRight
from diff_robot_hand.utils.mesh_and_urdf_utils import joint_values_order_mapping
import time  
from leaphand_rw.leaphand_rw import LeapNode, leap_from_rw_to_sim, leap_from_sim_to_rw
from loguru import logger as lgr   
from utils.robot_model import RobotModel
from utils.rotation import matrix_to_euler
import zmq
from xarm6_interface.arm_mplib import XARM6Planner, XARM6PlannerCfg
import socket
import os
from config.config import *
from config.leaphand_config import Leaphand_Config
from scripts.pos2pos.mlp import FingerMLP
import glob
import re
from diff_robot_hand import POS2POS_TRANSLATER_DIR

cuda = torch.cuda.is_available()
device = torch.device("cuda:0" if cuda else "cpu")
# joint_mapping 定义
joint_mapping = {
    "R7": {"glove_index": 7, "joint_name": "0", "scale": 25.0, "reverse": True, "bias": 0.0},
    "R6": {"glove_index": 6, "joint_name": "1", "scale": 50.0, "reverse": True, "bias": 0.0},
    "R5": {"glove_index": 5, "joint_name": "2", "scale": 53.0, "reverse": True, "bias": 0.0},
    "R4": {"glove_index": 4, "joint_name": "3", "scale": 40.0, "reverse": True, "bias": 0.0},
    "R11": {"glove_index": 11, "joint_name": "4", "scale": 500.0, "reverse": True, "bias": 0.0},
    "R10": {"glove_index": 10, "joint_name": "5", "scale": 50.0, "reverse": True, "bias": 0.0},
    "R9": {"glove_index": 9, "joint_name": "6", "scale": 53.0, "reverse": True, "bias": 0.0},
    "R8": {"glove_index": 8, "joint_name": "7", "scale": 40.0, "reverse": True, "bias": 0.0},
    "R15": {"glove_index": 15, "joint_name": "8", "scale": 35.0, "reverse": False, "bias": 0.0},
    "R14": {"glove_index": 14, "joint_name": "9", "scale": 50.0, "reverse": True, "bias": 0.0},
    "R13": {"glove_index": 13, "joint_name": "10", "scale": 53.0, "reverse": True, "bias": 0.0},
    "R12": {"glove_index": 12, "joint_name": "11", "scale": 40.0, "reverse": True, "bias": 0.0},
    "R2": {"glove_index": 2, "joint_name": "12", "scale": 30.0, "reverse": True, "bias": 0.0},
    "R20": {"glove_index": 20, "joint_name": "13", "scale": 20.0, "reverse": True, "bias": 0.0},
    "R1": {"glove_index": 1, "joint_name": "14", "scale": 32.0, "reverse": True, "bias": 0.0},
    "R0": {"glove_index": 0, "joint_name": "15", "scale": 32.0, "reverse": True, "bias": 0.0},
}

def link2link(right_hand_data):
    # 定义 normalize_value 函数来处理每个值的归一化
    def normalize_value(value, scale, bias=0.0, reverse=False):
        value -= bias
        if reverse:
            value = -value
        return value / scale

    # 创建一个空列表来存储排序后的关节值
    ordered_joint_values = []

    # 按照 joint_mapping 中的顺序遍历并进行变换
    for joint_key, joint_info in joint_mapping.items():
        glove_index = joint_info["glove_index"]
        scale = joint_info["scale"]
        bias = joint_info["bias"]
        reverse = joint_info["reverse"]
        
        # 获取相应的关节数据并进行标准化
        joint_value = right_hand_data[glove_index]  # 获取原始的关节值
        normalized_value = normalize_value(joint_value, scale, bias, reverse)
        
        # 将处理后的关节值添加到结果列表中
        ordered_joint_values.append(normalized_value)
    
    return ordered_joint_values

def main():
    global leap_hand, mlp

    # XArm
    init_flag = 0
    arm = XArmAPI('192.168.1.208')
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    xarm_right_init_pos = [400, 100, 450]
    arm.set_position(
        x=xarm_right_init_pos[0], y=xarm_right_init_pos[1], z=xarm_right_init_pos[2],
        roll=-180, pitch=0, yaw=0, speed=100, is_radian=False, wait=True
    )
    arm.set_mode(7)
    arm.set_state(0)
    scale = 1.5

    # socket (motion)
    motion_context = zmq.Context()
    print("Connecting to windows server...")
    motion_socket = motion_context.socket(zmq.REQ)
    motion_socket.connect("tcp://172.25.97.8:5555")

    # socket (glove)
    host, port = '0.0.0.0', 5559
    glove_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    glove_server_socket.bind((host, port))
    glove_server_socket.listen(1)
    print(f"Server listening on {host}:{port}")
    glove_server_socket.settimeout(10.0)
    glove_client_socket, client_address = glove_server_socket.accept()
    print(f"Connection from {client_address} established.")

    while True:
        motion_socket.send(b"request")
        pose24_byte = b""
        pose24_byte = motion_socket.recv()
        pose24_np = np.frombuffer(pose24_byte, dtype=np.float64)
        pose24_rwrist = np.reshape(pose24_np[16:],[4,4])
        EE_right_position = pose24_rwrist[:3, 3]    
        EE_right_rot_np = pose24_rwrist[:3, :3]
        if init_flag == 0:
            EE_right_pos_init = EE_right_position
            init_flag = 1
            continue
        else:
            EE_right_rel = EE_right_position - EE_right_pos_init
            EE_right_rot = EE_right_rot_np.copy()
            EE_right_euler = R.from_matrix(EE_right_rot).as_euler('xyz', degrees=True)
        xarm_right_target_position = np.zeros(3)
        xarm_right_target_position[0] = xarm_right_init_pos[0] + scale * EE_right_rel[0] * 1000
        xarm_right_target_position[1] = xarm_right_init_pos[1] + scale * EE_right_rel[1] * 1000
        xarm_right_target_position[2] = xarm_right_init_pos[2] + scale * EE_right_rel[2] * 1000
            
        # set XArm

        xyz = xarm_right_target_position
        xmin, xmax = 200, 800  # X轴的安全边界
        ymin, ymax = -300, 300  # Y轴的安全边界
        zmin, zmax = 100, 700  # Z轴的安全边界
        xyz[0] = np.clip(xyz[0], xmin, xmax)
        xyz[1] = np.clip(xyz[1], ymin, ymax)
        xyz[2] = np.clip(xyz[2], zmin, zmax)

        rpy = R.from_euler("xyz", np.array([EE_right_euler[2], -EE_right_euler[1], -EE_right_euler[0]]), degrees=True).as_euler("ZYX", degrees=True)
        rpy += [-90, 0, -180]
        arm.set_position(
                x=xyz[0], y=xyz[1], z=xyz[2], 
                roll=rpy[0], pitch=rpy[1], yaw=rpy[2], 
                speed=100, wait=False
        )
            

        # receive glove data
            # start_time = time.time()
        while True:
            data = ""
            chunk = glove_client_socket.recv(65536)
            data += chunk.decode("utf-8")

            right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
            right_hand_data = [float(value) for value in right_hand_matches[-28:]]
            if len(right_hand_data) == 28:
                break

            # set gripper
        print(right_hand_data[0])
        if right_hand_data[0] > 0.5:
            arm.set_gripper_position(0, wait=False)
        else:
            arm.set_gripper_position(800, wait=False)


if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    main()