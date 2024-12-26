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

def main():
    # socket (glove)
    host, port = '0.0.0.0', 5560
    glove_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    glove_server_socket.bind((host, port))
    glove_server_socket.listen(1)
    print(f"Server listening on {host}:{port}")
    glove_server_socket.settimeout(10.0)
    glove_client_socket, client_address = glove_server_socket.accept()
    print(f"Connection from {client_address} established.")

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

    while True:
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
        # if right_hand_data[0] > 0.5:
        #     arm.set_gripper_position(0, wait=False)
        # else:
        #     arm.set_gripper_position(800, wait=False)


if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    main()