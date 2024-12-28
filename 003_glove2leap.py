import random
import socket
import time
import re
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.backends.cudnn as cudnn
import torch.optim as optim
import torch.utils.data

import os
from tqdm import tqdm
import numpy as np
import argparse

import viser 
import sys
sys.path.append('../3rdparty/differentiable_robot_hand')
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
from config.config import *
from config.leaphand_config import Leaphand_Config
from config.shadowhand_config import Shadowhand_Config
from config.allegrohand_config import Allegrohand_Config

from scripts.utils.downscaling_utils import *
from diff_robot_hand.hand_model import *
from diff_robot_hand.neural.colnet import LinkPosToLinkColSiren
from scripts.pos2pos.mlp import FingerMLP
from utils import *
from diff_robot_hand import POS2POS_TRANSLATER_DIR
import glob

from leaphand_rw.leaphand_rw import LeapNode

cuda = torch.cuda.is_available()
device = torch.device("cuda:0" if cuda else "cpu")

# 创建 socket 服务器
def start_server(host='0.0.0.0', port=5561):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Server listening on {host}:{port}")
    
    try:
        while True:
            global client_address
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address} established.")
            yield client_socket  # 生成客户端socket以供后续使用
    except KeyboardInterrupt:
        print("Server shutting down.")
    finally:
        server_socket.close()

def update_hand_trimesh(joint_pos, start):
    ''' 更新手部模型网格 '''
    joint_pos = joint_pos.to(device) 
    in_col, collision_result_pb = hand.get_collision_results_pb(joint_pos)
    result_dict = hand.get_hand_trimesh(joint_pos, visual=False, collision=True, collision_result_pb=collision_result_pb)
    gt_my_hand_mesh_col = result_dict["collision"]
    server.scene.add_mesh_trimesh("gt_hand_mesh_col", gt_my_hand_mesh_col)

    joint_pos_tensor = torch.tensor(joint_pos[start:], dtype=torch.float32).unsqueeze(0).to(device)  # Shape: [1, n]
    
    # 预测部分
    mlp.eval()
    with torch.no_grad():
        joint_pos1 = mlp(joint_pos_tensor)

    if start != 0:
        zeros = torch.zeros(joint_pos1.size(0), start).to(device)
        joint_pos_tensor = torch.cat((zeros, joint_pos1), dim=1)
    else:
        joint_pos_tensor = joint_pos1

    joint_pos_full = joint_pos_tensor.cpu().numpy().squeeze(0)  # Shape: (16,)
    # print(joint_pos_full)
    control_pos = [
        joint_pos_full[1], joint_pos_full[0],  
        joint_pos_full[2], joint_pos_full[3],
        joint_pos_full[5], joint_pos_full[4],  
        joint_pos_full[6], joint_pos_full[7],
        joint_pos_full[9], joint_pos_full[8],  
        joint_pos_full[10], joint_pos_full[11],
        joint_pos_full[12], joint_pos_full[13],
        joint_pos_full[14], joint_pos_full[15]
    ]

    leap_hand.set_allegro(control_pos)
    in_col, collision_result_pb = hand.get_collision_results_pb(joint_pos_full)
    result_dict = hand.get_hand_trimesh(joint_pos_full, visual=False, collision=True, collision_result_pb=collision_result_pb)
    pred_my_hand_mesh_col = result_dict["collision"].apply_translation([0.0, 0.5, 0.0])
    server.scene.add_mesh_trimesh("pred_hand_mesh_col", pred_my_hand_mesh_col)

def normalize_value(value, scale, bias=0.0, reverse=False, softplus=False, softplus_offset=0.0):
    value += bias
    if reverse:
        value = -value
    if softplus: # softplus 函数
        if not reverse:
            value = -value
        value = np.log(1 + np.exp(value))
    value /= scale
    if softplus:
        if not reverse:
            value = -value
        value += softplus_offset
    return value

# joint_mapping 定义
joint_mapping = {
    "R0": {"glove_index": 0, "joint_name": "15", "scale": 16.0, "reverse": True, "bias": 35.0, "softplus": True},
    "R1": {"glove_index": 0, "joint_name": "14", "scale": 16.0, "reverse": True, "bias": 35.0, "softplus": True},
    "R2": {"glove_index": 1, "joint_name": "12", "scale": 38.0, "reverse": True, "bias": 0.0},
    "R20": {"glove_index": 2, "joint_name": "13", "scale": 19.0, "reverse": False, "bias": 35.0, "softplus": True, "softplus_offset": 1.57},
    "R4": {"glove_index": 4, "joint_name": "3", "scale": 40.0, "reverse": True, "bias": 0.0},
    "R5": {"glove_index": 5, "joint_name": "2", "scale": 53.0, "reverse": True, "bias": 0.0},
    "R6": {"glove_index": 6, "joint_name": "1", "scale": 50.0, "reverse": True, "bias": 0.0},
    "R7": {"glove_index": 7, "joint_name": "0", "scale": 25.0, "reverse": True, "bias": 0.0},
    "R8": {"glove_index": 8, "joint_name": "7", "scale": 40.0, "reverse": True, "bias": 0.0},
    "R9": {"glove_index": 9, "joint_name": "6", "scale": 53.0, "reverse": True, "bias": 0.0},
    "R10": {"glove_index": 10, "joint_name": "5", "scale": 50.0, "reverse": True, "bias": 0.0},
    "R11": {"glove_index": 11, "joint_name": "4", "scale": 500.0, "reverse": True, "bias": 0.0},
    "R12": {"glove_index": 12, "joint_name": "11", "scale": 40.0, "reverse": True, "bias": 0.0},
    "R13": {"glove_index": 13, "joint_name": "10", "scale": 53.0, "reverse": True, "bias": 0.0},
    "R14": {"glove_index": 14, "joint_name": "9", "scale": 50.0, "reverse": True, "bias": 0.0},
    "R15": {"glove_index": 15, "joint_name": "8", "scale": 35.0, "reverse": False, "bias": 0.0},
}

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='选择生成关节值的方法。')
    parser.add_argument('--hand', type=str, choices=['leaphand', 'shadowhand','allegrohand'], default='leaphand', help='选择 hand')
    parser.add_argument('--epoch', type=int, default=200, help='Number of epochs to select uniformly if --id is set to "all"')
    
    args = parser.parse_args()

    if args.hand == 'leaphand':
        hand = LeapHandRight() 
        config = Leaphand_Config("whole","whole") 
        start = 0
    elif args.hand == 'shadowhand':
        hand = ShadowHandRight() 
        config = Shadowhand_Config("whole","whole")
        start = 6
    elif args.hand == 'allegrohand':
        hand = AllegroHandLeft()
        config = Allegrohand_Config("whole","whole")
        start = 6

    leap_hand = LeapNode()

    server = viser.ViserServer()
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    mlp = FingerMLP(hand, hand.ndof - start, hand.ndof - start, start=start).to(device)

    file_list = glob.glob(f"{POS2POS_TRANSLATER_DIR}/{args.hand}/generator_epoch_{args.epoch}_*.pth")
    if not file_list:
        raise FileNotFoundError("未找到匹配的模型文件。")
    elif len(file_list) > 1:
        raise ValueError("找到多个匹配的模型文件，请更具体地指定文件名。")
    else:
        file_path = file_list[0]
        mlp.load_state_dict(torch.load(file_path))     
                
    # Create joint angle sliders.
    gui_joints = {}
    initial_angles = []
    for (joint_name, lower, upper) in zip(hand.actuated_joint_names_pb, hand.lower_joint_limits[0], hand.upper_joint_limits[0]):
        lower = lower.cpu().numpy() if lower is not None else -np.pi
        upper = upper.cpu().numpy() if upper is not None else np.pi
        lower = float(lower)
        upper = float(upper)

        if lower <= 0 and upper >= 0:
            initial_angle = 0.0
        else:
            initial_angle = (lower + upper) / 2.0
        initial_angle = float(initial_angle)

        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=1e-3,
            initial_value=initial_angle,
        )
        
        gui_joints[joint_name] = slider  # 存储滑条
        initial_angles.append(initial_angle)

    # 启动服务器并接收数据
    for client_socket in start_server():
        try:
            while True:
                data = ""
                chunk = client_socket.recv(65536)
                data += chunk.decode("utf-8")

                right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
                # print(right_hand_matches)
                # print("***", len(right_hand_matches))
                right_hand_data = [float(value) for value in right_hand_matches[-28:]]
                if len(right_hand_data) != 28:
                    continue

                # # 匹配右手传感器值
                # right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
                # right_hand_data = [float(value) for value in right_hand_matches[:28]]
                
                # 更新 Viser 中的滑条
                print_info = ""
                for mapping in joint_mapping.values():
                    glove_index = mapping["glove_index"]
                    # print(right_hand_data)
                    glove_value = right_hand_data[glove_index]
                    scale = mapping["scale"]
                    joint_name = mapping["joint_name"]

                    if joint_name in gui_joints:
                        slider = gui_joints[joint_name]
                        normalized_value = normalize_value(
                            glove_value,
                            scale,
                            bias=mapping.get("bias", 0.0),
                            reverse=mapping.get("reverse", False),
                            softplus=mapping.get("softplus", False),
                            softplus_offset=mapping.get("softplus_offset", 0.0)
                        )
                        slider.value = normalized_value

                    if joint_name in ["12", "13", "14", "15"]:
                        print_info += f"{joint_name}: {normalized_value:.2f} "

                print(print_info)

                ordered_joint_values = [slider.value for slider in gui_joints.values()]
                
                update_hand_trimesh(torch.tensor(ordered_joint_values, dtype=torch.float32), start)

        except ConnectionResetError:
            print("Connection closed by the client.")
        
        client_socket.close()
        print(f"Connection from {client_address} closed.\n")
    
    input()