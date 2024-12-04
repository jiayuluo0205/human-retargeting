import random
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
import sys
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

cuda = torch.cuda.is_available()
device = torch.device("cuda:0" if cuda else "cpu")
def update_hand_trimesh(joint_pos, start):
    ''' gt col hand mesh '''
    # 将joint_pos转换为PyTorch张量，并增加批次维度
  

    # 使用joint_pos_full进行后续处理
    in_col, collision_result_pb = hand.get_collision_results_pb(joint_pos)
    result_dict = hand.get_hand_trimesh(joint_pos, visual=False, collision=True, collision_result_pb=collision_result_pb)
    gt_my_hand_mesh_col = result_dict["collision"]
    server.scene.add_mesh_trimesh("gt_hand_mesh_col", gt_my_hand_mesh_col)
    joint_pos_tensor = torch.tensor(joint_pos[start:].copy(), dtype=torch.float32).unsqueeze(0).to(device)  # Shape: [1, n]


    # 预测部分
    mlp.eval()
    with torch.no_grad():
        joint_pos_tensor = torch.tensor(joint_pos[start:], dtype=torch.float32).unsqueeze(0).to(device)
        joint_pos1 = mlp(joint_pos_tensor)

    if start != 0:
        # 创建一个与batch_size匹配，列数为start的全零张量
        zeros = torch.zeros(joint_pos1.size(0), start).to(device)
        # 将零张量与joint_pos_tensor在特征维度上拼接
        joint_pos_tensor = torch.cat((zeros, joint_pos1), dim=1)
    else:
        joint_pos_tensor=joint_pos1
        pass  # 如果start为0，不需要补零
    joint_pos_full = joint_pos_tensor.cpu().numpy().squeeze(0)  # Shape: [n,]
    in_col, collision_result_pb = hand.get_collision_results_pb(joint_pos_full)
    result_dict = hand.get_hand_trimesh(joint_pos_full, visual=False, collision=True, collision_result_pb=collision_result_pb)
    pred_my_hand_mesh_col = result_dict["collision"].apply_translation([0.0, 0.5, 0.0])
    server.scene.add_mesh_trimesh("pred_hand_mesh_col", pred_my_hand_mesh_col)



if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='选择生成关节值的方法。')
    parser.add_argument('--hand', type=str, choices=['leaphand', 'shadowhand','allegrohand'], default='leaphand', help='选择 hand')
    parser.add_argument('--epoch', type=int, default=200, help='Number of epochs to select uniformly if --id is set to "all"')
    
    args = parser.parse_args()

       
    if args.hand == 'leaphand':
        hand = LeapHandRight() 
        config = Leaphand_Config("whole","whole") 
        start=0
    elif args.hand == 'shadowhand':
        hand = ShadowHandRight() 
        config = Shadowhand_Config("whole","whole")
        start=6
    elif args.hand =='allegrohand':
        hand = AllegroHandLeft()
        config = Allegrohand_Config("whole","whole")
        start=6
    server = viser.ViserServer()

    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    mlp = FingerMLP(hand,hand.ndof-start,hand.ndof-start,start=start).to(device)


    # 使用glob.glob找到匹配的文件列表
    file_list = glob.glob(f"{POS2POS_TRANSLATER_DIR}/{args.hand}/generator_epoch_{args.epoch}_*.pth")

    # 检查是否找到匹配的文件
    if not file_list:
        raise FileNotFoundError("未找到匹配的模型文件。")
    elif len(file_list) > 1:
        raise ValueError("找到多个匹配的模型文件，请更具体地指定文件名。")
    else:
        # 加载模型
        file_path = file_list[0]
        mlp.load_state_dict(torch.load(file_path))     
                
    # Create joint angle sliders.
    gui_joints = []
    initial_angles = []
    for (joint_name, lower, upper) in zip(hand.actuated_joint_names_pb, hand.lower_joint_limits[0], hand.upper_joint_limits[0]):
        # 将lower和upper转换为Python的float类型
        lower = lower.cpu().numpy() if lower is not None else -np.pi
        upper = upper.cpu().numpy() if upper is not None else np.pi
        lower = float(lower)
        upper = float(upper)
        # 计算初始角度，确保initial_angle是float类型
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
        
        # 先将slider添加到列表中
        gui_joints.append(slider)

        # 设置on_update函数
        slider.on_update(
            lambda _, gui_joints=gui_joints: update_hand_trimesh(np.array([gui.value for gui in gui_joints]),start)
        )
        
        initial_angles.append(initial_angle)
    input()
