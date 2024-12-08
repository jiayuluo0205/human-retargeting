from __future__ import annotations

import time
from typing import Literal

import math
import numpy as np
import tyro
import torch

import viser
from viser.extras import ViserUrdf
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

@contextlib.contextmanager
def realsense_pipeline(fps: int = 60):
    """Context manager that yields a RealSense pipeline."""

    # Configure depth and color streams.
    pipeline = rs.pipeline()  # type: ignore
    config = rs.config()  # type: ignore

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)  # type: ignore
    config.resolve(pipeline_wrapper)

    config.enable_stream(rs.stream.depth, rs.format.z16, fps)  # type: ignore
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, fps)  # type: ignore

    # Start streaming.
    pipeline.start(config)

    yield pipeline

    # Close pipeline when done.
    pipeline.stop()

def start_server(host='0.0.0.0', port=5559, timeout=10.0):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Server listening on {host}:{port}")
    server_socket.settimeout(timeout)
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

def non_collide_mlp(joint_pos, start):
    ''' 更新手部模型网格 '''
    joint_pos = joint_pos.to(device) 

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
    leap_hand.set_allegro(joint_pos_full)
    return control_pos

def main():
    global hand, leap_hand, mlp
    # Start viser server.
    server = viser.ViserServer()
    # server 1
    context =zmq.Context()
    print("Connecting to windows server...")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://172.25.97.8:5555")
    # server 2
    """hand"""
    hand = LeapHandRight() 
    config = Leaphand_Config("whole","whole") 
    start = 0
    leap_hand = LeapNode()
    # server_hand = viser.ViserServer(port=8888)
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    mlp = FingerMLP(hand, hand.ndof - start, hand.ndof - start, start=start).to(device)

    file_list = glob.glob(f"{POS2POS_TRANSLATER_DIR}/leaphand/generator_epoch_200_*.pth")
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

    """hand"""

    i = 0
    arm = XArmAPI('192.168.1.208')
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    arm.set_position(x=400, y=100, z=350, roll=-180, pitch=0, yaw=0, speed=100, is_radian=False, wait=True)
    xarm_right_init_pos = [400, 100, 450]
    arm.set_mode(7)
    arm.set_state(0)
    speed = 100
    scale = 1.5


    XArm_model = RobotModel(robot_name='xarm', urdf_path='assets/robots/xarm6/xarm6_wo_ee_ori_jl.urdf', meshes_path='assets/robots/xarm6/meshes/')
    
    rw_hand = LeapNode(torque_enable=True)    
    sim_hand = LeapHandRight(load_visual_mesh=True, load_col_mesh=False, load_balls_urdf=False, load_n_collision_point=0)

    leaphand_model = RobotModel(robot_name='leaphand', urdf_path='assets/robots/leap_hand/leap_hand_right_extended.urdf', meshes_path='assets/robots/leap_hand/meshes/visual')

    X_ArmTag25_path = "/data/gjx/human-retargeting/data/transform/X_ArmTag25.npy"
    X_ArmTag25 = np.load(X_ArmTag25_path)
    X_ArmTag25[0, 3] += 0.05
    X_ArmTag25[1, 3] -= 0.02
    X_ArmTag25[2, 3] -= 0.0
    wxyz_ArmTag25 = R.from_matrix(X_ArmTag25[:3, :3]).as_quat()
    server.scene.add_frame("/tag", wxyz=wxyz_ArmTag25, position=X_ArmTag25[:3, 3])
    
    with realsense_pipeline() as pipeline:
        profile = pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        # intrinsc matrix of camera
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy
        fov_y = 2 * math.atan(cy / fy)

        detector = AprilTagDetector()
        detector.addFamily(fam="tag25h9") 
        estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(fx=fx, fy=fy, cx=cx, cy=cy, tagSize=0.0887))

        xarm6_planner_cfg = XARM6PlannerCfg(vis=False)
        xarm6_planner = XARM6Planner(xarm6_planner_cfg)
        for client_socket in start_server():
            print("connection start")
            while True:

                data = ""  # 清空data确保每次为新的数据包
                chunk = client_socket.recv(65536)
                if not chunk:
                    break
                data += chunk.decode("utf-8")

                # 匹配右手传感器值
                right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
                right_hand_data = [float(value) for value in right_hand_matches[:28]]

                """phantom arm calculation"""
                socket.send(b"request")
                # print(f"request sent")
                pose24_byte = b""
                pose24_byte = socket.recv()
                pose24_np = np.frombuffer(pose24_byte, dtype=np.float64)
                pose24_rwrist = np.reshape(pose24_np[16:],[4,4])
                EE_right_position = pose24_rwrist[:3,3]    
                EE_right_rot_np = pose24_rwrist[:3,:3]
                if i == 0:
                    EE_right_init = EE_right_position
                    i = 1
                    continue
                else:
                    EE_right_rel = EE_right_position - EE_right_init
                    EE_right_rot = EE_right_rot_np.copy()
                    EE_right_euler = R.from_matrix(EE_right_rot).as_euler('xyz', degrees=True)
                xarm_right_target_position = np.zeros(3)
                xarm_right_target_position[0] = xarm_right_init_pos[0] + scale * EE_right_rel[0]*1000
                xarm_right_target_position[1] = xarm_right_init_pos[1] + scale * EE_right_rel[1]*1000
                xarm_right_target_position[2] = xarm_right_init_pos[2] + scale * EE_right_rel[2]*1000
                X_target = np.eye(4)
                X_target[:3, 3] = np.array(xarm_right_target_position)
                X_target[:3, :3] = R.from_euler("xyz", [-EE_right_euler[0]-90, -EE_right_euler[1], EE_right_euler[2]-180], degrees=True).as_matrix()
                """phantom arm calculation"""

                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                tags = detector.detect(gray)
                if tags:
                    for tag in tags:
                        camera_tf3d = estimator.estimate(tag).inverse()
                        tag_tf3d = estimator.estimate(tag)
                        camera_wxyz = (
                            camera_tf3d.rotation().getQuaternion().W(), 
                            camera_tf3d.rotation().getQuaternion().X(), 
                            camera_tf3d.rotation().getQuaternion().Y(), 
                            camera_tf3d.rotation().getQuaternion().Z()
                        )
                        camera_position = (
                            camera_tf3d.translation().X(),
                            camera_tf3d.translation().Y(),
                            camera_tf3d.translation().Z()
                        )
                        server.scene.add_frame("/tag/camera", wxyz=camera_wxyz, position=camera_position)
                        
                        camera_xyzw = (camera_wxyz[1], camera_wxyz[2], camera_wxyz[3], camera_wxyz[0])
                        camera_rotmat = R.from_quat(camera_xyzw).as_matrix()
                        X_Tag25Camera2 = np.eye(4)
                        X_Tag25Camera2[:3, :3] = camera_rotmat
                        X_Tag25Camera2[:3, 3] = np.array(camera_position)

                        X_ArmCamera2 = X_ArmTag25 @ X_Tag25Camera2
                        
                        clients = server.get_clients()
                        for id, client in clients.items():
                            client.camera.wxyz = R.from_matrix(X_ArmCamera2[:3, :3]).as_quat()[[3, 0, 1, 2]]
                            client.camera.position = X_ArmCamera2[:3, 3]
                            client.camera.fov = fov_y
                            new_image = client.camera.get_render(height=480, width=640, transport_format="png")
                            mask = new_image[:, :, 3] != 0
                            new_image[:, :, 0] = (new_image[:, :, 0] * 0.4).astype(np.int8)
                            new_image[:, :, 1] = (new_image[:, :, 1] * 0.75).astype(np.int8)
                            new_image[:, :, 2] = (new_image[:, :, 2] * 1).astype(np.int8)
                            opacity = 0.3
                            color_image[mask] = color_image[mask] * opacity + new_image[:, :, :3][mask] * (1 - opacity)
                            color_image = cv2.resize(color_image, (1280, 960))
                            cv2.imshow('2CFuture', color_image)
                            color_image = cv2.resize(color_image, (640, 480))

                        # hand read pos
                        config = arm.get_servo_angle(is_radian=True)[1][:6]

                        # arm phantom
                        start_qpos = [math.degrees(rad) for rad in config]
                        X_target_m = X_target.copy()
                        X_target_m[:3, 3] = X_target_m[:3, 3] / 1000.0
                        _, qpos = xarm6_planner.mplib_ik(current_qpos=np.array(start_qpos), target_pose=X_target_m, return_closest=True)
                        if qpos is None:
                            continue
                        xarm_trimesh = XArm_model.get_trimesh_q(qpos)['visual'] # arm qpos
                        server.scene.add_mesh_trimesh('xarm_trimesh', xarm_trimesh)

                        root_transform = XArm_model.frame_status['eef_point'].get_matrix()[0].cpu().numpy()
                        rotation = root_transform[:3, :3]
                        translation = root_transform[:3, 3]
                        euler = R.from_matrix(rotation).as_euler('XYZ')
                        dummy_values = np.concatenate([translation, euler]) #virtual_joint_x/y/z/r/p/y

                        ordered_joint_values = link2link(right_hand_data)
                        sim_joint_values = non_collide_mlp(torch.tensor(ordered_joint_values, dtype=torch.float32), start)

                        # rw_joint_values = rw_hand.read_pos()
                        # sim_joint_values = leap_from_rw_to_sim(rw_joint_values, sim_hand.actuated_joint_names)
                        sim_dummy_joint_values = np.concatenate([dummy_values, sim_joint_values]) 

                        leaphand_trimesh = leaphand_model.get_trimesh_q(sim_dummy_joint_values)['visual'] # hand qpos
                        server.scene.add_mesh_trimesh('leaphand_trimesh', leaphand_trimesh)

                else:
                    color_image[:10, :] = [0, 0, 255]  # 红色 (BGR 格式)
                    color_image[-10:, :] = [0, 0, 255]
                    color_image[:, :10] = [0, 0, 255]
                    color_image[:, -10:] = [0, 0, 255]
                    color_image = cv2.resize(color_image, (1280, 960))
                    cv2.imshow('2CFuture', color_image)
                    color_image = cv2.resize(color_image, (640, 480))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

if __name__ == "__main__":
    tyro.cli(main)