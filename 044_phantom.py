from __future__ import annotations

import time
from typing import Literal

import math
import numpy as np
import tyro
import torch
import pygame
import trimesh
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

@contextlib.contextmanager
def realsense_pipeline(fps: int = 60):
    """Context manager that yields a RealSense pipeline."""

    # Configure depth and color streams.
    pipeline = rs.pipeline()  # type: ignore
    config = rs.config()  # type: ignore
    # serial_number = '233622079809'
    # config.enable_device(serial_number)

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)  # type: ignore
    config.resolve(pipeline_wrapper)

    config.enable_stream(rs.stream.depth, rs.format.z16, fps)  # type: ignore
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, fps)  # type: ignore

    # Start streaming.
    pipeline.start(config)

    yield pipeline

    # Close pipeline when done.
    pipeline.stop()


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


def non_collide_mlp(joint_pos):
    ''' 更新手部模型网格 '''
    joint_pos = joint_pos.to(device) 
    joint_pos_tensor = torch.tensor(joint_pos, dtype=torch.float32).unsqueeze(0).to(device)  # Shape: [1, n]
    
    # 预测部分
    mlp.eval()
    with torch.no_grad():
        joint_pos_tensor = mlp(joint_pos_tensor)

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
    return joint_pos_full, control_pos


def main():
    global leap_hand, mlp
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    exc_time = 0
    start_time = 0
    end_time = 0
    renderer = pyrender.OffscreenRenderer(640, 480)

    # pygame (pedal)
    pygame.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick connected!")
        exit()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

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

    hand = LeapHandRight() 
    config = Leaphand_Config("whole","whole")
    leap_hand = LeapNode()
    mlp = FingerMLP(hand, hand.ndof, hand.ndof).to(device)

    # load mlp model
    file_list = glob.glob(f"{POS2POS_TRANSLATER_DIR}/leaphand/generator_epoch_200_*.pth")
    if not file_list:
        raise FileNotFoundError("No model file found.")
    elif len(file_list) > 1:
        raise ValueError("Multiple model files found. Please specify one.")
    else:
        mlp.load_state_dict(torch.load(file_list[0]))     

    # phantom visualization model
    XArm_vis_model = RobotModel(robot_name='xarm', urdf_path='assets/robots/xarm6/xarm6_wo_ee_ori_jl.urdf', meshes_path='assets/robots/xarm6/meshes/')
    leaphand_vis_model = RobotModel(robot_name='leaphand', urdf_path='assets/robots/leap_hand/leap_hand_right_extended.urdf', meshes_path='assets/robots/leap_hand/meshes/visual')

    # April Tag 
    X_ArmTag25_path = "/data/gjx/human-retargeting/data/transform/X_ArmTag25.npy"
    X_ArmTag25 = np.load(X_ArmTag25_path)
    # X_ArmTag25[:3, 3] += [0.1, 0.0, 0.0]  # add bias between tag and arm to align imaginary and real
    # X_ArmTag25_euler = R.from_matrix(X_ArmTag25[:3, :3]).as_euler('XYZ', degrees=True)
    # X_ArmTag25_euler += [-7, -2, -3]
    # X_ArmTag25[:3, :3] = R.from_euler('XYZ', X_ArmTag25_euler, degrees=True).as_matrix()

    # wxyz_ArmTag25 = R.from_matrix(X_ArmTag25[:3, :3]).as_quat()  # viser only
    
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

        last_phantom_mode = False
        while True:
            print(f"execution time: {exc_time}")
            # get current mode
            pygame.event.get()
            pedal_value = joystick.get_axis(2)
            phantom_mode = pedal_value < 0

            # phantom arm calculation
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
            X_target = np.eye(4)
            X_target[:3, 3] = np.array(xarm_right_target_position)
            X_target[:3, :3] = R.from_euler("xyz", [-EE_right_euler[0]-90, -EE_right_euler[1], EE_right_euler[2]-180], degrees=True).as_matrix()
                
            # receive glove data
            # start_time = time.time()
            while True:
                data = ""
                chunk = glove_client_socket.recv(65536)
                data += chunk.decode("utf-8")

                right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
                # print(right_hand_matches)
                # print("***", len(right_hand_matches))
                right_hand_data = [float(value) for value in right_hand_matches[-28:]]
                if len(right_hand_data) == 28:
                    break
            # end_time = time.time()
            # if end_time - start_time > 0.02:
            #     print(end_time - start_time)
            
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # xarm trimesh
            config = arm.get_servo_angle(is_radian=True)[1][:6]
            start_qpos = [math.degrees(rad) for rad in config]
            X_target_m = X_target.copy()
            X_target_m[:3, 3] = X_target_m[:3, 3] / 1000.0
            _, xarm_qpos = xarm6_planner.mplib_ik(current_qpos=np.array(start_qpos), target_pose=X_target_m, return_closest=True)
            if xarm_qpos is None:
                continue
            xarm_trimesh = XArm_vis_model.get_trimesh_q(xarm_qpos)['visual'] # arm qpos

            # hand trimesh
            root_transform = XArm_vis_model.frame_status['eef_point'].get_matrix()[0].cpu().numpy()
            rotation = root_transform[:3, :3]
            translation = root_transform[:3, 3]
            euler = R.from_matrix(rotation).as_euler('XYZ')
            dummy_values = np.concatenate([translation, euler]) #virtual_joint_x/y/z/r/p/y
            ordered_joint_values = link2link(right_hand_data)
            joint_pos_full, sim_joint_values = non_collide_mlp(torch.tensor(ordered_joint_values, dtype=torch.float32))
            if phantom_mode:
                phantom_joint_value = joint_pos_full
            elif last_phantom_mode:
                leap_hand.set_allegro(phantom_joint_value)
            # rw_joint_values = rw_hand.read_pos()
            # sim_joint_values = leap_from_rw_to_sim(rw_joint_values, sim_hand.actuated_joint_names)
            sim_dummy_joint_values = np.concatenate([dummy_values, sim_joint_values]) 
            leaphand_trimesh = leaphand_vis_model.get_trimesh_q(sim_dummy_joint_values)['visual'] # hand qpos
            # server.scene.add_mesh_trimesh('leaphand_trimesh', leaphand_trimesh)

            show_image = color_image.copy()
            tags = detector.detect(gray)
            if not tags:
                show_image[:10, :] = [0, 0, 255]  # 红色 (BGR 格式)
                show_image[-10:, :] = [0, 0, 255]
                show_image[:, :10] = [0, 0, 255]
                show_image[:, -10:] = [0, 0, 255]
                if phantom_mode:
                    print("No tag detected! Unable to show phantom.")
                show_image = cv2.resize(show_image, (1280, 960))
                cv2.imshow('TelePhantom', show_image)
                continue

            tag = tags[0]
            if phantom_mode:
                # get camera frame
                camera_tf3d = estimator.estimate(tag).inverse()
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
            
                camera_xyzw = (camera_wxyz[1], camera_wxyz[2], camera_wxyz[3], camera_wxyz[0])
                camera_rotmat = R.from_quat(camera_xyzw).as_matrix()
                X_Tag25Camera2 = np.eye(4)
                X_Tag25Camera2[:3, :3] = camera_rotmat
                X_Tag25Camera2[:3, 3] = np.array(camera_position)
                X_ArmCamera2 = X_ArmTag25 @ X_Tag25Camera2
                # X_ArmCamera2 = X_Tag25Camera2

                X_ArmCamera2[:3, 3] += np.array([0.06, -0.04, 0.0])
                X_ArmPhantom = np.eye(4)
                X_ArmPhantom[:3, 3] = np.array([0.0, 0.0, 0.0])
                X_ArmPhantom[:3, :3] = R.from_euler("XYZ", [-5, -2.05, 1.5], degrees=True).as_matrix()
                X_ArmCamera2 = X_ArmPhantom @ X_ArmCamera2
                
                combined_trimesh = trimesh.util.concatenate([xarm_trimesh, leaphand_trimesh])
                # combined_trimesh.show()

                # X_ArmCamera2 = [[-0.9998680353164673, 0.016218112781643867, 0.0008981706923805177, 0.4588461220264435],
                #     [0.01620866172015667, 0.999822199344635, -0.009639588184654713, -0.011901105754077435],
                #     [-0.0010543467942625284, -0.009623757563531399, -0.9999530911445618, 0.9407138824462891],
                #     [0, 0, 0, 1]]
                camera_pose = np.array(X_ArmCamera2)
                rotation_matrix = np.array([
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                ])
                camera_pose = np.dot(camera_pose, rotation_matrix)

                scene = pyrender.Scene()
                scene.bg_color = np.array([0, 0, 0, 0])

                camera = pyrender.PerspectiveCamera(yfov=fov_y)
                scene.add(camera, pose=camera_pose)

                light = pyrender.PointLight(color=np.ones(3), intensity=10.0)
                scene.add(light, pose=camera_pose)

                ambient_light = pyrender.DirectionalLight(color=np.ones(3) * 0.1, intensity=0.2)
                scene.add(ambient_light, pose=camera_pose)

                emissive_material = pyrender.MetallicRoughnessMaterial(
                    baseColorFactor=np.array([0.7, 0.7, 0.7, 1.0]),  # 银色金属外观
                    metallicFactor=0.9,  # 金属感
                    roughnessFactor=0.5,  # 适中的粗糙度
                    emissiveFactor=np.array([0.7, 0.7, 0.7])
                )
                scene.add(pyrender.Mesh.from_trimesh(combined_trimesh, material=emissive_material))

                renderer = pyrender.OffscreenRenderer(640, 480)
                render_rgba, render_depth = renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
                mask = render_rgba[:, :, 3] != 0
                render_rgba = np.array(render_rgba[:, :, :3])
                cv2.imshow('Phantom', cv2.cvtColor(render_rgba, cv2.COLOR_RGB2BGR))
                render_rgba[:, :, 0] = (render_rgba[:, :, 0] * 0.8).astype(np.int8)
                render_rgba[:, :, 1] = (render_rgba[:, :, 1] * 0.8).astype(np.int8)
                render_rgba[:, :, 2] = (render_rgba[:, :, 2] * 0.3).astype(np.int8)
                opacity = 0.5
                show_image[mask] = show_image[mask] * opacity + render_rgba[mask] * (1 - opacity)
            else:
                show_image = color_image.copy()
            
            show_image = cv2.resize(show_image, (1280, 960))
            cv2.imshow('TelePhantom', show_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # set XArm
            if last_phantom_mode and not phantom_mode:
                start_time = time.time()
                xyz, rpy = phantom_xyz, phantom_rpy
                joint = phantom_joint
            else:
                xyz = xarm_right_target_position
                rpy = R.from_euler("xyz", np.array([EE_right_euler[2], -EE_right_euler[1], -EE_right_euler[0]]), degrees=True).as_euler("ZYX", degrees=True)
                rpy += [-90, 0, -180]
                ordered_joint_values = link2link(right_hand_data)
                joint, sim_joint_values = non_collide_mlp(torch.tensor(ordered_joint_values, dtype=torch.float32))
            
            if not phantom_mode:
                arm.set_position(
                    x=xyz[0], y=xyz[1], z=xyz[2], 
                    roll=rpy[0], pitch=rpy[1], yaw=rpy[2], 
                    speed=150, wait=False
                )
                leap_hand.set_allegro(joint)

            if phantom_mode:
                end_time = time.time()
                if start_time != 0:
                    exc_time += end_time - start_time
                    start_time = 0
                phantom_xyz, phantom_rpy = xyz, rpy
                phantom_joint = joint

            last_phantom_mode = phantom_mode

if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    main()
