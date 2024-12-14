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
from xarm6_interface.utils  import MultiRealsense

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
    serial_number = '233622079809'
    config.enable_device(serial_number)
    #
    config.enable_device('147122075879')

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
    # arm.set_mode(0)
    # arm.set_state(state=0)
    # xarm_right_init_pos = [400, 100, 450]
    # arm.set_position(
    #     x=xarm_right_init_pos[0], y=xarm_right_init_pos[1], z=xarm_right_init_pos[2],
    #     roll=-180, pitch=0, yaw=0, speed=100, is_radian=False, wait=True
    # )
    arm.set_mode(7)
    arm.set_state(0)
    scale = 1.5

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
    wxyz_ArmTag25 = R.from_matrix(X_ArmTag25[:3, :3]).as_quat()  # viser only

    # rw_hand = LeapNode(torque_enable=True)    
    # sim_hand = LeapHandRight(load_visual_mesh=True, load_col_mesh=False, load_balls_urdf=False, load_n_collision_point=0)

    import viser
    server = viser.ViserServer()
    
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
            # get current mode
            pygame.event.get()
            pedal_value = joystick.get_axis(2)
            phantom_mode = pedal_value < 0

            # set XArm
            xyz = xarm_right_target_position
            rpy = R.from_euler("xyz", np.array([EE_right_euler[2], -EE_right_euler[1], -EE_right_euler[0]]), degrees=True).as_euler("ZYX", degrees=True)
            rpy += [-90, 0, -180]
            arm.set_position(
                x=xyz[0], y=xyz[1], z=xyz[2], 
                roll=rpy[0], pitch=rpy[1], yaw=rpy[2], 
                speed=150, wait=False
            )

            # set LeapHand
            ordered_joint_values = link2link(right_hand_data)
            joint_pos_full, sim_joint_values = non_collide_mlp(torch.tensor(ordered_joint_values, dtype=torch.float32))
            leap_hand.set_allegro(joint_pos_full)
            
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # xarm trimesh
            xarm_qpos = arm.get_servo_angle(is_radian=True)[1][:6]
            # print(xarm_qpos)

            # xarm_qpos = [19.0, 13.5, -57.2, -4.9, 44.4, -1.2]
            # xarm_qpos = [math.radians(degree) for degree in xarm_qpos]
            # print(xarm_qpos)
            xarm_trimesh = XArm_vis_model.get_trimesh_q(xarm_qpos)['visual'] # arm qpos

            # xarm_qpos = [1.7, -44.6, -53.8, -4.9, 96.3, -1.2]
            # xarm_qpos = [math.radians(degree) for degree in xarm_qpos]
            # print(xarm_qpos)

            # hand trimesh
            # root_transform = XArm_vis_model.frame_status['eef_point'].get_matrix()[0].cpu().numpy()
            # rotation = root_transform[:3, :3]
            # translation = root_transform[:3, 3]
            # euler = R.from_matrix(rotation).as_euler('XYZ')
            # dummy_values = np.concatenate([translation, euler]) #virtual_joint_x/y/z/r/p/y
            # rw_joint_values = rw_hand.read_pos()
            # sim_joint_values = leap_from_rw_to_sim(rw_joint_values, sim_hand.actuated_joint_names)
            # sim_dummy_joint_values = np.concatenate([dummy_values, sim_joint_values]) 
            # leaphand_trimesh = leaphand_vis_model.get_trimesh_q(sim_dummy_joint_values)['visual'] # hand qpos
            # server.scene.add_mesh_trimesh('leaphand_trimesh', leaphand_trimesh)

            show_image = color_image.copy()
            
            # camera_pose = np.array(X_ArmCamera2)
            camera_pose = np.array([
                [-0.9998680353164673, 0.016218112781643867, 0.0008981706923805177, 0.4588461220264435],
                [0.01620866172015667, 0.999822199344635, -0.009639588184654713, -0.011901105754077435],
                [-0.0010543467942625284, -0.009623757563531399, -0.9999530911445618, 0.9407138824462891],
                [0, 0, 0, 1]
            ])
            # camera_pose[:3, 3] += [0.03, -0.02, 0.0]
            X_ArmCamera = camera_pose

            X_PhantomArm = np.eye(4)
            X_PhantomArm[:3, 3] = [0.03, -0.02, 0.0]
            X_PhantomArm[:3, :3] = R.from_euler('Y', [-6], degrees=True).as_matrix()
            X_PhantomCamera = X_PhantomArm @ X_ArmCamera

            camera_pose = X_PhantomCamera

            rotation_matrix = np.array([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ])
            camera_pose = np.dot(camera_pose, rotation_matrix)

            # server.scene.add_mesh_trimesh('combined_trimesh', xarm_trimesh)
            # server.scene.add_frame("tag", wxyz=wxyz_ArmTag25, position=X_ArmTag25[:3, 3])
            # position = camera_pose[:3, 3]
            # wxyz = R.from_matrix(camera_pose[:3, :3]).as_quat()[[3, 0, 1, 2]]
            # server.scene.add_frame("camera", wxyz=wxyz, position=position)

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
            scene.add(pyrender.Mesh.from_trimesh(xarm_trimesh, material=emissive_material))

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
            
            show_image = cv2.resize(show_image, (1280, 960))
            cv2.imshow('TelePhantom', show_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            last_phantom_mode = phantom_mode

if __name__ == "__main__":
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    main()
