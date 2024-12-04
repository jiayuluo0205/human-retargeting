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

def main():
    arm = XArmAPI('192.168.1.208')
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)

    # Start viser server.
    server = viser.ViserServer()
    XArm_model = RobotModel(robot_name='xarm', urdf_path='assets/robots/xarm6/xarm6_wo_ee_ori_jl.urdf', meshes_path='assets/robots/xarm6/meshes/')
    
    rw_hand = LeapNode(torque_enable=False)    
    sim_hand = LeapHandRight(load_visual_mesh=True, load_col_mesh=False, load_balls_urdf=False, load_n_collision_point=0)

    leaphand_model = RobotModel(robot_name='leaphand', urdf_path='assets/robots/leap_hand/leap_hand_right_extended.urdf', meshes_path='assets/robots/leap_hand/meshes/visual')

    tag_wxyz = R.from_euler("YXZ", [0.0, 180.0, 270.0], degrees=True).as_quat()[[3, 0, 1, 2]]
    server.scene.add_frame("/tag", wxyz=tag_wxyz)
    
    with realsense_pipeline() as pipeline:
        profile = pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        # intrinsc matrix of camera
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy
        focal_length = (intrinsics.fx, intrinsics.fy)
        principal_point = (intrinsics.ppx, intrinsics.ppy)
        print(focal_length)
        print(principal_point)
        fov_y = 2 * math.atan(cy / fy)

        detector = AprilTagDetector()
        detector.addFamily(fam="tag36h11")
        estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(fx=fx, fy=fy, cx=cx, cy=cy, tagSize=0.095))


        while True:
          
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
                    # print(camera_position)
                    server.scene.add_frame("/tag/camera", wxyz=camera_wxyz, position=camera_position)

                    
                    X_WorldTag = np.eye(4)
                    tag_xyzw = (tag_wxyz[1], tag_wxyz[2], tag_wxyz[3], tag_wxyz[0])
                    X_WorldTag[:3, :3] = R.from_quat(tag_xyzw).as_matrix()
                    
                    camera_xyzw = (camera_wxyz[1], camera_wxyz[2], camera_wxyz[3], camera_wxyz[0])
                    camera_rotmat = R.from_quat(camera_xyzw).as_matrix()
                    X_TagCamera = np.eye(4)
                    X_TagCamera[:3, :3] = camera_rotmat
                    X_TagCamera[:3, 3] = np.array(camera_position)

                    X_WorldCamera = X_WorldTag @ X_TagCamera
                    
                    clients = server.get_clients()
                    for id, client in clients.items():
                        client.camera.wxyz = R.from_matrix(X_WorldCamera[:3, :3]).as_quat()[[3, 0, 1, 2]]
                        client.camera.position = X_WorldCamera[:3, 3]
                        client.camera.fov = fov_y
                        new_image = client.camera.get_render(height=480, width=640, transport_format="png")
                        mask = new_image[:, :, 3] != 0
                        new_image[:, :, 0] = (new_image[:, :, 0] * 0.4).astype(np.int8)
                        new_image[:, :, 1] = (new_image[:, :, 1] * 0.75).astype(np.int8)
                        new_image[:, :, 2] = (new_image[:, :, 2] * 1).astype(np.int8)
                        # print(new_image[mask].mean(axis=0))
                        opacity = 0.3
                        color_image[mask] = color_image[mask] * opacity + new_image[:, :, :3][mask] * (1 - opacity)
                        cv2.imshow('2CFuture', color_image)
                    
                    # viser_urdf.update_cfg(np.array(config))
                    # hand read pos
                    rw_joint_values = rw_hand.read_pos()
                    sim_joint_values = leap_from_rw_to_sim(rw_joint_values, sim_hand.actuated_joint_names)
                    sim_to_rw_joint_values = leap_from_sim_to_rw(sim_joint_values, sim_hand.actuated_joint_names)
                    assert np.allclose(sim_to_rw_joint_values, rw_joint_values)

                    config = arm.get_servo_angle(is_radian=True)[1][:6]
                    xarm_trimesh = XArm_model.get_trimesh_q(config)['visual']
                    server.scene.add_mesh_trimesh('xarm_trimesh', xarm_trimesh)

                    root_transform = XArm_model.frame_status['eef_point'].get_matrix()[0].cpu().numpy()
                    rotation = root_transform[:3, :3]
                    translation = root_transform[:3, 3]
                    euler = R.from_matrix(rotation).as_euler('XYZ')
                    dummy_values = np.concatenate([translation, euler])
 
                    
                    sim_dummy_joint_values = np.concatenate([dummy_values, sim_joint_values]) #virtual_joint_x/y/z/r/p/y


                    leaphand_trimesh = leaphand_model.get_trimesh_q(sim_dummy_joint_values)['visual']
                    server.scene.add_mesh_trimesh('leaphand_trimesh', leaphand_trimesh)

            else:
                color_image[:10, :] = [0, 0, 255]  # 红色 (BGR 格式)
                color_image[-10:, :] = [0, 0, 255]
                color_image[:, :10] = [0, 0, 255]
                color_image[:, -10:] = [0, 0, 255]
                cv2.imshow('2CFuture', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    tyro.cli(main)