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
from xarm6_interface.utils.realsense import MultiRealsense, get_masked_pointcloud, remove_outliers

import os
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

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
    # load the collected data
    serial_number="241122074374" 
    exp_name = "1206_excalib_capture00"
    
    K_path = Path(__file__).resolve().parent.parent.parent / "data" / "camera" / serial_number / "K.npy"
    arm_right_cam_K = np.load(K_path)
    save_data_rel_dir_path = (Path(os.path.join(ROOT_DIR, f"3rdparty/xarm6/data/camera/{serial_number}")) / exp_name).resolve()
    # save_data_path = K_path.parent
    arm_right_cam_X_BaseCamera_path = save_data_rel_dir_path / "optimized_X_BaseCamera.npy"
    arm_right_cam_X_BaseCamera = np.load(arm_right_cam_X_BaseCamera_path)

    multi_rs = MultiRealsense([serial_number])
    multi_rs.set_intrinsics(0, arm_right_cam_K[0, 0], arm_right_cam_K[1, 1], arm_right_cam_K[0, 2], arm_right_cam_K[1, 2])
    camera_wxyzs = [
        R.from_matrix(arm_right_cam_X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]],
    ]
    camera_positions = [arm_right_cam_X_BaseCamera[:3, 3]]
    X_BaseCamera_list = [arm_right_cam_X_BaseCamera]

    arm = XArmAPI('192.168.1.208')
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)

    # Start viser server.
    server = viser.ViserServer()

    tag_wxyz = R.from_euler("YXZ", [0.0, 180.0, 270.0], degrees=True).as_quat()[[3, 0, 1, 2]]
    server.scene.add_frame("/tag", wxyz=tag_wxyz)

    # top_cam_serial = '241122074374'
    # camera_serial_nums = [top_cam_serial]
    # multi_rs = MultiRealsense(camera_serial_nums)
    # # arm_right_cam_K_path = Path("third_party/xarm6/data/camera/mounted_white/K.npy")
    # arm_right_cam_K_path = Path(f"third_party/xarm6/data/camera/{top_cam_serial}/K.npy")
    # arm_right_cam_K = np.load(arm_right_cam_K_path)
    # arm_right_cam_X_BaseCamera_path = Path(f"third_party/xarm6/data/camera/{top_cam_serial}/1113_excalib_capture02/optimized_X_BaseCamera.npy")
    # # arm_right_cam_X_BaseCamera_path = Path("third_party/xarm6/data/camera/mounted_top/1112_excalib_capture00/optimized_X_BaseCamera.npy")
    # arm_right_cam_X_BaseCamera = np.load(arm_right_cam_X_BaseCamera_path)
    # multi_rs.set_intrinsics(0, arm_right_cam_K[0, 0], arm_right_cam_K[1, 1], arm_right_cam_K[0, 2], arm_right_cam_K[1, 2])
    # camera_wxyzs = [
    #     R.from_matrix(arm_right_cam_X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]],
    # ]
    # camera_positions = [arm_right_cam_X_BaseCamera[:3, 3]]
    # X_BaseCamera_list = [arm_right_cam_X_BaseCamera]

    


    
    with realsense_pipeline() as pipeline:
        profile = pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        # intrinsc matrix of camera
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy
        # focal_length = (intrinsics.fx, intrinsics.fy)
        # principal_point = (intrinsics.ppx, intrinsics.ppy)
        # print(focal_length)
        # print(principal_point)
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



if __name__ == "__main__":
    tyro.cli(main)