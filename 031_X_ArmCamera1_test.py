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

    X_ArmCamera1_path = "3rdparty/xarm6/data/camera/241122074374/1206_excalib_capture00/optimized_X_BaseCamera.npy"
    X_ArmCamera1 = np.load(X_ArmCamera1_path) # 241122074374
    
    wxyz_ArmCamera1 = R.from_matrix(X_ArmCamera1[:3, :3]).as_quat()[[3, 0, 1, 2]]
    server.scene.add_frame("/world/camera1", wxyz=wxyz_ArmCamera1, position=X_ArmCamera1[:3, 3])
    print(X_ArmCamera1[:3, 3])

    while True:
        time.sleep(10)

if __name__ == "__main__":
    tyro.cli(main)