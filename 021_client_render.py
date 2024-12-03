from __future__ import annotations

import time
from typing import Literal

import numpy as np
import tyro

import viser
from viser.extras import ViserUrdf
from pathlib import Path
import contextlib
import cv2
import pyrealsense2 as rs
from robotpy_apriltag import AprilTag, AprilTagPoseEstimator, AprilTagDetector
from scipy.spatial.transform import Rotation as R

def create_robot_control_sliders(
    server: viser.ViserServer, viser_urdf: ViserUrdf
) -> tuple[list[viser.GuiInputHandle[float]], list[float]]:
    """Create slider for each joint of the robot. We also update robot model
    when slider moves."""
    slider_handles: list[viser.GuiInputHandle[float]] = []
    initial_config: list[float] = []
    for joint_name, (
        lower,
        upper,
    ) in viser_urdf.get_actuated_joint_limits().items():
        lower = lower if lower is not None else -np.pi
        upper = upper if upper is not None else np.pi
        initial_pos = 0.0 if lower < 0 and upper > 0 else (lower + upper) / 2.0
        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=1e-3,
            initial_value=initial_pos,
        )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: viser_urdf.update_cfg(
                np.array([slider.value for slider in slider_handles])
            )
        )
        slider_handles.append(slider)
        initial_config.append(initial_pos)
    return slider_handles, initial_config

@contextlib.contextmanager
def realsense_pipeline(fps: int = 30):
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
    # Start viser server.
    server = viser.ViserServer()
    urdf_path = Path("D:/Projects/human-retargeting/3rdparty/xarm6/assets/xarm6/xarm6_wo_ee_ori_jl.urdf")
    # Load URDF.
    #
    # This takes either a yourdfpy.URDF object or a path to a .urdf file.
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path = urdf_path,
        scale=1
    )
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
        import math
        fov_y = 2 * math.atan(cy / fy)

        detector = AprilTagDetector()
        detector.addFamily(fam="tag36h11")
        estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(fx=fx, fy=fy, cx=cx, cy=cy, tagSize=0.17))

        # Create sliders in GUI that help us move the robot joints.
        with server.gui.add_folder("Joint position control"):
            (slider_handles, initial_config) = create_robot_control_sliders(
                server, viser_urdf
            )

        # Set initial robot configuration.
        viser_urdf.update_cfg(np.array(initial_config))

        # Create joint reset button.
        reset_button = server.gui.add_button("Reset")

        @reset_button.on_click
        def _(_):
            for s, init_q in zip(slider_handles, initial_config):
                s.value = init_q

        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            tags = detector.detect(gray)
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
                    # cv2.imshow("image", new_image)
                    mask = new_image[:, :, 3] != 0
                    color_image[mask] = new_image[:, :, :3][mask]
                    cv2.imshow('new_image', color_image)

            # cv2.imshow('capture', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    tyro.cli(main)