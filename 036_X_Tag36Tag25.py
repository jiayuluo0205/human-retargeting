import cv2
import numpy as np
from robotpy_apriltag import AprilTag, AprilTagPoseEstimator, AprilTagDetector
import pyrealsense2 as rs  # type: ignore
import contextlib
from scipy.spatial.transform import Rotation as R
import viser
import tyro

@contextlib.contextmanager
def realsense_pipeline(fps: int = 30):
    """Context manager that yields a RealSense pipeline."""

    # Configure depth and color streams.
    pipeline = rs.pipeline()  # type: ignore
    config = rs.config()  # type: ignore

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)  # type: ignore
    config.resolve(pipeline_wrapper)

    config.enable_stream(rs.stream.depth, rs.format.z16, fps)  # type: ignore
    config.enable_stream(rs.stream.color, rs.format.rgb8, fps)  # type: ignore

    # Start streaming.
    pipeline.start(config)

    yield pipeline

    # Close pipeline when done.
    pipeline.stop()

def main():
    server = viser.ViserServer()
    with realsense_pipeline() as pipeline:
        X_Camera2Tag36_path = "/data/gjx/human-retargeting/data/transform/X_Camera2Tag36.npy"
        X_Camera2Tag25_path = "/data/gjx/human-retargeting/data/transform/X_Camera2Tag25.npy"
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            
            """validation"""
            X_Camera2Tag36 = np.load(X_Camera2Tag36_path)
            X_Camera2Tag25 = np.load(X_Camera2Tag25_path)

            X_Tag36Camera2 = np.linalg.inv(X_Camera2Tag36)
            X_Tag36Tag25 = X_Tag36Camera2 @ X_Camera2Tag25
            
            # wxyz_Tag25 = R.from_matrix(X_Camera2Tag25[:3, :3]).as_quat()[[3, 0, 1, 2]]
            # wxyz_Tag36 = R.from_matrix(X_Camera2Tag36[:3, :3]).as_quat()[[3, 0, 1, 2]]
            # server.scene.add_frame("Tag25", wxyz=wxyz_Tag25, position=X_Camera2Tag25[:3, 3])
            # server.scene.add_frame("Tag36", wxyz=wxyz_Tag36, position=X_Camera2Tag36[:3, 3])

            wxyz_Tag25 = R.from_matrix(X_Tag36Tag25[:3, :3]).as_quat()[[3, 0, 1, 2]]
            server.scene.add_frame("Tag25", wxyz=wxyz_Tag25, position=X_Tag36Tag25[:3, 3])

            """save"""
            np.save("data/transform/X_Tag36Tag25.npy", X_Tag36Tag25)

if __name__ == "__main__":
    tyro.cli(main)