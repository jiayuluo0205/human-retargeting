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
        detector = AprilTagDetector()
        detector.addFamily(fam="tag36h11")
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

            profile = pipeline.get_active_profile()
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            fx, fy = intrinsics.fx, intrinsics.fy
            cx, cy = intrinsics.ppx, intrinsics.ppy

            estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(fx=fx, fy=fy, cx=cx, cy=cy, tagSize=0.178))
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray)
            for tag in tags:
                # X_Camera2Tag36
                tf3d_Camera2Tag36 = estimator.estimate(tag)
                xyzw_Camera2Tag36 = (
                    tf3d_Camera2Tag36.rotation().getQuaternion().X(), 
                    tf3d_Camera2Tag36.rotation().getQuaternion().Y(), 
                    tf3d_Camera2Tag36.rotation().getQuaternion().Z(),
                    tf3d_Camera2Tag36.rotation().getQuaternion().W()
                )
                wxyz_Camera2Tag36 = (
                    tf3d_Camera2Tag36.rotation().getQuaternion().W(),
                    tf3d_Camera2Tag36.rotation().getQuaternion().X(), 
                    tf3d_Camera2Tag36.rotation().getQuaternion().Y(), 
                    tf3d_Camera2Tag36.rotation().getQuaternion().Z()
                )
                position_Camera2Tag36 = (
                    tf3d_Camera2Tag36.translation().X(),
                    tf3d_Camera2Tag36.translation().Y(),
                    tf3d_Camera2Tag36.translation().Z()
                )
                
                """validation"""
                server.scene.add_frame("Tag36", wxyz=wxyz_Camera2Tag36, position=position_Camera2Tag36)

                """save"""
                X_Camera2Tag36 = np.eye(4)
                X_Camera2Tag36[:3, :3] = R.from_quat(xyzw_Camera2Tag36).as_matrix()
                X_Camera2Tag36[:3, 3] = position_Camera2Tag36
                np.save("data/transform/X_Camera2Tag36.npy", X_Camera2Tag36)

if __name__ == "__main__":
    tyro.cli(main)