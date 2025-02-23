import cv2
import numpy as np
from robotpy_apriltag import AprilTag, AprilTagPoseEstimator, AprilTagDetector
import pyrealsense2 as rs  # type: ignore
help(rs)
breakpoint()
import contextlib
from scipy.spatial.transform import Rotation as R

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

with realsense_pipeline() as pipeline:
    detector = AprilTagDetector()
    detector.addFamily(fam="tag36h11")
    # detector.addFamily(fam="tag25h9")
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
        print("fx, fy, cx, cy: ", fx, fy, cx, cy)
        estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(fx=fx, fy=fy, cx=cx, cy=cy, tagSize=0.064))
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)
        for tag in tags:
            cv2.circle(color_image, (int(tag.getCorner(0).x), int(tag.getCorner(0).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCorner(1).x), int(tag.getCorner(1).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCorner(2).x), int(tag.getCorner(2).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCorner(3).x), int(tag.getCorner(3).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCenter().x), int(tag.getCenter().y)), 4,(255,0,0), 2)
            tag_tf3d = estimator.estimate(tag)
            tag_xyzw = ( 
                tag_tf3d.rotation().getQuaternion().X(), 
                tag_tf3d.rotation().getQuaternion().Y(), 
                tag_tf3d.rotation().getQuaternion().Z(),
                tag_tf3d.rotation().getQuaternion().W()
            )
            tag_position = (
                tag_tf3d.translation().X(),
                tag_tf3d.translation().Y(),
                tag_tf3d.translation().Z()
            )
            # print(tag_xyzw)
            X_CameraTag25 = np.eye(4) #147122075879
            X_CameraTag25[:3, :3] = R.from_quat(tag_xyzw).as_matrix()
            X_CameraTag25[:3, 3] = tag_position
            np.save("data/transform/camera_tag25.npy", X_CameraTag25)

        # # 显示检测结果
        cv2.imshow('capture', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break