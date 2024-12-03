import cv2
import numpy as np
from robotpy_apriltag import AprilTag, AprilTagPoseEstimate, AprilTagDetector
import pyrealsense2 as rs  # type: ignore
import contextlib

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
    detector.addFamily(fam="tag25h9")
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)
        for tag in tags:
            cv2.circle(color_image, (int(tag.getCorner(0).x), int(tag.getCorner(0).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCorner(1).x), int(tag.getCorner(1).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCorner(2).x), int(tag.getCorner(2).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCorner(3).x), int(tag.getCorner(3).y)), 4,(255,0,0), 2)
            cv2.circle(color_image, (int(tag.getCenter().x), int(tag.getCenter().y)), 4,(255,0,0), 2)

        # # 显示检测结果
        cv2.imshow('capture', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break