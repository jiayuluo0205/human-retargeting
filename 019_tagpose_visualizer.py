import contextlib
import cv2

import numpy as np
import numpy.typing as npt
import pyrealsense2 as rs  # type: ignore
from scipy.spatial.transform import Rotation as R
from robotpy_apriltag import AprilTag, AprilTagPoseEstimator, AprilTagDetector

import viser

'''
image size?
tag coordinate?
camera coordinate?
'''

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
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming.
    pipeline.start(config)

    yield pipeline

    # Close pipeline when done.
    pipeline.stop()


def main():
    # Start visualization server.
    server = viser.ViserServer(port=8080)
    wxyz = R.from_euler("YZX", [180.0, 45.0, 54.45], degrees=True).as_quat()[[3, 0, 1, 2]]
    position = (2.6, 2.6, 2.6)

    with realsense_pipeline() as pipeline:
        profile = pipeline.get_active_profile()
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        # intrinsc matrix of camera
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy
        print(cx, cy, fx, fy)

        detector = AprilTagDetector()
        detector.addFamily(fam="tag36h11")
        estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(fx=fx, fy=fy, cx=cx, cy=cy, tagSize=0.14))

        X_ImageCamera = np.eye(4)
        X_ImageCamera[:2, 3] = np.array((-0.5, -0.5))
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_frame = frames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())

            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray)
            for tag in tags:
                cv2.circle(color_image, (int(tag.getCorner(0).x), int(tag.getCorner(0).y)), 4,(255,0,0), 2)
                cv2.circle(color_image, (int(tag.getCorner(1).x), int(tag.getCorner(1).y)), 4,(255,0,0), 2)
                cv2.circle(color_image, (int(tag.getCorner(2).x), int(tag.getCorner(2).y)), 4,(255,0,0), 2)
                cv2.circle(color_image, (int(tag.getCorner(3).x), int(tag.getCorner(3).y)), 4,(255,0,0), 2)
                cv2.circle(color_image, (int(tag.getCenter().x), int(tag.getCenter().y)), 4,(255,0,0), 2)

                u, v = int(tag.getCenter().x), int(tag.getCenter().y)
                d = depth_image[v, u]
                x = (u - cx) * d / fx
                y = (v - cy) * d / fy
                z = d
                
                tag_rotmat = tag.getHomographyMatrix()
                tag_tf3d = estimator.estimate(tag)

                X_CameraTag = np.eye(4)
                X_CameraTag[:3, 3] = np.array((x/1000, y/1000, z/1000))
                X_CameraTag[:3, :3] = tag_rotmat
                X_ImageTag = X_ImageCamera @ X_CameraTag

                tag_wxyz = R.from_matrix(X_ImageTag[:3, :3]).as_quat()[[3, 0, 1, 2]]
                tag_position = (X_ImageTag[0, 3], X_ImageTag[1, 3], 0.0)
                # print(f"tag position:{tag_position}")
                tag_wxyz = (
                    tag_tf3d.rotation().getQuaternion().inverse().W(), 
                    tag_tf3d.rotation().getQuaternion().inverse().X(), 
                    tag_tf3d.rotation().getQuaternion().inverse().Y(), 
                    tag_tf3d.rotation().getQuaternion().inverse().Z()
                )
                tag_position = (
                    -tag_tf3d.translation().X(),
                    -tag_tf3d.translation().Y(),
                    -tag_tf3d.translation().Z()
                )
                print(tag_position)
                server.scene.add_frame("/world/tag_frame", wxyz=tag_wxyz, position=tag_position)


            # server.scene.add_frame("/world/color_frame/camera_frame", position=(-0.5, -0.5, 0))
            server.scene.add_frame("/world/color_frame", wxyz=wxyz, position=position)
            # server.scene.add_image(image=color_image, render_width=1, render_height=1, name="original", position=(0,0,0))
            # server.scene.add_image(image=color_image, render_width=100, render_height=100, name="color", position=position, wxyz=wxyz)




if __name__ == "__main__":
    main()