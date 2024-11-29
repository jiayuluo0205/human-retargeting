from __future__ import annotations

import contextlib

import numpy as np
import numpy.typing as npt
import pyrealsense2 as rs  # type: ignore
from tqdm.auto import tqdm
from scipy.spatial.transform import Rotation as R

import viser


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
    pipeline.close()


def main():
    # Start visualization server.
    server = viser.ViserServer(port=8080)
    with realsense_pipeline() as pipeline:
        for i in tqdm(range(10000000)):
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

            T_WorldCamera = viser.extras.Record3dFrame.T_world_camera
            print(T_WorldCamera)

            wxyz = R.from_euler("zxy", [45.0, 0.0, 0.0], degrees=True).as_quat()[[3, 0, 1, 2]]
            
            server.scene.add_frame(
                "color", wxyz=wxyz, position=(0,0,0)
            )
            server.scene.add_image(image=color_image, render_width=1, render_height=1, name="color", position=(0,0,0), wxyz=wxyz)


if __name__ == "__main__":
    main()