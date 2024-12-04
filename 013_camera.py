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
    pipeline.stop()


def main():
    # Start visualization server.
    server = viser.ViserServer(port=8080)
    wxyz = R.from_euler("YZX", [180.0, 45.0, 54.45], degrees=True).as_quat()[[3, 0, 1, 2]]
    position = (2.6, 2.6, 2.6)

    with realsense_pipeline() as pipeline:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            
            server.scene.add_frame("/world/color_frame", wxyz=wxyz, position=position)

            server.scene.add_image(image=color_image, render_width=1, render_height=1, name="original", position=(0,0,0))

            server.scene.add_image(image=color_image, render_width=1, render_height=1, name="color", position=position, wxyz=wxyz)


if __name__ == "__main__":
    main()