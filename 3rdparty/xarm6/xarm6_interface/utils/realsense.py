import cv2
from pathlib import Path
import time
from loguru import logger as lgr
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R


def get_profiles(verbose=False):
    # ----------------------------------------------------------------------------
    # -                        Open3D: www.open3d.org                            -
    # ----------------------------------------------------------------------------
    # Copyright (c) 2018-2023 www.open3d.org
    # SPDX-License-Identifier: MIT
    # ----------------------------------------------------------------------------

    # examples/python/reconstruction_system/sensors/realsense_helper.py

    # pyrealsense2 is required.
    # Please see instructions in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python
    ctx = rs.context()
    devices = ctx.query_devices()

    color_profiles = []
    depth_profiles = []
    for device in devices:
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        if verbose:
            print("Sensor: {}, {}".format(name, serial))
            print("Supported video formats:")
        for sensor in device.query_sensors():
            for stream_profile in sensor.get_stream_profiles():
                stream_type = str(stream_profile.stream_type())

                if stream_type in ["stream.color", "stream.depth"]:
                    v_profile = stream_profile.as_video_stream_profile()
                    fmt = stream_profile.format()
                    w, h = v_profile.width(), v_profile.height()
                    fps = v_profile.fps()

                    video_type = stream_type.split(".")[-1]
                    if verbose:
                        print(
                            "  {}: width={}, height={}, fps={}, fmt={}".format(
                                video_type, w, h, fps, fmt
                            )
                        )
                    if video_type == "color":
                        color_profiles.append((w, h, fps, fmt))
                    else:
                        depth_profiles.append((w, h, fps, fmt))

    return color_profiles, depth_profiles



class Realsense:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        color_profiles, depth_profiles = get_profiles()
        w, h, fps, fmt = depth_profiles[1]
        self.config.enable_stream(rs.stream.depth, w, h, fmt, fps)
        lgr.info("Depth stream: {}x{} @ {}fps, {}".format(w, h, fps, fmt))
        w, h, fps, fmt = color_profiles[19]
        self.config.enable_stream(rs.stream.color, w, h, fmt, fps)
        lgr.info("Color stream: {}x{} @ {}fps, {}".format(w, h, fps, fmt))
        lgr.info("Realsense camera initialized")
        self.w = w
        self.h = h
        self.fps = fps
        self.align_to_color = rs.align(rs.stream.color)
        self.pipe_profile = self.pipeline.start(self.config)
        self.intr = (
            self.pipe_profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        self.K = np.array(
            [
                [self.intr.fx, 0, self.intr.ppx],
                [0, self.intr.fy, self.intr.ppy],
                [0, 0, 1],
            ]
        )
        lgr.info("Intrinsics: {}".format(self.intr))
        self.fov_x = 2 * np.arctan(
            self.intr.width / (2 * self.intr.fx)
        )  # Horizontal FOV in radians
        self.fov_y = 2 * np.arctan(
            self.intr.height / (2 * self.intr.fy)
        )  # Vertical FOV in radians
        self.aspect_ratio = self.intr.width / self.intr.height
        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            self.intr.width,
            self.intr.height,
            self.intr.fx,
            self.intr.fy,
            self.intr.ppx,
            self.intr.ppy,
        )

    def set_intrinsics(self, fx, fy, ppx, ppy):
        self.K = np.array(
            [
                [fx, 0, ppx],
                [0, fy, ppy],
                [0, 0, 1],
            ]
        )
        self.intr.fx = fx
        self.intr.fy = fy
        self.intr.ppx = ppx
        self.intr.ppy = ppy
        self.fov_x = 2 * np.arctan(self.intr.width / (2 * self.intr.fx))
        self.fov_y = 2 * np.arctan(self.intr.height / (2 * self.intr.fy))
        self.aspect_ratio = self.intr.width / self.intr.height
        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            self.intr.width,
            self.intr.height,
            self.intr.fx,
            self.intr.fy,
            self.intr.ppx,
            self.intr.ppy,
        )
        lgr.info("Intrinsics: {}".format(self.intr))

    def getCurrentData(self, pointcloud=True):
        self.frames = self.pipeline.wait_for_frames()
        self.align_frames = self.align_to_color.process(self.frames)
        self.depth_frame = self.align_frames.get_depth_frame()
        self.color_frame = self.align_frames.get_color_frame()
        self.depth_color_frame = rs.colorizer().colorize(self.depth_frame)
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())
        self.depth_color_image = np.asanyarray(self.depth_color_frame.get_data())
        rtr_dict = {}
        rtr_dict["rgb"] = self.color_image
        rtr_dict["depth"] = self.depth_image
        rtr_dict["colored_depth"] = self.depth_color_image
        rtr_dict["pointcloud_o3d"] = None

        if pointcloud:
            depth = o3d.geometry.Image(self.depth_image)
            rgb = o3d.geometry.Image(self.color_image)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                rgb, depth, convert_rgb_to_intensity=False
            )
            self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd, self.pinhole_camera_intrinsic
            )
            rtr_dict["pointcloud_o3d"] = self.pcd

        return rtr_dict

    def get_intrin_extrin(self):
        self.frames = self.pipeline.wait_for_frames()
        self.depth_frame = self.frames.get_depth_frame()
        self.color_frame = self.frames.get_color_frame()
        self.dprofile = self.depth_frame.get_profile()
        self.cprofile = self.color_frame.get_profile()
        self.cvsprofile = rs.video_stream_profile(self.cprofile)
        self.dvsprofile = rs.video_stream_profile(self.dprofile)
        self.color_intrin = self.cvsprofile.get_intrinsics()
        print("color_intrin", self.color_intrin)
        self.depth_intrin = self.dvsprofile.get_intrinsics()
        print("depth_intrin", self.depth_intrin)
        self.extrin = self.dprofile.get_extrinsics_to(self.cprofile)
        print("extrin: ", self.extrin)
        self.depth_sensor = self.pipe_profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        print("depth_scale", self.depth_scale)

    def create_window_and_capture_data(
        self,
        save_dir: Path = Path("data"),
        rgb: bool = True,
        depth: bool = True,
        pointcloud: bool = True,
        colored_depth: bool = False,
    ) -> None:

        # Ensure the save directory exists
        if not save_dir.exists():
            save_dir.mkdir(parents=True)

        index = 0  # Initialize the index for saved images
        while True:
            # Get the current data from the Realsense instance
            cap_dict = self.getCurrentData(pointcloud=pointcloud)
            color_image = cap_dict["rgb"]
            depth_image = cap_dict["depth"]
            colored_depth_image = cap_dict["colored_depth"]
            pcd = cap_dict["pointcloud_o3d"]

            # Display the RGB and colored depth images
            if rgb:
                cv2.imshow("Color Image", cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            if depth or colored_depth:
                cv2.imshow("Colored Depth Image", colored_depth_image)
            if pointcloud and pcd:
                pcd.transform(
                    [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
                )
                if not hasattr(self, "vis_o3d"):
                    self.vis_o3d = o3d.visualization.Visualizer()
                    self.vis_o3d.create_window("Pointcloud")
                    vis_pointcloud = o3d.geometry.PointCloud()
                    self.has_added_pointcloud = False

                vis_pointcloud.clear()
                vis_pointcloud += pcd

                if not self.has_added_pointcloud:
                    self.vis_o3d.add_geometry(vis_pointcloud)
                    self.has_added_pointcloud = True

                self.vis_o3d.update_geometry(vis_pointcloud)
                self.vis_o3d.poll_events()
                self.vis_o3d.update_renderer()

            # Wait for a key press
            key = cv2.waitKey(1) & 0xFF

            # If the space bar is pressed, save the images
            if key == ord(" "):
                if rgb:
                    color_filename = str((save_dir / f"color_{index:04d}.png").resolve())
                    cv2.imwrite(
                        color_filename, cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                    )

                if colored_depth:
                    colored_depth_filename = str((save_dir / f"colored_depth_{index:04d}.png").resolve())
                    cv2.imwrite(
                        colored_depth_filename,
                        cv2.cvtColor(colored_depth_image, cv2.COLOR_RGB2BGR),
                    )

                if depth:
                    depth_filename = str((save_dir / f"depth_{index:04d}.npy").resolve())
                    np.save(depth_filename, depth_image)

                if pointcloud:
                    pcd_filename = str((save_dir / f"pointcloud_{index:04d}.ply").resolve())
                    o3d.io.write_point_cloud(pcd_filename, pcd)

                index += 1

            # Exit on 'q' key press or ESC key press
            elif key == ord("q") or key == 27:
                break

        # Close all OpenCV windows
        cv2.destroyAllWindows()
        if pointcloud:
            self.vis_o3d.destroy_window()
        self.stop()

    def stop(self):
        self.pipeline.stop()


class MultiRealsense:
    def __init__(self, serial_numbers):
        """
        Initialize the MultiRealsense class to handle multiple cameras.

        Args:
        - serial_numbers (list of str): A list of serial numbers for each camera.
        """
        self.num_cameras = len(serial_numbers)
        self.serial_numbers = serial_numbers
        self.pipelines = []
        self.configs = []
        self.align_to_color = []
        self.intrinsics = []
        self.camera_data = [{} for _ in range(self.num_cameras)]
        self.filters = build_filters()

        # Initialize pipelines, configs, and align objects for each camera
        for i in range(self.num_cameras):
            pipeline = rs.pipeline()
            config = rs.config()

            # Enable the device by its serial number
            config.enable_device(self.serial_numbers[i])

            # Get profiles for each camera
            color_profiles, depth_profiles = get_profiles()
            w, h, fps, fmt = depth_profiles[1]
            config.enable_stream(rs.stream.depth, w, h, fmt, fps)
            lgr.info(f"Camera {i+1} (Serial: {self.serial_numbers[i]}): Depth stream: {w}x{h} @ {fps}fps, {fmt}")
            w, h, fps, fmt = color_profiles[19]
            config.enable_stream(rs.stream.color, w, h, fmt, fps)
            lgr.info(f"Camera {i+1} (Serial: {self.serial_numbers[i]}): Color stream: {w}x{h} @ {fps}fps, {fmt}")

            # Store pipeline, config, and alignment objects
            self.pipelines.append(pipeline)
            self.configs.append(config)
            self.align_to_color.append(rs.align(rs.stream.color))

        # Start all camera pipelines
        for pipeline, config in zip(self.pipelines, self.configs):
            pipe_profile = pipeline.start(config)
            intr = pipe_profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.intrinsics.append(intr)

            # Print intrinsics for each camera
            lgr.info(f"Intrinsics for Camera {self.serial_numbers[self.pipelines.index(pipeline)]}: {intr}")

    def set_intrinsics(self, camera_index, fx, fy, ppx, ppy):
        """
        Set the intrinsics for a specific camera.

        Args:
        - camera_index (int): Index of the camera to set intrinsics for (0-based).
        - fx (float): Focal length along the x-axis.
        - fy (float): Focal length along the y-axis.
        - ppx (float): Principal point offset along the x-axis.
        - ppy (float): Principal point offset along the y-axis.
        """
        if camera_index >= self.num_cameras:
            raise IndexError("Camera index out of range")

        # Update intrinsics for the specified camera
        intr = self.intrinsics[camera_index]
        intr.fx = fx
        intr.fy = fy
        intr.ppx = ppx
        intr.ppy = ppy

        # Update relevant matrices and parameters
        K = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]])
        self.camera_data[camera_index]['K'] = K

        fov_x = 2 * np.arctan(intr.width / (2 * fx))
        fov_y = 2 * np.arctan(intr.height / (2 * fy))
        aspect_ratio = intr.width / intr.height

        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            intr.width,
            intr.height,
            fx,
            fy,
            ppx,
            ppy
        )

        # Store the updated intrinsics
        self.camera_data[camera_index].update({
            'K': K,
            'fov_x': fov_x,
            'fov_y': fov_y,
            'aspect_ratio': aspect_ratio,
            'pinhole_camera_intrinsic': pinhole_camera_intrinsic
        })

        lgr.info(f"Updated intrinsics for camera {camera_index + 1}: {intr}")

    def getCurrentData(self, pointcloud=True, apply_filters=True):
        for i, pipeline in enumerate(self.pipelines):
            frames = pipeline.wait_for_frames()
            align_frames = self.align_to_color[i].process(frames)
            depth_frame = align_frames.get_depth_frame()
            if apply_filters:
                for f in self.filters:
                    depth_frame = f.process(depth_frame)
            color_frame = align_frames.get_color_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            # depth_image = smooth_depth_image(depth_image)
            color_image = np.asanyarray(color_frame.get_data())
            
            # Store data in the corresponding camera dictionary
            self.camera_data[i]['rgb'] = color_image
            self.camera_data[i]['depth'] = depth_image
            self.camera_data[i]['pointcloud_o3d'] = None

            if pointcloud:
                depth = o3d.geometry.Image(depth_image)
                rgb = o3d.geometry.Image(color_image)
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    rgb, depth, convert_rgb_to_intensity=False
                )
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd, self.camera_data[i]['pinhole_camera_intrinsic'])
                self.camera_data[i]['pointcloud_o3d'] = pcd

        return self.camera_data

    def create_window_and_capture_data(self, save_dir: Path = Path("data"), rgb: bool = True, depth: bool = True, pointcloud: bool = True):
        if not save_dir.exists():
            save_dir.mkdir(parents=True)

        index = 0  # Initialize the index for saved images
        while True:
            camera_data = self.getCurrentData(pointcloud=pointcloud)
            for i, data in enumerate(camera_data):
                color_image = data["rgb"]
                depth_image = data["depth"]
                pcd = data["pointcloud_o3d"]

                # Display RGB images from all cameras
                if rgb:
                    cv2.imshow(f"Camera {i+1} - Color Image", cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
                # Display colored depth images from all cameras
                if depth and 'colored_depth' in data:
                    cv2.imshow(f"Camera {i+1} - Colored Depth Image", data["colored_depth"])

                if pointcloud and pcd:
                    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                    vis_o3d = o3d.visualization.Visualizer()
                    vis_o3d.create_window(f"Camera {i+1} - Pointcloud")
                    vis_pointcloud = o3d.geometry.PointCloud()
                    vis_pointcloud += pcd
                    vis_o3d.add_geometry(vis_pointcloud)
                    vis_o3d.poll_events()
                    vis_o3d.update_renderer()

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

        # Close all OpenCV windows
        cv2.destroyAllWindows()
        if pointcloud:
            for vis_o3d in self.vis_o3d_list:
                vis_o3d.destroy_window()
        self.stop()

    def stop(self):
        for pipeline in self.pipelines:
            pipeline.stop()


def build_rs_cam(cam_K_path, X_BaseCamera_path):
    cam = Realsense()
    cam_K = np.load(cam_K_path)
    cam.set_intrinsics(fx=cam_K[0, 0], fy=cam_K[1, 1], ppx=cam_K[0, 2], ppy=cam_K[1, 2])
    X_BaseCamera = np.load(X_BaseCamera_path)
    cam_wxyz = R.from_matrix(X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]]
    cam_position = X_BaseCamera[:3, 3]
    rtr_dict = {
        "cam": cam,
        "cam_wxyz": cam_wxyz,
        "cam_position": cam_position,
        "X_BaseCamera": X_BaseCamera,
    }
    return rtr_dict
    
def get_masked_pointcloud(rgb, depth, mask, intrinsic):
    # Ensure the mask has the same shape as the depth image
    if mask.shape != depth.shape:
        raise ValueError("The mask shape must match the depth image shape")

    # Apply the mask to the depth image
    masked_depth = np.where(mask, depth, 0)

    # Convert masked depth and rgb images to Open3D format
    masked_depth_o3d = o3d.geometry.Image(masked_depth)
    rgb_o3d = o3d.geometry.Image(rgb)

    # Create an RGBD image from the masked depth and original RGB images
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, masked_depth_o3d, convert_rgb_to_intensity=False
    )

    # Generate the point cloud from the RGBD image
    masked_pc_o3d = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, intrinsic
    )

    return masked_pc_o3d

def build_filters():
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    temporal = rs.temporal_filter()
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)
    hole_filling = rs.hole_filling_filter()
    filters = [ depth_to_disparity, spatial, temporal, disparity_to_depth, hole_filling]
    return filters

def remove_outliers(point_cloud, nb_neighbors=20, std_ratio=2.0, nb_points=50, radius=0.02, z_min_thresh=None):
    """
    Remove outliers from a point cloud using Statistical Outlier Removal (SOR) and filter points below a Z threshold.

    Args:
    - point_cloud (o3d.geometry.PointCloud): The input noisy point cloud.
    - nb_neighbors (int): Number of neighbors to analyze for each point.
    - std_ratio (float): The standard deviation multiplier threshold.
    - nb_points (int): Minimum number of points within the radius for a point to be considered an inlier.
    - radius (float): Radius for the Radius Outlier Removal.
    - z_min_thresh (float or None): The minimum Z-coordinate threshold. Points with Z < z_min_thresh will be removed.

    Returns:
    - o3d.geometry.PointCloud: The filtered point cloud with outliers and points below Z threshold removed.
    """
    # Filter out points below the Z threshold, if specified
    if z_min_thresh is not None:
        # Get the numpy array of the points
        points = np.asarray(point_cloud.points)

        # Find the indices of points where Z is greater than or equal to z_min_thresh
        valid_indices = np.where(points[:, 2] >= z_min_thresh)[0]

        # Select points that meet the Z threshold condition
        filtered_point_cloud = point_cloud.select_by_index(valid_indices)
    else:
        filtered_point_cloud = point_cloud
    
    for i in range(5):
        # Apply statistical outlier removal
        cl, ind = filtered_point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        # Filter the inlier points
        filtered_point_cloud = filtered_point_cloud.select_by_index(ind)

        # Apply radius outlier removal
        cl, ind = filtered_point_cloud.remove_radius_outlier(nb_points=nb_points, radius=radius)
        # Filter the inlier points
        filtered_point_cloud = filtered_point_cloud.select_by_index(ind)

    return filtered_point_cloud

if __name__ == "__main__":

    camera = Realsense()
    K_path = Path("/home/aris/projects/xarm6/xarm6_interface/calib/K.npy")
    K = np.load(K_path)
    camera.set_intrinsics(K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    camera.create_window_and_capture_data(
        save_dir=Path("rs_calib"),
        rgb=True,
        depth=False,
        pointcloud=False,
        colored_depth=False,
    )
