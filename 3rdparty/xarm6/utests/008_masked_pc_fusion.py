import sys
import os
import cv2
import numpy as np
from glob import glob
import matplotlib.pyplot as plt
from loguru import logger as lgr
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import open3d as o3d

def create_colored_point_cloud(image, depth, camera_K, mask=None):
    """Create a colored point cloud from the RGB image, depth image, and camera intrinsic matrix."""
    # Get image dimensions
    h, w = depth.shape
    
    # Create a mesh grid of pixel coordinates
    i, j = np.indices((h, w))
    
    # Flatten the depth image
    z = depth.flatten()
    i = i.flatten()
    j = j.flatten()
    
    # Convert pixel coordinates to normalized camera coordinates
    x = (j - camera_K[0, 2]) / camera_K[0, 0] * z
    y = (i - camera_K[1, 2]) / camera_K[1, 1] * z
    
    # Stack to create 3D points
    points = np.vstack((x, y, z)).T

    # Mask the points
    if mask is not None:
        mask = mask.flatten()
        points = points[mask > 0]
        colors = image.reshape(-1, 3)[mask > 0]
    else:
        colors = image.reshape(-1, 3)
    
    # Return the point cloud with associated colors
    return points, colors

def transform_points(points, pose):
    """Apply a transformation pose to a set of points."""
    # Apply rotation and translation
    # pose_inv = np.linalg.inv(pose)
    transformed_points = (pose @ np.vstack((points.T, np.ones(points.shape[0])))).T[:, :3]
    return transformed_points


if __name__ == "__main__":
    save_dir = Path('xarm6_interface/utils/box_data')
    saved_masks_paths = list(save_dir.glob('mask_*.npy'))
    saved_images_paths = list(save_dir.glob('color_*.png'))
    saved_depths_paths = list(save_dir.glob('depth_*.npy'))

    camera_K = np.load('xarm6_interface/calib/K.npy')
    camera_d = np.load('xarm6_interface/calib/d.npy')
    camera_d = camera_d[:, :8]

    valid_object_poses_path = save_dir / 'valid_frame_object_X.npy'
    valid_object_poses = np.load(valid_object_poses_path)  # shape: (n_valid_frame, 4, 4)

    # Prepare a list to hold all points and colors
    all_points = []
    all_colors = []

    # Iterate over all valid frames
    for i, mask_path in enumerate(saved_masks_paths):
        mask_file_name = mask_path.name
        img_file_name = mask_file_name.replace('mask_', 'color_').replace('.npy', '')
        depth_file_name = mask_file_name.replace('mask_', 'depth_').replace('.png', '')
        mask = np.load(mask_path)
        image = cv2.imread(str(save_dir / img_file_name))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        depth = np.load(str(save_dir / depth_file_name))

        # Get the corresponding object pose
        object_pose = valid_object_poses[i]

        # Generate colored point cloud for this frame
        points, colors = create_colored_point_cloud(image, depth, camera_K, mask)

        # Transform the points to the object's world frame
        points = transform_points(points, object_pose)

        # Accumulate the points and colors
        all_points.append(points)
        all_colors.append(colors)

    # Concatenate all points and colors
    all_points = np.vstack(all_points)
    all_colors = np.vstack(all_colors) / 255.0  # Normalize colors to [0, 1]

    # Create Open3D point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(all_points)
    point_cloud.colors = o3d.utility.Vector3dVector(all_colors)

    # Save the point cloud to a .ply file
    o3d.io.write_point_cloud(str(save_dir / 'fused_colored_point_cloud.ply'), point_cloud)

    lgr.info("Point cloud has been saved to: {}", str(save_dir / 'fused_colored_point_cloud.ply'))
