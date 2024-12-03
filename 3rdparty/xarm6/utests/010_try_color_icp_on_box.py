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
import copy

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

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target],
                                      zoom=0.5,
                                      front=[-0.2458, -0.8088, 0.5342],
                                      lookat=[1.7745, 2.2305, 0.9787],
                                      up=[0.3109, -0.5878, -0.7468])
    

if __name__ == "__main__":
    save_dir = Path('xarm6_interface/utils/box_data')
    saved_masks_paths = sorted(list(save_dir.glob('mask_*.npy')))
    saved_images_paths = sorted(list(save_dir.glob('color_*.png')))
    saved_depths_paths = sorted(list(save_dir.glob('depth_*.npy')))

    camera_K = np.load('xarm6_interface/calib/K.npy')
    camera_d = np.load('xarm6_interface/calib/d.npy')
    camera_d = camera_d[:, :8]

    valid_object_poses_path = save_dir / 'valid_frame_object_X.npy'
    valid_object_poses = np.load(valid_object_poses_path)  # shape: (n_valid_frame, 4, 4)

    # Prepare a list to hold all points and colors
    all_points = []
    all_colors = []
    colored_pc_o3d = []

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
        pc_o3d = o3d.geometry.PointCloud()
        pc_o3d.points = o3d.utility.Vector3dVector(points)
        pc_o3d.colors = o3d.utility.Vector3dVector(colors / 255.0)
        colored_pc_o3d.append(pc_o3d)

    source = colored_pc_o3d[0]
    target = colored_pc_o3d[1]

    X_CamSource = valid_object_poses[0]
    X_CamTarget = valid_object_poses[1]
    X_SourceTarget = np.eye(4)

    draw_registration_result_original_color(source, target, X_SourceTarget)

    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = X_SourceTarget
    print("3. Colored point cloud registration")
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)

        print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        
        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                            relative_rmse=1e-6,
                                                            max_iteration=iter))
        current_transformation = result_icp.transformation
        print(result_icp)
        print(current_transformation)
    draw_registration_result_original_color(source, target,
                                            result_icp.transformation)