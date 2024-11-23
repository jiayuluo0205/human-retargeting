
from xarm6_interface.utils.realsense import build_rs_cam, MultiRealsense, get_masked_pointcloud
from pathlib import Path
import viser 
from scipy.spatial.transform import Rotation as R
import open3d as o3d

from xarm6_interface import SAM_TYPE, SAM_PATH
from xarm6_interface.utils.sam_prompt_drawer_old import SAMPromptDrawer
from xarm6_interface.utils.gpis import gpis_fit
import cv2
import numpy as np

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

def shrink_mask(mask_np, shrink_coefficient=0.95):
    """
    Shrink the edges of a binary mask by a specified coefficient.
    
    Args:
    - mask_np (numpy.ndarray): Input binary mask (1 for mask, 0 for background).
    - shrink_coefficient (float): The coefficient by which to shrink the mask (0 < coefficient < 1).
    
    Returns:
    - shrunk_mask (numpy.ndarray): The shrunk binary mask.
    """
    # Convert mask to uint8 type (required for OpenCV operations)
    mask_uint8 = mask_np.astype(np.uint8) * 255

    # Find contours in the binary mask
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Calculate the moments to find the centroid
    M = cv2.moments(mask_uint8)
    if M["m00"] == 0:
        # Handle cases where mask area is zero
        return mask_np
    
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    centroid = np.array([cx, cy])

    # Create an empty mask for the shrunk contour
    shrunk_mask = np.zeros_like(mask_uint8)

    # Iterate through contours and shrink them
    for contour in contours:
        # Calculate new contour points by moving towards the centroid
        shrunk_contour = np.array([
            shrink_coefficient * (point[0] - centroid) + centroid for point in contour
        ], dtype=np.int32)
        
        # Draw the shrunk contours on the empty mask
        cv2.drawContours(shrunk_mask, [shrunk_contour], -1, 255, thickness=cv2.FILLED)
    
    # Convert back to binary format (0 and 1)
    shrunk_mask = (shrunk_mask > 0).astype(np.uint8)
    
    return shrunk_mask

if __name__ == "__main__":
    prompt_drawer = SAMPromptDrawer(window_name="Prompt Drawer", screen_scale=2.0, sam_checkpoint=SAM_PATH, device="cuda", model_type=SAM_TYPE)
    
    ''' multi realsense setup '''
    arm_left_cam_serial = "147122075879"
    arm_right_cam_serial = "241122074374"
    camera_serial_nums = [arm_left_cam_serial, arm_right_cam_serial]
    multi_rs = MultiRealsense(camera_serial_nums)
    arm_left_cam_K_path = Path("third_party/xarm6/data/camera/mounted_black/K.npy") 
    arm_left_cam_X_BaseCamera_path = Path("third_party/xarm6/data/camera/mounted_black/0914_excalib_capture1/optimized_X_BaseCamera.npy")
    arm_left_cam_K = np.load(arm_left_cam_K_path)
    arm_left_cam_X_BaseCamera = np.load(arm_left_cam_X_BaseCamera_path)
    arm_right_cam_K_path = Path("third_party/xarm6/data/camera/mounted_white/K.npy")
    arm_right_cam_X_BaseCamera_path = Path("third_party/xarm6/data/camera/mounted_white/0914_excalib_capture3/optimized_X_BaseCamera.npy")
    arm_right_cam_K = np.load(arm_right_cam_K_path)
    arm_right_cam_X_BaseCamera = np.load(arm_right_cam_X_BaseCamera_path)
    multi_rs.set_intrinsics(0, arm_left_cam_K[0, 0], arm_left_cam_K[1, 1], arm_left_cam_K[0, 2], arm_left_cam_K[1, 2])
    multi_rs.set_intrinsics(1, arm_right_cam_K[0, 0], arm_right_cam_K[1, 1], arm_right_cam_K[0, 2], arm_right_cam_K[1, 2])
    camera_wxyzs = [
        R.from_matrix(arm_left_cam_X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]],
        R.from_matrix(arm_right_cam_X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]],
    ]
    camera_positions = [arm_left_cam_X_BaseCamera[:3, 3],
                          arm_right_cam_X_BaseCamera[:3, 3]]
    X_BaseCamera_list = [arm_left_cam_X_BaseCamera, arm_right_cam_X_BaseCamera]
    
    ''' seg '''
    # stablize the camera
    for i in range(50):
        multi_rs.getCurrentData()
    
    rtr_dict_list = multi_rs.getCurrentData()
    input_boxes_list = []
    rgb_list = []
    mask_np_list = []
    shrink_mask_list = []
    masked_pc_o3d_list = []
    for camera_idx in range(len(rtr_dict_list)):
        rtr_dict = rtr_dict_list[camera_idx]
        rgb = rtr_dict["rgb"]
        depth = rtr_dict["depth"]
        pc_o3d = rtr_dict["pointcloud_o3d"]
        prompt_drawer.reset()
        mask_np = prompt_drawer.run(rgb)
        shrink_mask_np = shrink_mask(mask_np, shrink_coefficient=0.8)
        rgb_list.append(rgb)
            
        h, w = mask_np.shape[-2:]
        mask_np_list.append(mask_np)
        shrink_mask_list.append(shrink_mask_np)
        masked_pc_o3d = get_masked_pointcloud(rgb, depth, shrink_mask_np.reshape(h,w), multi_rs.camera_data[camera_idx]["pinhole_camera_intrinsic"])
        X_BaseCamera = X_BaseCamera_list[camera_idx]
        rs_pc_in_C_np = np.asarray(masked_pc_o3d.points) 
        rs_pc_in_B = X_BaseCamera @ np.vstack([rs_pc_in_C_np.T, np.ones(rs_pc_in_C_np.shape[0])])
        rs_pc_in_B = rs_pc_in_B[:3].T
        masked_pc_o3d_W = o3d.geometry.PointCloud()
        masked_pc_o3d_W.points = o3d.utility.Vector3dVector(rs_pc_in_B)
        masked_pc_o3d_W.colors = masked_pc_o3d.colors
        masked_pc_o3d_list.append(masked_pc_o3d_W)
        
    # fusion the masked pc
    masked_pc_o3d_fusion = o3d.geometry.PointCloud()
    for masked_pc_o3d in masked_pc_o3d_list:
        masked_pc_o3d_fusion += masked_pc_o3d
    
    masked_pc_o3d_fusion = remove_outliers(masked_pc_o3d_fusion, z_min_thresh=0.02)    
    
    
    gpis, fitted_pcd = gpis_fit(masked_pc_o3d_fusion,)
    
    ''' get frames and show '''
    sv = viser.ViserServer()


    while True:
        rtr_dict_list = multi_rs.getCurrentData(pointcloud=True)
        for camera_idx in range(len(rtr_dict_list)):
            rtr_dict = rtr_dict_list[camera_idx]
            
            rs_rgb = rtr_dict["rgb"]
            rs_pc = rtr_dict["pointcloud_o3d"]
            
            camera_wxyz = camera_wxyzs[camera_idx]
            camera_pos = camera_positions[camera_idx]

            sv.scene.add_camera_frustum(f"rs_camera_img_{camera_idx}", fov=rtr_dict["fov_x"], aspect=rtr_dict["aspect_ratio"], wxyz=camera_wxyz, position=camera_pos, image=rs_rgb, scale=0.2)
            shrink_mask_image = np.concatenate([shrink_mask_list[camera_idx].reshape(h, w, 1) * 255] * 3, axis=-1).astype(np.uint8)
            sv.scene.add_camera_frustum(f"rs_camera_seg_{camera_idx}_shrink", fov=rtr_dict["fov_x"], aspect=rtr_dict["aspect_ratio"], wxyz=camera_wxyz, position=camera_pos, image=shrink_mask_image, scale=0.2)
            mask_image = np.concatenate([mask_np_list[camera_idx].reshape(h, w, 1) * 255] * 3, axis=-1).astype(np.uint8)
            sv.scene.add_camera_frustum(f"rs_camera_seg_{camera_idx}", fov=rtr_dict["fov_x"], aspect=rtr_dict["aspect_ratio"], wxyz=camera_wxyz, position=camera_pos, image=mask_image, scale=0.2)
            
            X_BaseCamera = X_BaseCamera_list[camera_idx]
            rs_pc_in_C_np = np.asarray(rs_pc.points) 
            rs_pc_in_B = X_BaseCamera @ np.vstack([rs_pc_in_C_np.T, np.ones(rs_pc_in_C_np.shape[0])])
            rs_pc_in_B = rs_pc_in_B[:3].T
            sv.scene.add_point_cloud(f"rs_camera_pc_{camera_idx}", rs_pc_in_B, colors=np.asarray(rs_pc.colors), point_size=0.001, point_shape="circle")

        sv.scene.add_point_cloud("masked_pc_o3d_fusion", np.asarray(masked_pc_o3d_fusion.points), colors=np.asarray(masked_pc_o3d_fusion.colors), point_size=0.001, point_shape="circle")
        sv.scene.add_point_cloud("gpis", np.asarray(fitted_pcd.points), colors=np.asarray(fitted_pcd.colors), point_size=0.001, point_shape="circle")
        
        
        