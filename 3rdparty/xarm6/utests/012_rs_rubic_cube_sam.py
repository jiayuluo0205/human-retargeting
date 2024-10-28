import time 
import copy
import torch
import viser 
import numpy as np
import open3d as o3d
from pathlib import Path
import matplotlib.pyplot as plt
from xarm6_interface.utils.realsense import build_rs_cam, get_masked_pointcloud
from xarm6_interface.utils.sam2_utils import grounding_dino_get_bbox, build_sam2_camera_predictor, show_bbox, show_mask
from xarm6_interface import SAM2_CHECKPOINT, SAM2_MODEL_CONFIG
from loguru import logger as lgr
from scipy.spatial.transform import Rotation as R

rgb = None 
depth = None
colored_depth = None
pc_o3d = None
rgbs = []
depths = []
cloud_base = o3d.geometry.PointCloud()
X_ObjectCloud1 = np.identity(4)

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def pc_registration(source, target):
    voxel_radius = [0.005, 0.002, 0.001, 0.0005]
    max_iter = [50, 30, 15, 15]
    current_transformation = np.identity(4)
    
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        source_down = source.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        # result_icp = o3d.pipelines.registration.registration_colored_icp(
        #     source_down, target_down, radius, current_transformation,
        #     o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        #     o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
        #                                                     relative_rmse=1e-6,
        #                                                     max_iteration=iter))
        voxel_size = voxel_radius[scale]
        result_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down, voxel_size * 1.5,
            current_transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iter))
        current_transformation = result_icp.transformation

    return np.asarray(result_icp.transformation)

def tf_filter(tf, trans_tol=0.5, rotation_tol=30):
    rot_mat = tf[:3, :3].copy()  # Make a copy to ensure it is writable
    rot_axisang = R.from_matrix(rot_mat).as_rotvec()
    rot_theta = np.linalg.norm(rot_axisang)
    rot_theta_degree = np.rad2deg(rot_theta)
    is_good_result = True

    if (tf[0, 3] > trans_tol or tf[0, 3] < -trans_tol or 
            tf[1, 3] > trans_tol or tf[1, 3] < -trans_tol or 
            tf[2, 3] > trans_tol or tf[2, 3] < -trans_tol):
        is_good_result = False
        lgr.warning('Something wrong with 1/ translation : (, turn back a little bit...')
        lgr.warning('>> the translation is [{},{},{}]'.format(tf[0, 3], tf[1, 3], tf[2, 3]))

    elif (rot_theta_degree > rotation_tol or 
          rot_theta_degree < -rotation_tol):
        is_good_result = False
        lgr.warning('Something wrong with 2/ rotation : (, turn back a little bit...')
        lgr.warning('>> the rotation is {} (in degrees)'.format(rot_theta_degree))

    return is_good_result
    
            
if __name__ == "__main__":
    cam_rt_dict = build_rs_cam(
        cam_K_path = Path(__file__).resolve().parent.parent / "data" / "camera" / "mounted_rs" / "K.npy",
        X_BaseCamera_path = Path(__file__).resolve().parent.parent / "data" / "camera" / "mounted_rs" / "X_BaseCamera.npy",
    )
    cam = cam_rt_dict["cam"]
    cam_wxyz = cam_rt_dict["cam_wxyz"]
    cam_position = cam_rt_dict["cam_position"]
    X_BaseCamera = cam_rt_dict["X_BaseCamera"]
    
    ''' the recon task '''
    text_prompt = "a rubic cube on the table."
    sv = viser.ViserServer()
    
    ''' stablize the camera '''
    for i in range(50):
        cam.getCurrentData()
    ''' get the bbox for track '''
    rtr_dict = cam.getCurrentData()
    rgb = rtr_dict["rgb"]
    depth = rtr_dict["depth"]
    colored_depth = rtr_dict["colored_depth"]
    pc_o3d = rtr_dict["pointcloud_o3d"]
    input_boxes = grounding_dino_get_bbox(rgb, text_prompt)
    if input_boxes.shape != (4,):
        input_boxes = input_boxes[0].reshape(2, 2)
    # build the sam 2 model
    torch.autocast(device_type="cuda", dtype=torch.bfloat16).__enter__()
    if torch.cuda.get_device_properties(0).major >= 8:
        # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
    predictor = build_sam2_camera_predictor(SAM2_MODEL_CONFIG, SAM2_CHECKPOINT)
    predictor.load_first_frame(rgb)
    ann_frame_idx = 0 
    ann_obj_id = 1
    _, out_obj_ids, out_mask_logits = predictor.add_new_prompt(
        frame_idx=ann_frame_idx, obj_id=ann_obj_id, bbox=input_boxes.reshape(2, 2)
    )
    mask_np = (out_mask_logits[0] > 0.0).cpu().numpy()
    h, w = mask_np.shape[-2:]
    masked_pc = get_masked_pointcloud(rgb, depth, mask_np.reshape(h,w), cam.pinhole_camera_intrinsic)
    cloud_base += masked_pc
    cloud1 = copy.deepcopy(cloud_base)
    
    plt.gca().imshow(rgb)
    show_bbox(input_boxes.reshape(2, 2), plt.gca())
    show_mask((out_mask_logits[0] > 0.0).cpu().numpy(), plt.gca(), obj_id=out_obj_ids[0])
    plt.show()
    
    cmap = plt.get_cmap("tab10")
    mask_color = np.array([*cmap(0)[:3], 0.6])
    
    while True: 
        rtr_dict = cam.getCurrentData()
        rgb = rtr_dict["rgb"]
        depth = rtr_dict["depth"]
        colored_depth = rtr_dict["colored_depth"]
        pc_o3d = rtr_dict["pointcloud_o3d"]
        
        ''' sam tracking and mask pc '''
        ann_frame_idx += 1
        out_obj_ids, out_mask_logits = predictor.track(rgb)
        mask_np = (out_mask_logits[0] > 0.0).cpu().numpy()
        h, w = mask_np.shape[-2:]
        mask_image = mask_np.reshape(h, w, 1) * mask_color.reshape(1, 1, -1)
        cloud2 = get_masked_pointcloud(rgb, depth, mask_np.reshape(h,w), cam.pinhole_camera_intrinsic)
        
        sv.scene.add_point_cloud("cloud1", points=np.asarray(cloud1.points), colors=cloud1.colors, point_shape="circle", point_size=0.003)
        sv.scene.add_point_cloud("cloud2", points=np.asarray(cloud2.points), colors=cloud2.colors, point_shape="circle", point_size=0.003)
        
        ''' begin registration '''
        X_Cloud1Cloud2 = pc_registration(cloud1, cloud2)
        is_good_result = tf_filter(X_Cloud1Cloud2)
        if is_good_result:
            X_ObjectCloud1 = X_ObjectCloud1 @ np.linalg.inv(X_Cloud1Cloud2)
            cloud1 = copy.deepcopy(cloud2)
            cloud2.transform(X_ObjectCloud1)
            cloud_base += cloud2
            cloud_base.voxel_down_sample(0.001)
            
        sv.scene.add_camera_frustum(
            "cam_rgb", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=rgb, wxyz=cam_wxyz, position=cam_position
        )
        sv.scene.add_camera_frustum(
            "cam_depth", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=colored_depth,  wxyz=cam_wxyz, position=cam_position
        )
        sv.scene.add_camera_frustum(
            "cam_mask", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=mask_image, wxyz=cam_wxyz, position=cam_position, format="png"
        )
        
        cam_points = np.asarray(pc_o3d.points)
        world_points = np.dot(X_BaseCamera, np.concatenate([cam_points.T, np.ones((1, cam_points.shape[0]))], axis=0))[0:3].T
        cam_masked_points = np.asarray(masked_pc.points)
        world_masked_points = np.dot(X_BaseCamera, np.concatenate([cam_masked_points.T, np.ones((1, cam_masked_points.shape[0]))], axis=0))[0:3].T
        cam_object_points = np.asarray(cloud_base.points)
        world_object_points = np.dot(X_BaseCamera, np.concatenate([cam_object_points.T, np.ones((1, cam_object_points.shape[0]))], axis=0))[0:3].T
        
        lgr.info(f"before: {cam_points.shape}, after: {cam_masked_points.shape}")
        sv.scene.add_point_cloud("pc", points=world_points, colors=pc_o3d.colors, point_shape="circle", point_size=0.003)
        sv.scene.add_point_cloud("pc_masked", points=world_masked_points, colors=masked_pc.colors, point_shape="circle", point_size=0.003)
        sv.scene.add_point_cloud("pc_object", points=world_object_points, colors=cloud_base.colors, point_shape="circle", point_size=0.003)
        
        time.sleep(0.005)
        