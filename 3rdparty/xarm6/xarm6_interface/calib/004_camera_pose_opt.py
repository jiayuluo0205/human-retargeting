import cv2
import time
import torch
import viser 
import sapien 
import numpy as np 
import open3d as o3d
import matplotlib.pyplot as plt
from tqdm import tqdm
from loguru import logger as lgr
from pathlib import Path
from enum import Enum, auto
from scipy.spatial.transform import Rotation as R
from xarm6_interface.arm_pk import XArm6WOEE
from xarm6_interface.arm_rw import XArm6RealWorld
from xarm6_interface.arm_mplib import XARM6PlannerCfg, XARM6Planner
from xarm6_interface.envs.table_and_workspace_pc import WoodenTableMount, create_bounding_box_pc, create_plane_pc, env_pc_post_process
from xarm6_interface.utils import as_mesh, NVDiffrastRenderer, vis_robot_frames, Realsense, convert_camera_pose_z_forward_to_sapien, add_noise_to_transform, robust_compute_rotation_matrix_from_ortho6d 

import os
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

if __name__ == '__main__':
    arm = XArm6WOEE()

    # load the collected data
    serial_number= "241122074374" # "241122074374", "233622079809"
    exp_name = "0223_excalib_capture00_front"
    K_path = Path(__file__).resolve().parent.parent.parent / "data" / "camera" / serial_number / "K.npy"
    K = np.load(K_path)
    save_data_rel_dir_path = (Path(os.path.join(ROOT_DIR, f"3rdparty/xarm6/data/camera/{serial_number}")) / exp_name).resolve()
    # init_X_BaseCamera_path = (Path("data") / exp_name / "init_X_BaseCamera.npy").resolve()
    save_data_path = K_path.parent
    init_X_BaseCamera_path = save_data_path / "init_X_BaseCamera.npy"
    
    init_X_BaseCamera = np.load(init_X_BaseCamera_path)
    init_X_CameraBase = np.linalg.inv(init_X_BaseCamera)
    sample_id_paths = list(save_data_rel_dir_path.glob("*"))
    sample_id_paths = sorted(sample_id_paths)
    sample_id_paths = [p for p in sample_id_paths if p.is_dir()]

    sample_img_paths = [p / "rgb_image.jpg" for p in sample_id_paths]
    sample_mask_paths = [p / "mask.npy" for p in sample_id_paths]
    sample_pointcloud_paths = [p / "point_cloud.ply" for p in sample_id_paths]

    dataset_rw_joint_values = []
    dataset_rw_arm_visual_mesh = []
    dataset_rw_masks = []
    dataset_rw_imgs = []
    for sample_id_path, sample_img_path, sample_mask_path, sample_pc_path in zip(sample_id_paths, sample_img_paths, sample_mask_paths, sample_pointcloud_paths):
        rw_joint_values = np.load(sample_id_path / "joint_values.npy")
        rw_arm_visual_mesh = as_mesh(arm.get_state_trimesh(rw_joint_values)['visual'])
        rw_bg_arm_mask = np.load(sample_mask_path)
        rw_rgb_img = cv2.imread(str(sample_img_path))
        rw_rgb_img = cv2.cvtColor(rw_rgb_img, cv2.COLOR_BGR2RGB)
        dataset_rw_joint_values.append(rw_joint_values)
        dataset_rw_arm_visual_mesh.append(rw_arm_visual_mesh)
        dataset_rw_masks.append(rw_bg_arm_mask)
        dataset_rw_imgs.append(rw_rgb_img)

    # setup camera 
    camera = Realsense()
    camera.set_intrinsics(K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    H, W = camera.h, camera.w
    K = camera.K

    ''' setup the digital twin xarm and the mask renderer'''
    renderer = NVDiffrastRenderer([H, W])

    cam_pose_params = [init_X_CameraBase[:3, 3], init_X_CameraBase[:3, 0], init_X_CameraBase[:3, 1]]
    cam_pose_params = torch.from_numpy(np.array(cam_pose_params).flatten()).cuda().float()
    cam_pose_params.requires_grad = True
    optimizer = torch.optim.Adam([cam_pose_params], lr=3e-3)

    ''' begin the optimization '''
    cam_pose_during_opt = []
    for epoch in tqdm(range(200)):
        rendered_masks = []
        cam_pose_during_opt.append(cam_pose_params.detach().cpu().numpy())
        for sample_id, rw_joint_values, rw_arm_visual_mesh, rw_bg_arm_mask, rw_rgba_img in zip(range(len(dataset_rw_joint_values)), dataset_rw_joint_values, dataset_rw_arm_visual_mesh, dataset_rw_masks, dataset_rw_imgs):
            cam_position = cam_pose_params[:3]  # [3]
            cam_6d = cam_pose_params[3:]
            cam_rot = robust_compute_rotation_matrix_from_ortho6d(cam_6d.unsqueeze(0)).squeeze(0)  # [3, 3]
            current_X_CameraBase = torch.concat([cam_rot, cam_position.unsqueeze(1)], dim=1)  # [3, 4]
            current_X_CameraBase = torch.concat([current_X_CameraBase, torch.tensor([[0, 0, 0, 1]]).cuda().float()], dim=0)  # [4, 4]
            
            mask = renderer.render_mask(torch.from_numpy(rw_arm_visual_mesh.vertices).cuda().float(),
                                torch.from_numpy(rw_arm_visual_mesh.faces).cuda().int(),
                                torch.from_numpy(K).cuda().float(),
                                current_X_CameraBase
                            )
            rendered_masks.append(mask)
        rendered_masks = torch.stack(rendered_masks, dim=0)  # [N, H, W]
        diff_mask = torch.abs(rendered_masks - torch.from_numpy(np.stack(dataset_rw_masks, axis=0)).cuda().float())
        loss = diff_mask.mean()
        lgr.info(f"Epoch {epoch}: Loss: {loss.item()}")
        if loss.item() < 1e-3:
            break
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        
    lgr.info("Optimization done!")
    cam_pose_during_opt_np = np.array(cam_pose_during_opt)  # [N, 9]
    cam_pose_during_opt_torch = torch.from_numpy(cam_pose_during_opt_np).cuda().float()  # [N, 9]
    cam_position_during_opt_torch = cam_pose_during_opt_torch[:, :3]  # [N, 3]
    cam_6d_during_opt_torch = cam_pose_during_opt_torch[:, 3:]  # [N, 6]
    cam_rot_during_opt_torch = robust_compute_rotation_matrix_from_ortho6d(cam_6d_during_opt_torch)  # [B, 3, 3]
    cam_pose_during_opt_torch = torch.cat([cam_rot_during_opt_torch, cam_position_during_opt_torch.unsqueeze(-1)], dim=-1)  # [B, 3, 4]
    cam_pose_during_opt_torch = torch.cat([cam_pose_during_opt_torch, torch.tensor([[[0, 0, 0, 1]]]).cuda().float().repeat(cam_pose_during_opt_torch.shape[0], 1, 1)], dim=1)  # [B, 4, 4]
    cam_pose_during_opt_np = cam_pose_during_opt_torch.detach().cpu().numpy()  # [B, 4, 4]
    X_BaseCamera_during_opt = np.linalg.inv(cam_pose_during_opt_np)  # [B, 4, 4]
    X_BaseCamera_final = X_BaseCamera_during_opt[-1]  # [4, 4]
    rendered_masks_np = rendered_masks.detach().cpu().numpy()  # [N, H, W]

    lgr.info("X_BaseCamera_final: \n{}".format(X_BaseCamera_final))

    ''' setup the viser server '''
    sv = viser.ViserServer()

    # add initial camera pose, gt camera pose, and optimized camera pose
    sv.scene.add_camera_frustum("init_camera", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=R.from_matrix(init_X_BaseCamera[0:3, 0:3]).as_quat()[[3, 0, 1, 2]], position=init_X_BaseCamera[0:3, 3], image=None, scale=0.2, color=[255, 0, 0])
    cam_slider = sv.gui.add_slider("cam_pose_slider", 0, len(cam_pose_during_opt_np)-1, step=1, initial_value=0)
    def update_cam_pose(cam_pose_id):
        cam_pose = X_BaseCamera_during_opt[cam_pose_id]
        sv.scene.add_camera_frustum("optimized_camera", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=R.from_matrix(cam_pose[0:3, 0:3]).as_quat()[[3, 0, 1, 2]], position=cam_pose[0:3, 3], image=None, scale=0.2, color=[0, 0, 255])
    cam_slider.on_update(lambda _: update_cam_pose(cam_slider.value))
    
    dataset_slider = sv.gui.add_slider("dataset_slider", 0, len(dataset_rw_joint_values)-1, step=1, initial_value=0)
    def update_dataset(dataset_id):
        rw_arm_visual_mesh = dataset_rw_arm_visual_mesh[dataset_id]
        rw_bg_arm_mask = dataset_rw_masks[dataset_id]
        rw_rgb_img = dataset_rw_imgs[dataset_id]
        sv.scene.add_mesh_simple("rw_arm", vertices=rw_arm_visual_mesh.vertices, faces=rw_arm_visual_mesh.faces, color=[0.7, 0.7, 0.7], opacity=0.5)
        
        camera_wxyz = R.from_matrix(X_BaseCamera_final[0:3, 0:3]).as_quat()[[3, 0, 1, 2]]
        camera_pos = X_BaseCamera_final[0:3, 3]

        mask_np = np.stack([rw_bg_arm_mask * 255]*3, axis=-1).astype(np.uint8) 
        rendered_mask = rendered_masks_np[dataset_id]
        rendered_mask = np.stack([rendered_mask* 255]*3, axis=-1).astype(np.uint8) 
        sv.scene.add_camera_frustum("rw_rgb_img", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rw_rgb_img, scale=0.2)
        sv.scene.add_camera_frustum("rw_bg_arm_mask", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=mask_np, scale=0.2)
        sv.scene.add_camera_frustum("rw_rendered_mask", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rendered_mask, scale=0.2)

    dataset_slider.on_update(lambda _: update_dataset(dataset_slider.value))
    update_dataset(0)
    save_button = sv.gui.add_button("save_camera_pose",)
    save_button.on_click(
        lambda _: np.save(save_data_rel_dir_path / "optimized_X_BaseCamera.npy", X_BaseCamera_final)
    )

    while True:
        time.sleep(1)
