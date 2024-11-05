import time
import torch
import viser 
import sapien 
import numpy as np 
from tqdm import tqdm
from loguru import logger as lgr
from PIL import Image, ImageColor
from scipy.spatial.transform import Rotation as R
from xarm6_interface.arm_pk import XArm6WOEE
from xarm6_interface.arm_mplib import XARM6PlannerCfg, XARM6Planner
from xarm6_interface.envs.table_and_workspace_pc import WoodenTableMount, create_bounding_box_pc, create_plane_pc, env_pc_post_process
from xarm6_interface.utils import as_mesh, NVDiffrastRenderer, vis_robot_frames, Realsense, convert_camera_pose_z_forward_to_sapien, add_noise_to_transform, robust_compute_rotation_matrix_from_ortho6d 

if __name__ == '__main__':
    ''' the test '''
    # setup camera 
    camera = Realsense()
    H, W = camera.h, camera.w
    K = camera.K
    # handcraft a transformation matrix
    X_BaseCamera = np.eye(4)
    X_BaseCamera[0:3, 0:3] = R.from_euler('x', -90-65, degrees=True).as_matrix()
    X_BaseCamera[0:3, 3] = np.array([0.5, -0.5, 1.1])
    X_CameraBase = np.linalg.inv(X_BaseCamera)
    lgr.info("X_BaseCamera: \n{}".format(X_BaseCamera))
    lgr.info("X_CameraBase: \n{}".format(X_CameraBase))

    rotation_noise_angle_std = 20/180*np.pi
    translation_noise_std = 0.2
    gt_X_CameraBase = X_CameraBase.copy()
    gt_X_BaseCamera = np.linalg.inv(gt_X_CameraBase)
    init_X_CameraBase = add_noise_to_transform(gt_X_CameraBase, rotation_noise_angle_std=rotation_noise_angle_std, translation_noise_std=translation_noise_std)
    init_X_BaseCamera = np.linalg.inv(init_X_CameraBase)
    lgr.info("init_X_CameraBase: \n{}".format(init_X_CameraBase))
    lgr.info("gt_X_CameraBase: \n{}".format(gt_X_CameraBase))


    ''' setup xarm and the real world camera '''  # the `real world` xarm6 robot
    xarm6_planner_cfg = XARM6PlannerCfg(vis=False, n_env_pc=10000)
    xarm6_planner = XARM6Planner(xarm6_planner_cfg)
    sp_camera = xarm6_planner.scene.add_camera(
            name="rw_camera",
            width=W,
            height=H,
            fovy=camera.fov_y,
            near=0.001,
            far=10.0,
        )
    sapien_cam_pose = convert_camera_pose_z_forward_to_sapien(X_BaseCamera)
    sp_camera.entity.set_pose(sapien.Pose(sapien_cam_pose))
    sp_camera.set_focal_lengths(camera.intr.fx, camera.intr.fy)
    sp_camera.set_principal_point(camera.intr.ppx, camera.intr.ppy)
    env_params = WoodenTableMount()
    workspace_pc = create_bounding_box_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    table_plane_pc = create_plane_pc(env_params.table_plane_xmin, env_params.table_plane_ymin, env_params.table_plane_zmin, env_params.table_plane_xmax, env_params.table_plane_ymax, env_params.table_plane_zmax, xarm6_planner_cfg.n_env_pc)
    workspace_xmin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmin, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymin, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymax_pc = create_plane_pc(env_params.xmin, env_params.ymax, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    env_pc = np.concatenate([workspace_pc, table_plane_pc, workspace_xmin_pc, workspace_ymin_pc, workspace_ymax_pc], axis=0)
    env_pc = env_pc_post_process(env_pc, filter_norm_thresh=0.1, n_save_pc=None)
    xarm6_planner.mplib_add_point_cloud(env_pc)
    sampled_joint_values = xarm6_planner.mplib_sample_joint_values(1000, collision_check=True)
    n_valid_samples = len(sampled_joint_values)

    ''' setup the digital twin xarm and the mask renderer'''
    arm = XArm6WOEE()
    renderer = NVDiffrastRenderer([H, W])

    dataset_rw_joint_values = []
    dataset_rw_arm_visual_mesh = []
    dataset_rw_masks = []
    dataset_rw_imgs = []

    cam_pose_params = [init_X_CameraBase[:3, 3], init_X_CameraBase[:3, 0], init_X_CameraBase[:3, 1]]
    cam_pose_params = torch.from_numpy(np.array(cam_pose_params).flatten()).cuda().float()
    cam_pose_params.requires_grad = True
    optimizer = torch.optim.Adam([cam_pose_params], lr=3e-3)

    ''' begin the test '''
    def data_capture(joint_values):
        current_joint_values = xarm6_planner.sp_robot.get_qpos()
        planning_result = xarm6_planner.mplib_plan_qpos(current_joint_values, [joint_values])
        is_success = planning_result['status'] == 'Success'
        lgr.info(f"Sample {sample_id}: {is_success}")
        if not is_success:
            lgr.warning(f"Sample {sample_id} failed!")
        else:
            xarm6_planner.sp_follow_path(planning_result)
        rw_joint_values = xarm6_planner.sp_robot.get_qpos()
        xarm6_planner.scene.update_render()  # sync pose from SAPIEN to renderer
        sp_camera.take_picture()  
        rw_rgba = sp_camera.get_picture("Color")  # [H, W, 4]
        rw_rgba_img = (rw_rgba * 255).clip(0, 255).astype("uint8")
        seg_labels = sp_camera.get_picture("Segmentation")  # [H, W, 4]
        actor_seg = seg_labels[..., 1].astype(np.uint8)  # actor-level
        actor_and_bg_seg = actor_seg > 1  # 0 for background, 1 for actor
        rw_bg_arm_mask = actor_and_bg_seg
        rw_arm_visual_mesh = as_mesh(arm.get_state_trimesh(rw_joint_values)['visual']) 

        return rw_joint_values, rw_arm_visual_mesh, rw_bg_arm_mask, rw_rgba_img


    for sample_id, joint_values in enumerate(sampled_joint_values[:10]):
        rw_joint_values, rw_arm_visual_mesh, rw_bg_arm_mask, rw_rgba_img = data_capture(joint_values)
        dataset_rw_joint_values.append(rw_joint_values)
        dataset_rw_arm_visual_mesh.append(rw_arm_visual_mesh)
        dataset_rw_masks.append(rw_bg_arm_mask)
        dataset_rw_imgs.append(rw_rgba_img)

    ''' begin the optimization '''
    cam_pose_during_opt = []
    for epoch in tqdm(range(1000)):
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
    lgr.info("Final cam params: \n{}".format(cam_pose_during_opt[-1]))
    cam_pose_during_opt_np = np.array(cam_pose_during_opt)  # [N, 9]
    cam_pose_during_opt_torch = torch.from_numpy(cam_pose_during_opt_np).cuda().float()  # [N, 9]
    cam_position_during_opt_torch = cam_pose_during_opt_torch[:, :3]  # [N, 3]
    cam_6d_during_opt_torch = cam_pose_during_opt_torch[:, 3:]  # [N, 6]
    cam_rot_during_opt_torch = robust_compute_rotation_matrix_from_ortho6d(cam_6d_during_opt_torch)  # [B, 3, 3]
    cam_pose_during_opt_torch = torch.cat([cam_rot_during_opt_torch, cam_position_during_opt_torch.unsqueeze(-1)], dim=-1)  # [B, 3, 4]
    cam_pose_during_opt_torch = torch.cat([cam_pose_during_opt_torch, torch.tensor([[[0, 0, 0, 1]]]).cuda().float().repeat(cam_pose_during_opt_torch.shape[0], 1, 1)], dim=1)  # [B, 4, 4]
    cam_pose_during_opt_np = cam_pose_during_opt_torch.detach().cpu().numpy()  # [B, 4, 4]
    X_BaseCamera_during_opt = np.linalg.inv(cam_pose_during_opt_np)  # [B, 4, 4]
    ''' setup the viser server '''
    sv = viser.ViserServer()

    # add initial camera pose, gt camera pose, and optimized camera pose
    sv.scene.add_camera_frustum("init_camera", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=R.from_matrix(init_X_BaseCamera[0:3, 0:3]).as_quat()[[3, 0, 1, 2]], position=init_X_BaseCamera[0:3, 3], image=None, scale=0.2, color=[255, 0, 0])
    sv.scene.add_camera_frustum("gt_camera", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=R.from_matrix(gt_X_BaseCamera[0:3, 0:3]).as_quat()[[3, 0, 1, 2]], position=gt_X_BaseCamera[0:3, 3], image=None, scale=0.2, color=[0, 255, 0])

    cam_slider = sv.gui.add_slider("cam_pose_slider", 0, len(cam_pose_during_opt_np)-1, step=1, initial_value=0)
    def update_cam_pose(cam_pose_id):
        cam_pose = X_BaseCamera_during_opt[cam_pose_id]
        sv.scene.add_camera_frustum("optimized_camera", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=R.from_matrix(cam_pose[0:3, 0:3]).as_quat()[[3, 0, 1, 2]], position=cam_pose[0:3, 3], image=None, scale=0.2, color=[0, 0, 255])
    cam_slider.on_update(lambda _: update_cam_pose(cam_slider.value))

    # sv.scene.add_point_cloud("env_pc", env_pc, colors=[200, 50, 50], point_size=0.005)
    # sample_slider = sv.gui.add_slider("sample_slider", 0, n_valid_samples-1, step=1, initial_value=0)
    # def update_robot_trimesh_camera_and_mask(joint_values):
    #     xarm6_planner.sp_robot.set_qpos(joint_values)
    #     xarm6_planner.scene.step()  # run a physical step
    #     xarm6_planner.scene.update_render()  # sync pose from SAPIEN to renderer
    #     sp_camera.take_picture()  # submit rendering jobs to the GPU
    #     rgba = sp_camera.get_picture("Color")  # [H, W, 4]
    #     rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
    #     seg_labels = sp_camera.get_picture("Segmentation")  # [H, W, 4]
    #     actor_seg = seg_labels[..., 1].astype(np.uint8)  # actor-level
    #     colormap = sorted(set(ImageColor.colormap.values()))
    #     color_palette = np.array(
    #         [ImageColor.getrgb(color) for color in colormap], dtype=np.uint8
    #     )

    #     trimesh_dict = arm.get_state_trimesh(
    #         joint_values,
    #         visual=True,
    #         collision=True,
    #     )
    #     visual_mesh = trimesh_dict["visual"]
    #     collision_mesh = trimesh_dict["collision"]
    #     sv.scene.add_mesh_trimesh("visual_mesh", visual_mesh)
    #     sv.scene.add_mesh_trimesh("collision_mesh", collision_mesh)
    #     vis_robot_frames(sv, arm.current_status, axes_length=0.15, axes_radius=0.005)

    #     arm_visual_mesh = as_mesh(visual_mesh)
    #     mask = renderer.render_mask(torch.from_numpy(arm_visual_mesh.vertices).cuda().float(),
    #                                 torch.from_numpy(arm_visual_mesh.faces).cuda().int(),
    #                                 torch.from_numpy(K).cuda().float(),
    #                                 torch.from_numpy(X_CameraBase).cuda().float())
    #     colored_mask_np = np.stack([mask.cpu().numpy()]*3, axis=-1)
    #     camera_wxyz = R.from_matrix(X_BaseCamera[0:3, 0:3]).as_quat()[[3, 0, 1, 2]]
    #     camera_pos = X_BaseCamera[0:3, 3]
    #     sv.scene.add_camera_frustum("camera", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=colored_mask_np, scale=0.2)
    #     sv.scene.add_camera_frustum("sp_camera_img", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rgba_img, scale=0.2)
    #     sv.scene.add_camera_frustum("sp_camera_seg", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=color_palette[actor_seg], scale=0.2)

    # sample_slider.on_update(
    #     lambda _: update_robot_trimesh_camera_and_mask(sampled_joint_values[sample_slider.value])
    # )
    # update_robot_trimesh_camera_and_mask(arm.reference_joint_values_np)

    while True:
        time.sleep(1)
