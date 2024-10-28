import time
import torch
import viser 
import sapien 
import numpy as np 
import nvdiffrast.torch as dr
from loguru import logger as lgr
from PIL import Image, ImageColor
from scipy.spatial.transform import Rotation as R
from xarm6_interface.arm_pk import XArm6WOEE
from xarm6_interface.arm_mplib import XARM6PlannerCfg, XARM6Planner
from xarm6_interface.envs.table_and_workspace_pc import WoodenTableMount, create_bounding_box_pc, create_plane_pc, env_pc_post_process
from xarm6_interface.utils import as_mesh, NVDiffrastRenderer, vis_robot_frames, Realsense, convert_camera_pose_z_forward_to_sapien



if __name__ == '__main__':
    arm = XArm6WOEE()
    sv = viser.ViserServer()
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

    arm_visual_mesh = as_mesh(arm.get_state_trimesh(arm.reference_joint_values)['visual'])
    renderer = NVDiffrastRenderer([H, W])

    ''' setup xarm '''  # the `real world` xarm6 robot
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


    ''' setup env '''
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
    sv.scene.add_point_cloud("env_pc", env_pc, colors=[200, 50, 50], point_size=0.005)
    sample_slider = sv.gui.add_slider("sample_slider", 0, n_valid_samples-1, step=1, initial_value=0)

    joint_gui_handles = []
    def update_robot_trimesh_camera_and_mask(joint_values):
        xarm6_planner.sp_robot.set_qpos(joint_values)
        xarm6_planner.scene.update_render()  # sync pose from SAPIEN to renderer
        sp_camera.take_picture()  # submit rendering jobs to the GPU
        rgba = sp_camera.get_picture("Color")  # [H, W, 4]
        rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
        seg_labels = sp_camera.get_picture("Segmentation")  # [H, W, 4]
        actor_seg = seg_labels[..., 1].astype(np.uint8)  # actor-level
        colormap = sorted(set(ImageColor.colormap.values()))
        color_palette = np.array(
            [ImageColor.getrgb(color) for color in colormap], dtype=np.uint8
        )
        actor_and_bg_seg = actor_seg != 1  
        actor_and_bg_seg_img = actor_and_bg_seg.astype(np.uint8) * 255
        actor_and_bg_seg_img = np.stack([actor_and_bg_seg_img]*3, axis=-1)

        trimesh_dict = arm.get_state_trimesh(
            joint_values,
            visual=True,
            collision=True,
        )
        visual_mesh = trimesh_dict["visual"]
        collision_mesh = trimesh_dict["collision"]
        sv.scene.add_mesh_trimesh("visual_mesh", visual_mesh)
        sv.scene.add_mesh_trimesh("collision_mesh", collision_mesh)
        vis_robot_frames(sv, arm.current_status, axes_length=0.15, axes_radius=0.005)

        arm_visual_mesh = as_mesh(visual_mesh)
        mask = renderer.render_mask(torch.from_numpy(arm_visual_mesh.vertices).cuda().float(),
                                    torch.from_numpy(arm_visual_mesh.faces).cuda().int(),
                                    torch.from_numpy(K).cuda().float(),
                                    torch.from_numpy(X_CameraBase).cuda().float())
        colored_mask_np = np.stack([mask.cpu().numpy()]*3, axis=-1)
        camera_wxyz = R.from_matrix(X_BaseCamera[0:3, 0:3]).as_quat()[[3, 0, 1, 2]]
        camera_pos = X_BaseCamera[0:3, 3]
        sv.scene.add_camera_frustum("camera", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=colored_mask_np, scale=0.2)
        sv.scene.add_camera_frustum("sp_camera_img", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rgba_img, scale=0.2)
        sv.scene.add_camera_frustum("sp_camera_seg", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=color_palette[actor_seg], scale=0.2)
        sv.scene.add_camera_frustum("sp_camera_actor_and_bg_seg", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=actor_and_bg_seg_img, scale=0.2)

    for joint_name, lower, upper, initial_angle in zip(
        arm.actuated_joint_names, arm.lower_joint_limits_np, arm.upper_joint_limits_np, arm.reference_joint_values_np
    ):
        lower = float(lower) if lower is not None else -np.pi
        upper = float(upper) if upper is not None else np.pi
        slider = sv.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=0.05,
            initial_value=float(initial_angle),
        )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: update_robot_trimesh_camera_and_mask([gui.value for gui in joint_gui_handles])
        )
        joint_gui_handles.append(slider)

    sample_slider.on_update(
        lambda _: update_robot_trimesh_camera_and_mask(sampled_joint_values[sample_slider.value])
    )
    update_robot_trimesh_camera_and_mask(arm.reference_joint_values_np)

    while True:
        time.sleep(1)
