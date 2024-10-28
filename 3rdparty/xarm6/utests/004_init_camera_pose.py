
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

# an initial camera pose guess
X_BaseCamera = np.eye(4)
X_BaseCamera[0:3, 0:3] = R.from_euler('x', -90-65, degrees=True).as_matrix()
X_BaseCamera[0:3, 3] = np.array([0.5, -0.5, 1.1])

if __name__ == '__main__':
    ''' the test '''
    # setup the real camera 
    camera = Realsense()
    H, W = camera.h, camera.w
    K = camera.K
    # setup the sim camera 
    arm = XArm6WOEE()
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

    sv = viser.ViserServer()
    camera_wxyz = R.from_matrix(X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]]
    camera_pos = X_BaseCamera[:3, 3]
    camera_control = sv.scene.add_transform_controls(
        f"camera_pose",
        opacity=0.75,
        disable_sliders=True,
        scale=0.25,
        line_width=2.5,
        wxyz=camera_wxyz,
        position=camera_pos,
    )

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
        actor_and_bg_seg = actor_seg > 1  
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
        camera_wxyz = R.from_matrix(X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]]
        camera_pos = X_BaseCamera[:3, 3]
        sv.scene.add_camera_frustum("sp_camera_img", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rgba_img, scale=0.2)
        sv.scene.add_camera_frustum("sp_camera_seg", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=color_palette[actor_seg], scale=0.2)
        sv.scene.add_camera_frustum("sp_camera_actor_and_bg_seg", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=actor_and_bg_seg_img, scale=0.2)

        rs_rgb, rs_depth, rs_pc = camera.getCurrentData("rgb+depth+pointcloud")
        sv.scene.add_camera_frustum("rs_camera_img", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rs_rgb, scale=0.2)
        rs_pc_in_C_np = np.asarray(rs_pc.points) 
        rs_pc_in_B = X_BaseCamera @ np.vstack([rs_pc_in_C_np.T, np.ones(rs_pc_in_C_np.shape[0])])
        rs_pc_in_B = rs_pc_in_B[:3].T
        sv.scene.add_point_cloud("rs_camera_pc", rs_pc_in_B, colors=[200, 50, 50], point_size=0.005)


    def update_camera_pose(camera_wxyz, camera_pos):
        global X_BaseCamera
        X_BaseCamera = np.eye(4)
        X_BaseCamera[:3, :3] = R.from_quat(camera_wxyz[[1, 2, 3, 0]]).as_matrix()
        X_BaseCamera[:3, 3] = camera_pos
        X_BaseCamera_sapien = convert_camera_pose_z_forward_to_sapien(X_BaseCamera)
        sp_camera.entity.set_pose(sapien.Pose(X_BaseCamera_sapien))
        update_robot_trimesh_camera_and_mask([gui.value for gui in joint_gui_handles])

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

    camera_control.on_update(
        lambda _: update_camera_pose(camera_control.wxyz, camera_control.position)
    )
    update_robot_trimesh_camera_and_mask([gui.value for gui in joint_gui_handles])

    save_button = sv.gui.add_button("save_camera_pose",)
    save_button.on_click(
        lambda _: np.save("camera_pose.npy", X_BaseCamera)
    )

    while True:
        time.sleep(1)
        update_robot_trimesh_camera_and_mask([gui.value for gui in joint_gui_handles])
    # [0, 0, -76.2, 0, 76.2, 0]
