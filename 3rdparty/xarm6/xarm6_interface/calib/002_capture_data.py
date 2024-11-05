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

class VerificationState(Enum):
    WAITING_FOR_VERIFICATION = auto()
    VERIFIED_YES = auto()
    VERIFIED_NO = auto()


def update_viser_mp_result(sv:viser.ViserServer, arm_pk, current_joint_values, target_joint_values, waypt_joint_values_np, n_vis_waypt=5):
    # Load the current and target arm meshes
    current_arm_mesh = arm_pk.get_state_trimesh(current_joint_values, visual=True, collision=False)["visual"]
    target_arm_mesh = as_mesh(arm_pk.get_state_trimesh(target_joint_values, visual=True, collision=False)["visual"])

    # Add the current mesh to the scene
    sv.scene.add_mesh_trimesh("current_arm_mesh", current_arm_mesh)

    # Get the 'rainbow' colormap
    colormap = plt.get_cmap('rainbow')

    # Set color for the target mesh using the 'rainbow' colormap (choose a specific color, e.g., 0.8)
    target_color = colormap(0.9)[:3]  # Extract RGB values from the colormap

    # Add the target mesh to the scene with the specified color
    sv.scene.add_mesh_simple("target_arm_mesh", target_arm_mesh.vertices, target_arm_mesh.faces, color=target_color, opacity=0.9)

    # Return if there are no waypoints
    if len(waypt_joint_values_np) == 0:
        return

    # Determine waypoints to visualize
    n_total_waypt = len(waypt_joint_values_np)  # Total number of waypoints
    vis_waypt_idx = np.linspace(0, n_total_waypt - 1, num=n_vis_waypt + 2, dtype=int)[1:-1]
    vis_waypt_joint_values_np = waypt_joint_values_np[vis_waypt_idx]

    # Generate colors for each waypoint from the 'rainbow' colormap
    colors = colormap(np.linspace(0.1, 0.9, len(vis_waypt_joint_values_np)))  # Generate colors for each waypoint

    # Add each waypoint mesh to the scene with a color from the colormap
    for waypt_idx, vis_waypt_joint_values in enumerate(vis_waypt_joint_values_np):
        vis_waypt_mesh = as_mesh(arm_pk.get_state_trimesh(vis_waypt_joint_values, visual=True, collision=False)["visual"])
        waypoint_color = colors[waypt_idx][:3]  # Extract RGB values for this waypoint

        sv.scene.add_mesh_simple(f"vis_waypt_mesh_{waypt_idx}", vis_waypt_mesh.vertices, vis_waypt_mesh.faces, color=waypoint_color, opacity=0.5)


if __name__ == '__main__':
    ''' params '''
    n_sample_joint_values = 1000
    timestep = 1.0/30.0
    exp_name = "001_0909test"
    save_data_rel_dir_path = (Path("data") / exp_name).resolve()
    init_X_BaseCamera_path = (Path("data") / exp_name / "init_X_BaseCamera.npy").resolve()

    ''' setup the realworld arm and camera '''
    # setup the real world xarm6
    xarm6_rw = XArm6RealWorld()
    # setup camera 
    camera = Realsense()
    H, W = camera.h, camera.w
    K = camera.K
    init_X_BaseCamera = np.load(init_X_BaseCamera_path)
    init_X_CameraBase = np.linalg.inv(init_X_BaseCamera)
    lgr.info("init_X_BaseCamera: \n{}".format(init_X_BaseCamera))

    ''' setup the digital twin xarm for vis '''
    xarm6_pk = XArm6WOEE()

    ''' setup the simulated xarm (for motion planning with collision checking and visualization) '''
    # setup the sim xarm6
    xarm6_planner_cfg = XARM6PlannerCfg(vis=False, n_env_pc=10000, timestep=timestep)
    xarm6_planner = XARM6Planner(xarm6_planner_cfg)
    # setup the sim camera
    sp_camera = xarm6_planner.scene.add_camera(
        name="sim_camera",
        width=W,
        height=H,
        fovy=camera.fov_y,
        near=0.001,
        far=10.0,
    )
    sapien_cam_pose = convert_camera_pose_z_forward_to_sapien(init_X_CameraBase)
    sp_camera.entity.set_pose(sapien.Pose(sapien_cam_pose))
    sp_camera.set_focal_lengths(camera.intr.fx, camera.intr.fy)
    sp_camera.set_principal_point(camera.intr.ppx, camera.intr.ppy)
    # setup the collision environment
    env_params = WoodenTableMount()
    workspace_pc = create_bounding_box_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    table_plane_pc = create_plane_pc(env_params.table_plane_xmin, env_params.table_plane_ymin, env_params.table_plane_zmin, env_params.table_plane_xmax, env_params.table_plane_ymax, env_params.table_plane_zmax, xarm6_planner_cfg.n_env_pc)
    workspace_xmin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmin, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymin, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymax_pc = create_plane_pc(env_params.xmin, env_params.ymax, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    env_pc = np.concatenate([workspace_pc, table_plane_pc, workspace_xmin_pc, workspace_ymin_pc, workspace_ymax_pc], axis=0)
    env_pc = env_pc_post_process(env_pc, filter_norm_thresh=0.1, n_save_pc=None)
    xarm6_planner.mplib_add_point_cloud(env_pc)
    # sample some collision-free joint values
    collision_free_joint_values = xarm6_planner.mplib_sample_joint_values(n_sample_joint_values, collision_check=True)
    collision_free_joint_values = [xarm6_pk.reference_joint_values_np] + collision_free_joint_values
    n_valid_samples = len(collision_free_joint_values)

    ''' start the data collection '''

    ''' setup the gui server '''
    sv = viser.ViserServer()
    button_verify_yes = sv.gui.add_button("verify_yes")
    button_verify_no = sv.gui.add_button("verify_no")
    buttion_exit = sv.gui.add_button("exit")
    verification_state = VerificationState.WAITING_FOR_VERIFICATION

    def set_verification_state(state):
        global verification_state
        verification_state = state
        lgr.info(f"Verification state: {verification_state}")

    def on_exit():
        xarm6_rw.close()
        exit()

    button_verify_yes.on_click(lambda _: set_verification_state(VerificationState.VERIFIED_YES))
    button_verify_no.on_click(lambda _: set_verification_state(VerificationState.VERIFIED_NO))
    buttion_exit.on_click(lambda _: on_exit())

    ''' start the data collection '''

    # 1: collect the zero/reference joint values
    next_goto_sample_id = 0 
    n_collected_sample = 0

    while True:
        current_joint_values = xarm6_rw.get_joint_values()
        joint_values_to_check = collision_free_joint_values[next_goto_sample_id]
        planning_result = xarm6_planner.mplib_plan_qpos(current_joint_values, [joint_values_to_check])
        mp_is_success = planning_result['status'] == 'Success'
        if mp_is_success:
            lgr.info(f"Sample {joint_values_to_check}: Collision-free planning: Success")
            waypt_joint_values_np = planning_result['position']

            update_viser_mp_result(sv, xarm6_pk, current_joint_values, joint_values_to_check, waypt_joint_values_np)
            while verification_state == VerificationState.WAITING_FOR_VERIFICATION:
                time.sleep(1.0)  # wait for the user to verify
            if verification_state == VerificationState.VERIFIED_NO:
                lgr.info(f"Sample {joint_values_to_check}: Verified: No")
                verification_state = VerificationState.WAITING_FOR_VERIFICATION
                next_goto_sample_id += 1
                continue
            elif verification_state == VerificationState.VERIFIED_YES:
                lgr.info(f"Sample {joint_values_to_check}: Verified: Yes")
                verification_state = VerificationState.WAITING_FOR_VERIFICATION
                if len(waypt_joint_values_np) != 0:
                    xarm6_rw.set_joint_values_sequence(waypt_joint_values_np, wait=True, speed=50, sleep_time=timestep)
                else:
                    xarm6_rw.set_joint_values(joint_values_to_check)
                # rgb_image, depth_image, pc_o3d = camera.getCurrentData("rgb+depth+pointcloud")
                time.sleep(1.0)
                rt_dict = camera.getCurrentData(pointcloud=True)
                rgb_image = rt_dict["rgb"]
                depth_image = rt_dict["depth"]
                pc_o3d = rt_dict["pointcloud_o3d"]

                # save the data
                sample_dir_path = save_data_rel_dir_path / f"{n_collected_sample:04d}"
                sample_dir_path.mkdir(parents=True, exist_ok=True)
                np.save(sample_dir_path / "joint_values.npy", xarm6_rw.get_joint_values())
                cv2.imwrite(str(sample_dir_path / "rgb_image.jpg"), cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                np.save(sample_dir_path / "depth_image.npy", depth_image)
                o3d.io.write_point_cloud(str(sample_dir_path / "point_cloud.ply"), pc_o3d)

                next_goto_sample_id += 1
                n_collected_sample += 1
                continue
        else:
            lgr.info(f"Sample {joint_values_to_check}: Collision-free planning: Failed")
            next_goto_sample_id += 1
