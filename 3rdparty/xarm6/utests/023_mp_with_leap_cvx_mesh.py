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
from xarm6_interface.arm_mplib import XARM6PlannerCfg, XARM6Planner, min_jerk_interpolator_with_alpha
from xarm6_interface.envs.table_and_workspace_pc import WoodenTableMount, create_bounding_box_pc, create_plane_pc, env_pc_post_process
from xarm6_interface.utils import as_mesh, NVDiffrastRenderer, vis_robot_frames, Realsense, convert_camera_pose_z_forward_to_sapien, add_noise_to_transform, robust_compute_rotation_matrix_from_ortho6d 
import trimesh

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


# the planner/policy output action(joint positions) frequency at 20 Hz 
# then the cmds are being interpolated and send at 500Hz to the robot
# so each action is being interpolated into 25 cmds

if __name__ == '__main__':
    ''' params '''
    n_sample_joint_values = 1000
    planner_timestep = 1.0/20.0   
    cmds_timestep = 1.0/500.0
    
    ''' setup the planner and vis '''
    xarm6_pk = XArm6WOEE()
    xarm6_planner_cfg = XARM6PlannerCfg(vis=False, n_env_pc=10000, timestep=planner_timestep)
    xarm6_planner = XARM6Planner(xarm6_planner_cfg)
    env_params = WoodenTableMount()
    workspace_pc = create_bounding_box_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    table_plane_pc = create_plane_pc(env_params.table_plane_xmin, env_params.table_plane_ymin, env_params.table_plane_zmin, env_params.table_plane_xmax, env_params.table_plane_ymax, env_params.table_plane_zmax, xarm6_planner_cfg.n_env_pc)
    workspace_xmin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmin, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymin, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymax_pc = create_plane_pc(env_params.xmin, env_params.ymax, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    env_pc = np.concatenate([workspace_pc, table_plane_pc, workspace_xmin_pc, workspace_ymin_pc, workspace_ymax_pc], axis=0)
    env_pc = env_pc_post_process(env_pc, filter_norm_thresh=0.1, n_save_pc=None)
    xarm6_planner.mplib_add_point_cloud(env_pc)
    
    leapmount_data_dir = Path("third_party/xarm6/data/leapmount")
    leapmount_trimesh = trimesh.load_mesh(leapmount_data_dir / "hand_open_col_mesh_cvx.obj")
    leapmount_transform = np.load(leapmount_data_dir / "X_ArmLeapbase.npy")
    xarm6_planner.mplib_update_attached_object(
        leapmount_trimesh,
        leapmount_transform,
    )
    
    ''' setup the realworld arm '''
    xarm6_rw = XArm6RealWorld()

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
    
    default_joint_values = np.array([0.0, 0.0, -76.2, 0.0, 76.2, 0.0]) / 180 * np.pi
    current_joint_values = xarm6_rw.get_joint_values()
    end_joint_values = default_joint_values
    
    ''' plan from start to end '''
    
    planning_result = xarm6_planner.mplib_plan_qpos(current_joint_values, [end_joint_values])
    
    mp_is_success = planning_result['status'] == 'Success'
    if mp_is_success:
        lgr.info(f"Collision-free planning: Success")
        waypt_joint_values_np = planning_result['position']
        update_viser_mp_result(sv, xarm6_pk, current_joint_values, end_joint_values, waypt_joint_values_np)
        
        while verification_state == VerificationState.WAITING_FOR_VERIFICATION:
            time.sleep(1.0)  # wait for the user to verify
        if verification_state == VerificationState.VERIFIED_NO:
            lgr.info(f"Sample {end_joint_values}: Verified: No")
            verification_state = VerificationState.WAITING_FOR_VERIFICATION
        elif verification_state == VerificationState.VERIFIED_YES:
            verification_state = VerificationState.WAITING_FOR_VERIFICATION
            if len(waypt_joint_values_np) != 0:
                cmd_joint_values_np = min_jerk_interpolator_with_alpha(waypt_joint_values_np, planner_timestep, cmds_timestep, alpha=0.2)
                xarm6_rw.set_joint_values_sequence(cmd_joint_values_np, wait=False, speed=50, sleep_time=cmds_timestep)
            else:
                xarm6_rw.set_joint_values(end_joint_values)
            lgr.info(f"Sample {end_joint_values}: Verified: Yes")
    else:
        lgr.info(f"Collision-free planning: Fail")