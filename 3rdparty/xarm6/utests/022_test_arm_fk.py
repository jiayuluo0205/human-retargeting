from diff_robot_hand.hand_model import LeapHandRight
import viser 

import numpy as np
from loguru import logger as lgr
from scipy.spatial.transform import Rotation as R

from diff_robot_hand.utils.viser_utils import vis_robot_frames

def transform4x4_to_wxyz_xyz(transform4x4):
    wxyz = R.from_matrix(transform4x4[:3, :3]).as_quat()[[3, 0, 1, 2]]
    xyz = transform4x4[:3, 3]
    return wxyz, xyz

def wxyz_xyz_to_transform4x4(wxyz, xyz):
    transform4x4 = np.eye(4)
    transform4x4[:3, :3] = R.from_quat([wxyz[1], wxyz[2], wxyz[3], wxyz[0]]).as_matrix()
    transform4x4[:3, 3] = xyz
    return transform4x4

from xarm6_interface.arm_pk import XArm6WOEE
from pathlib import Path

if __name__ == "__main__":
    transform_save_path = Path("third_party/xarm6/data/leapmount") / "X_ArmLeapbase.npy"
    X_ArmBase = np.load(transform_save_path)
    sv = viser.ViserServer()
    
    hand = LeapHandRight(
        load_balls_urdf=False,
        load_visual_mesh=True,
        load_col_mesh=True,
    )
    
    arm = XArm6WOEE()
    
    hand_open_mesh = hand.get_hand_trimesh(np.zeros(16))["visual"]  #hand.get
    hand_open_col_mesh_cvx = hand.get_hand_trimesh(np.zeros(16), collision=True)["collision"].convex_hull
    hand_open_col_mesh_cvx.export(transform_save_path.parent / "hand_open_col_mesh_cvx.obj")
    arm_mesh = arm.get_state_trimesh(arm.reference_joint_values_np, visual=True, collision=False)["visual"]
    vis_robot_frames(sv, arm.current_status)
    X_WorldEef = arm.current_status["link_eef"].get_matrix().detach().cpu().numpy().reshape(4, 4)
    X_WorldLeapbase = X_WorldEef @ X_ArmBase
    

    sv.scene.add_mesh_trimesh("hand_open_mesh", hand_open_mesh.apply_transform(X_WorldLeapbase))
    sv.scene.add_mesh_trimesh("arm_mesh", arm_mesh)
    
    base_transform = hand.current_status["base"].get_matrix().detach().cpu().numpy().reshape(4, 4)
    lgr.info(f"base_transform: {base_transform}")
    base_wxyz, base_xyz = transform4x4_to_wxyz_xyz(base_transform)  
    sv.scene.add_frame("base_frame", wxyz=base_wxyz, position=base_xyz, axes_radius=0.005)
    
    init_transform = np.eye(4)
    init_transform[:3, :3] = R.from_euler("YZ", [90, 180], degrees=True).as_matrix()
    init_wxyz, init_xyz = transform4x4_to_wxyz_xyz(init_transform)
    arm_transform = sv.scene.add_transform_controls(
        f"mount_transform",
        opacity=0.75,
        disable_sliders=True,
        scale=0.25,
        line_width=2.5,
        disable_rotations=True,
        wxyz=init_wxyz,
    )
    
    save_button = sv.gui.add_button("Save mount transform")
    
    def save_arm_to_hand_transform(
        arm_transform,
        base_transform,
        transform_save_path,
    ):
        X_WorldArm = arm_transform
        X_WorldBase = base_transform
        X_ArmBase = np.linalg.inv(X_WorldArm) @ X_WorldBase
        np.save(transform_save_path, X_ArmBase)
        
    
    
    # camera_control.wxyz, camera_control.position
    save_button.on_click(lambda _ : save_arm_to_hand_transform(wxyz_xyz_to_transform4x4(arm_transform.wxyz, arm_transform.position), base_transform, transform_save_path))
    
    
    import time 
    
    while True:
        time.sleep(0.1)
        lgr.info(f"arm_transform.position: {arm_transform.position}")
