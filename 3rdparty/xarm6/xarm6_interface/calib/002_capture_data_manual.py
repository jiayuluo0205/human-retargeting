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
from xarm6_interface import XARM6LEFT_IP, XARM6_IP
from xarm6_interface.arm_pk import XArm6WOEE
from xarm6_interface.arm_rw import XArm6RealWorld
from xarm6_interface.arm_mplib import XARM6PlannerCfg, XARM6Planner
from xarm6_interface.envs.table_and_workspace_pc import WoodenTableMount, create_bounding_box_pc, create_plane_pc, env_pc_post_process
from xarm6_interface.utils import as_mesh, NVDiffrastRenderer, vis_robot_frames, Realsense, convert_camera_pose_z_forward_to_sapien, add_noise_to_transform, robust_compute_rotation_matrix_from_ortho6d 

import os
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

class VerificationState(Enum):
    WAITING_FOR_VERIFICATION = auto()
    VERIFIED_YES = auto()
    VERIFIED_NO = auto()


def update_viser_current_arm(sv:viser.ViserServer, arm_pk, current_joint_values, camera, init_X_BaseCamera):
    rt_dict = camera.getCurrentData(pointcloud=True)
    rgb_image = rt_dict["rgb"]
    pc_o3d = rt_dict["pointcloud_o3d"]
    current_arm_mesh = arm_pk.get_state_trimesh(current_joint_values, visual=True, collision=False)["visual"]
    sv.scene.add_mesh_trimesh("current_arm_mesh", current_arm_mesh)

    # add the camera pose
    camera_wxyz = R.from_matrix(init_X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]]
    camera_pos = init_X_BaseCamera[:3, 3]
    rtr_dict = camera.getCurrentData(pointcloud=True)
    rs_rgb = rtr_dict["rgb"]
    rs_pc = rtr_dict["pointcloud_o3d"]

    sv.scene.add_camera_frustum("rs_camera_img", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rs_rgb, scale=0.2)
    rs_pc_in_C_np = np.asarray(rs_pc.points) 
    rs_pc_in_B = init_X_BaseCamera @ np.vstack([rs_pc_in_C_np.T, np.ones(rs_pc_in_C_np.shape[0])])
    rs_pc_in_B = rs_pc_in_B[:3].T
    sv.scene.add_point_cloud("rs_camera_pc", rs_pc_in_B, colors=[200, 50, 50], point_size=0.005)


if __name__ == '__main__':

    ''' params '''
    serial_number="241122074374"
    exp_name = "1204_excalib_capture01"
    camera_data_path = Path(os.path.join(ROOT_DIR, f"3rdparty/xarm6/data/camera/{serial_number}"))
    save_data_rel_dir_path = (camera_data_path / exp_name).resolve()
    init_X_BaseCamera_path = (camera_data_path / "init_X_BaseCamera.npy").resolve() # init_X_BaseleftCamera.npy
    K_path = camera_data_path / "K.npy"

    ''' setup the realworld arm and camera '''
    # setup the real world xarm6
    xarm6_rw = XArm6RealWorld(ip=XARM6_IP)
    # setup camera 
    camera = Realsense(serial_number)
    K = np.load(K_path)
    camera.set_intrinsics(K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    H, W = camera.h, camera.w
    K = camera.K
    init_X_BaseCamera = np.load(init_X_BaseCamera_path)
    init_X_CameraBase = np.linalg.inv(init_X_BaseCamera)
    lgr.info("init_X_BaseCamera: \n{}".format(init_X_BaseCamera))

    ''' setup the digital twin xarm for vis '''
    xarm6_pk = XArm6WOEE()

    ''' setup the gui server '''
    sv = viser.ViserServer()
    button_verify_yes = sv.gui.add_button("verify_yes")
    buttion_exit = sv.gui.add_button("exit")


    def set_verification_state(state):
        global verification_state
        verification_state = state
        lgr.info(f"Verification state: {verification_state}")

    def on_exit():
        xarm6_rw.close()
        exit()


    n_collected_sample = 0

    def save_current_data():
        global n_collected_sample
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
        n_collected_sample += 1


    button_verify_yes.on_click(lambda _: save_current_data())
    buttion_exit.on_click(lambda _: on_exit())


    while True: 
        current_joint_values = xarm6_rw.get_joint_values()
        update_viser_current_arm(sv, xarm6_pk, current_joint_values, camera, init_X_BaseCamera)
        time.sleep(0.4)

