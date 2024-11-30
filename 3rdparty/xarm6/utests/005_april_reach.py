import time
import torch
import viser 
import sapien 
from pathlib import Path
import numpy as np 
from tqdm import tqdm
from loguru import logger as lgr
from PIL import Image, ImageColor
from scipy.spatial.transform import Rotation as R
from xarm6_interface.arm_pk import XArm6WOEE
from xarm6_interface.arm_mplib import XARM6PlannerCfg, XARM6Planner
from xarm6_interface.envs.table_and_workspace_pc import WoodenTableMount, create_bounding_box_pc, create_plane_pc, env_pc_post_process
from xarm6_interface.utils import as_mesh, NVDiffrastRenderer, vis_robot_frames, Realsense, convert_camera_pose_z_forward_to_sapien, add_noise_to_transform, robust_compute_rotation_matrix_from_ortho6d
from xarm6_interface.arm_rw import XArm6RealWorld
import cv2
from pupil_apriltags import Detector

def update_viser_current_arm(sv:viser.ViserServer, arm_pk, current_joint_values, camera, X_BaseCamera, camera_params, at_detector, tag_size):
    rt_dict = camera.getCurrentData(pointcloud=True)
    rgb_image = rt_dict["rgb"]
    pc_o3d = rt_dict["pointcloud_o3d"]

    gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    tags = at_detector.detect(
        gray_image, True, camera_params, tag_size
    )
    for tag_id, tag in enumerate(tags):
        X_CameraTag = np.eye(4)
        X_CameraTag[:3, :3] = tag.pose_R
        X_CameraTag[:3, 3] = tag.pose_t[:, 0]
        X_BaseTag = X_BaseCamera @ X_CameraTag
        lgr.info(f"Detected tag {tag_id} with pose {X_BaseTag}")
        X_BaseTag_wxyz = R.from_matrix(X_BaseTag[:3, :3]).as_quat()[[3, 0, 1, 2]]
        X_BaseTag_pos = X_BaseTag[:3, 3]
        sv.scene.add_frame("tag_frame", wxyz=X_BaseTag_wxyz, position=X_BaseTag_pos, axes_length=0.2, axes_radius=0.01)
    
    current_arm_mesh = arm_pk.get_state_trimesh(current_joint_values, visual=True, collision=False)["visual"]
    vis_robot_frames(sv, arm_pk.current_status, axes_length=0.15, axes_radius=0.005)
    
    sv.scene.add_mesh_trimesh("current_arm_mesh", current_arm_mesh)

    # add the camera pose
    camera_wxyz = R.from_matrix(X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]]
    camera_pos = X_BaseCamera[:3, 3]
    rtr_dict = camera.getCurrentData(pointcloud=True)
    rs_rgb = rtr_dict["rgb"]
    rs_pc = rtr_dict["pointcloud_o3d"]

    sv.scene.add_camera_frustum("rs_camera_img", fov=camera.fov_x, aspect=camera.aspect_ratio, wxyz=camera_wxyz, position=camera_pos, image=rs_rgb, scale=0.2)
    rs_pc_in_C_np = np.asarray(rs_pc.points) 
    rs_pc_in_B = X_BaseCamera @ np.vstack([rs_pc_in_C_np.T, np.ones(rs_pc_in_C_np.shape[0])])
    rs_pc_in_B = rs_pc_in_B[:3].T
    sv.scene.add_point_cloud("rs_camera_pc", rs_pc_in_B, colors=[200, 50, 50], point_size=0.005)


if __name__ == '__main__':
    ''' the test '''
    # setup camera 
    camera = Realsense()
    K_path = Path("xarm6_interface/calib/K_2.npy")
    opt_X_BaseCamera_path = Path("xarm6_interface/calib/data/002_0909test2/optimized_X_BaseCamera.npy")
    K = np.load(K_path)
    opt_X_BaseCamera = np.load(opt_X_BaseCamera_path)
    camera.set_intrinsics(K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    H, W = camera.h, camera.w
    K = camera.K

    at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )
    camera_params = (
        K[0, 0],
        K[1, 1],
        K[0, 2],
        K[1, 2],
    )
    "tag36h11-10; 0.048"

    xarm6_rw = XArm6RealWorld()

    ''' setup the digital twin xarm and the mask renderer'''
    xarm6_pk = XArm6WOEE()

    ''' setup the gui server '''
    sv = viser.ViserServer()
    button_verify_yes = sv.gui.add_button("verify_yes")
    buttion_exit = sv.gui.add_button("exit")

    while True: 
        current_joint_values = xarm6_rw.get_joint_values()

        update_viser_current_arm(sv, xarm6_pk, current_joint_values, camera, opt_X_BaseCamera, camera_params, at_detector, 0.048)
        time.sleep(0.1)