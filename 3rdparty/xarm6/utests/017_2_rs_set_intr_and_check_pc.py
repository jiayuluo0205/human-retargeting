
from xarm6_interface.utils.realsense import build_rs_cam, MultiRealsense
from pathlib import Path
import viser 
from scipy.spatial.transform import Rotation as R

import numpy as np
if __name__ == "__main__":
    ''' multi realsense setup '''
    arm_left_cam_serial = "147122075879"
    arm_right_cam_serial = "241122074374"
    camera_serial_nums = [arm_left_cam_serial, arm_right_cam_serial]
    multi_rs = MultiRealsense(camera_serial_nums)
    arm_left_cam_K_path = Path("third_party/xarm6/data/camera/mounted_black/K.npy") 
    arm_left_cam_X_BaseCamera_path = Path("third_party/xarm6/data/camera/mounted_black/0914_excalib_capture1/optimized_X_BaseCamera.npy")
    arm_left_cam_K = np.load(arm_left_cam_K_path)
    arm_left_cam_X_BaseCamera = np.load(arm_left_cam_X_BaseCamera_path)
    arm_right_cam_K_path = Path("third_party/xarm6/data/camera/mounted_white/K.npy")
    arm_right_cam_X_BaseCamera_path = Path("third_party/xarm6/data/camera/mounted_white/0914_excalib_capture3/optimized_X_BaseCamera.npy")
    arm_right_cam_K = np.load(arm_right_cam_K_path)
    arm_right_cam_X_BaseCamera = np.load(arm_right_cam_X_BaseCamera_path)
    multi_rs.set_intrinsics(0, arm_left_cam_K[0, 0], arm_left_cam_K[1, 1], arm_left_cam_K[0, 2], arm_left_cam_K[1, 2])
    multi_rs.set_intrinsics(1, arm_right_cam_K[0, 0], arm_right_cam_K[1, 1], arm_right_cam_K[0, 2], arm_right_cam_K[1, 2])
    
    camera_wxyzs = [
        R.from_matrix(arm_left_cam_X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]],
        R.from_matrix(arm_right_cam_X_BaseCamera[:3, :3]).as_quat()[[3, 0, 1, 2]],
    ]
    camera_positions = [arm_left_cam_X_BaseCamera[:3, 3],
                          arm_right_cam_X_BaseCamera[:3, 3]]
    X_BaseCamera_list = [arm_left_cam_X_BaseCamera, arm_right_cam_X_BaseCamera]
    
    ''' get frames and show '''
    sv = viser.ViserServer()


    while True:
        rtr_dict_list = multi_rs.getCurrentData(pointcloud=True)
        for camera_idx in range(len(rtr_dict_list)):
            rtr_dict = rtr_dict_list[camera_idx]
            
            rs_rgb = rtr_dict["rgb"]
            rs_pc = rtr_dict["pointcloud_o3d"]
            
            camera_wxyz = camera_wxyzs[camera_idx]
            camera_pos = camera_positions[camera_idx]

            sv.scene.add_camera_frustum(f"rs_camera_img_{camera_idx}", fov=rtr_dict["fov_x"], aspect=rtr_dict["aspect_ratio"], wxyz=camera_wxyz, position=camera_pos, image=rs_rgb, scale=0.2)
        
            X_BaseCamera = X_BaseCamera_list[camera_idx]
            rs_pc_in_C_np = np.asarray(rs_pc.points) 
            rs_pc_in_B = X_BaseCamera @ np.vstack([rs_pc_in_C_np.T, np.ones(rs_pc_in_C_np.shape[0])])
            rs_pc_in_B = rs_pc_in_B[:3].T
            sv.scene.add_point_cloud(f"rs_camera_pc_{camera_idx}", rs_pc_in_B, colors=np.asarray(rs_pc.colors), point_size=0.001, point_shape="circle")

