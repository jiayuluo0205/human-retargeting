import viser 
import numpy as np
from pathlib import Path
from xarm6_interface.utils.realsense import build_rs_cam
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import time 

if __name__ == "__main__":
    cam_rt_dict = build_rs_cam(
        cam_K_path = Path(__file__).resolve().parent.parent / "data" / "camera" / "mounted_rs" / "K.npy",
        X_BaseCamera_path = Path(__file__).resolve().parent.parent / "data" / "camera" / "mounted_rs" / "X_BaseCamera.npy",
    )
    cam = cam_rt_dict["cam"]
    cam_wxyz = cam_rt_dict["cam_wxyz"]
    cam_position = cam_rt_dict["cam_position"]
    X_BaseCamera = cam_rt_dict["X_BaseCamera"]
    
    sv = viser.ViserServer()
    while True: 
        rtr_dict = cam.getCurrentData()
        rgb = rtr_dict["rgb"]
        depth = rtr_dict["depth"]
        colored_depth = rtr_dict["colored_depth"]
        pc_o3d = rtr_dict["pointcloud_o3d"]
    
        sv.scene.add_camera_frustum(
            "cam_rgb", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=rgb, wxyz=cam_wxyz, position=cam_position
        )
        sv.scene.add_camera_frustum(
            "cam_depth", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=colored_depth,  wxyz=cam_wxyz, position=cam_position
        )
        cam_points = np.asarray(pc_o3d.points)
        world_points = np.dot(X_BaseCamera, np.concatenate([cam_points.T, np.ones((1, cam_points.shape[0]))], axis=0))[0:3].T
        sv.scene.add_point_cloud("pc", points=world_points, colors=pc_o3d.colors, point_shape="circle", point_size=0.003)
        time.sleep(0.1)
        
        