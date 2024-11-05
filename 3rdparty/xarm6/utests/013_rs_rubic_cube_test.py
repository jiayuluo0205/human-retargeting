import viser 
import torch
import numpy as np
from pathlib import Path
from xarm6_interface.utils.realsense import build_rs_cam
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import time 
from xarm6_interface.utils.realsense import build_rs_cam, get_masked_pointcloud
from xarm6_interface.utils.sam2_utils import grounding_dino_get_bbox, build_sam2_camera_predictor, show_bbox, show_mask
from xarm6_interface import SAM2_CHECKPOINT, SAM2_MODEL_CONFIG
from loguru import logger as lgr
import matplotlib.pyplot as plt

rgb = None
depth = None
mask_np = None
pc_o3d = None
masked_pc_o3d = None
saved_data_cnt = 0

if __name__ == "__main__":
    text_prompt = "a black box."
    save_data_dir = Path(__file__).resolve().parent.parent / "data" / text_prompt.replace(" ", "_").replace(".", "")
    save_data_dir.mkdir(parents=True, exist_ok=True)
    (save_data_dir / "rs_pc").mkdir(parents=True, exist_ok=True)
    (save_data_dir / "images").mkdir(parents=True, exist_ok=True)
    (save_data_dir / "rs_depth").mkdir(parents=True, exist_ok=True)
    (save_data_dir / "mask").mkdir(parents=True, exist_ok=True)
    (save_data_dir / "masked_rs_pc").mkdir(parents=True, exist_ok=True)
    
    ''' build the camera '''
    cam_rt_dict = build_rs_cam(
        cam_K_path = Path(__file__).resolve().parent.parent / "data" / "camera" / "free_rs" / "K.npy",
        X_BaseCamera_path = Path(__file__).resolve().parent.parent / "data" / "camera" / "mounted_rs" / "X_BaseCamera.npy",
    )
    cam = cam_rt_dict["cam"]
    cam_wxyz = cam_rt_dict["cam_wxyz"]
    cam_position = cam_rt_dict["cam_position"]
    X_BaseCamera = cam_rt_dict["X_BaseCamera"]
    
    
    ''' stablize the camera '''
    for i in range(50):
        cam.getCurrentData()
    ''' get the bbox for track '''
    rtr_dict = cam.getCurrentData()
    rgb = rtr_dict["rgb"]
    depth = rtr_dict["depth"]
    colored_depth = rtr_dict["colored_depth"]
    pc_o3d = rtr_dict["pointcloud_o3d"]
    input_boxes = grounding_dino_get_bbox(rgb, text_prompt)
    if input_boxes.shape != (4,):
        input_boxes = input_boxes[0].reshape(2, 2)
    # build the sam 2 model
    torch.autocast(device_type="cuda", dtype=torch.bfloat16).__enter__()
    if torch.cuda.get_device_properties(0).major >= 8:
        # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
    predictor = build_sam2_camera_predictor(SAM2_MODEL_CONFIG, SAM2_CHECKPOINT)
    predictor.load_first_frame(rgb)
    ann_frame_idx = 0 
    ann_obj_id = 1
    _, out_obj_ids, out_mask_logits = predictor.add_new_prompt(
        frame_idx=ann_frame_idx, obj_id=ann_obj_id, bbox=input_boxes.reshape(2, 2)
    )
    mask_np = (out_mask_logits[0] > 0.0).cpu().numpy()
    h, w = mask_np.shape[-2:]
    masked_pc_o3d = get_masked_pointcloud(rgb, depth, mask_np.reshape(h,w), cam.pinhole_camera_intrinsic)
    
    plt.gca().imshow(rgb)
    show_bbox(input_boxes.reshape(2, 2), plt.gca())
    show_mask((out_mask_logits[0] > 0.0).cpu().numpy(), plt.gca(), obj_id=out_obj_ids[0])
    plt.show()
    
    cmap = plt.get_cmap("tab10")
    mask_color = np.array([*cmap(0)[:3], 0.6])
    
    sv = viser.ViserServer()
    save_button = sv.gui.add_button("save")
    
    def save_current_data():
        global saved_data_cnt
        save_rgvb_path = save_data_dir / "images" / f"{saved_data_cnt:04d}.png"
        save_depth_path = save_data_dir / "rs_depth" / f"{saved_data_cnt:04d}.npy"
        save_mask_path = save_data_dir / "mask" / f"{saved_data_cnt:04d}.npy"
        save_pc_path = save_data_dir / "rs_pc" / f"{saved_data_cnt:04d}.ply"
        save_masked_pc_path = save_data_dir / "masked_rs_pc" / f"{saved_data_cnt:04d}.ply"
        
        plt.imsave(save_rgvb_path, rgb)
        np.save(save_depth_path, depth)
        np.save(save_mask_path, mask_np)
        o3d.io.write_point_cloud(str(save_pc_path), pc_o3d)
        o3d.io.write_point_cloud(str(save_masked_pc_path), masked_pc_o3d)
        
        saved_data_cnt += 1
        
    save_button.on_click(lambda _: save_current_data())
    

    while True: 
        rtr_dict = cam.getCurrentData()
        rgb = rtr_dict["rgb"]
        depth = rtr_dict["depth"]
        colored_depth = rtr_dict["colored_depth"]
        pc_o3d = rtr_dict["pointcloud_o3d"]

        ''' sam tracking and mask pc '''
        ann_frame_idx += 1
        out_obj_ids, out_mask_logits = predictor.track(rgb)
        mask_np = (out_mask_logits[0] > 0.0).cpu().numpy()
        h, w = mask_np.shape[-2:]
        mask_image = mask_np.reshape(h, w, 1) * mask_color.reshape(1, 1, -1)
        masked_pc_o3d = get_masked_pointcloud(rgb, depth, mask_np.reshape(h,w), cam.pinhole_camera_intrinsic)
        
        sv.scene.add_camera_frustum(
            "cam_rgb", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=rgb, wxyz=cam_wxyz, position=cam_position
        )
        sv.scene.add_camera_frustum(
            "cam_depth", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=colored_depth,  wxyz=cam_wxyz, position=cam_position
        )
        sv.scene.add_camera_frustum(
            "cam_mask", fov=cam.fov_x, aspect=cam.aspect_ratio, color=(150, 150, 150), scale=0.2,
            image=mask_image, wxyz=cam_wxyz, position=cam_position, format="png"
        )
        cam_points = np.asarray(pc_o3d.points)
        world_points = np.dot(X_BaseCamera, np.concatenate([cam_points.T, np.ones((1, cam_points.shape[0]))], axis=0))[0:3].T
        sv.scene.add_point_cloud("pc", points=world_points, colors=pc_o3d.colors, point_shape="circle", point_size=0.003)
        time.sleep(0.1)
        
        