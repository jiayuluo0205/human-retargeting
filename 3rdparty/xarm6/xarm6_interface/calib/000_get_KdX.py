import pyrealsense2 as rs
import numpy as np
import os

from xarm6_interface.utils import as_mesh, NVDiffrastRenderer, vis_robot_frames, Realsense, convert_camera_pose_z_forward_to_sapien, add_noise_to_transform, robust_compute_rotation_matrix_from_ortho6d 

serial_number = '147122075879' # '241122074374', '233622079809'
camera = Realsense(serial_number)
K = camera.K

print(f"K = {camera.K}")

save_path = f'../../data/camera/{serial_number}/'
os.makedirs(save_path, exist_ok=True)

np.save(save_path + 'K.npy', K)

assert False



# 初始化RealSense
pipeline = rs.pipeline()
config = rs.config()

# 指定序列号
serial_number = '233622079809'
config.enable_device(serial_number)

# 启用深度流和红外流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

pipeline_profile = pipeline.start(config)

# 获取左红外相机内参
left_profile = pipeline_profile.get_stream(rs.stream.infrared, 1)
left_intrinsics = left_profile.as_video_stream_profile().get_intrinsics()

# 构建相机内参矩阵K (使用左红外相机的内参)
K = np.array([[left_intrinsics.fx, 0, left_intrinsics.ppx],
              [0, left_intrinsics.fy, left_intrinsics.ppy],
              [0, 0, 1]])

# 获取畸变系数 (使用左红外相机的畸变系数)
d = np.array(left_intrinsics.coeffs)

# 获取右红外相机相对于左红外相机的外参
right_profile = pipeline_profile.get_stream(rs.stream.infrared, 2)
extrinsics = right_profile.get_extrinsics_to(left_profile)

# 构建4x4变换矩阵
rotation_matrix = np.array(extrinsics.rotation).reshape(3, 3)
translation_vector = np.array(extrinsics.translation).reshape(3, 1)
init_X_BaseleftCamera = np.vstack((
    np.hstack((rotation_matrix, translation_vector)),
    np.array([0, 0, 0, 1])
))

# 确保保存目录存在
save_path = f'../../data/camera/{serial_number}/'
os.makedirs(save_path, exist_ok=True)

# 保存为npy文件
np.save(save_path + 'K.npy', K)
np.save(save_path + 'd.npy', d)
np.save(save_path + 'init_X_BaseleftCamera.npy', init_X_BaseleftCamera)

print("内参矩阵 K:")
print(K)
print("\n畸变系数 d:")
print(d)
print("\n外参矩阵 init_X_BaseleftCamera:")
print(init_X_BaseleftCamera)

# 停止pipeline
pipeline.stop()