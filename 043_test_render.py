import trimesh
import numpy as np
import pyrender
from IPython.display import display, Image

# 创建一个球体
mesh = trimesh.creation.icosphere(radius=1.0)

# 创建pyrender场景
scene = pyrender.Scene()

# 创建网格节点
mesh_node = pyrender.Mesh.from_trimesh(mesh)

# 将网格添加到场景
scene.add(mesh_node)

# 设置视角：创建一个相机并将其放置到一个特定的位置
camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0)
camera_pose = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 3.0],  # z轴的深度为3.0
    [0.0, 0.0, 0.0, 1.0]
])

# 将相机添加到场景中
scene.add(camera, pose=camera_pose)

# 设置光源（点光源）
light = pyrender.PointLight(color=np.ones(3), intensity=10.0)
scene.add(light, pose=camera_pose)
scene.bg_color = np.array([255, 255, 255, 0])

# 创建渲染器
renderer = pyrender.OffscreenRenderer(800, 600)  # 图像大小为800x600

# 渲染图像
color, depth = renderer.render(scene, flags=pyrender.RenderFlags.RGBA)
print(color.shape, depth.shape)
print(color[:, :, 3])
mask = color[:, :, 3] == 0
color = np.array(color[:, :, :3])
print(color.shape, depth.shape)
print(mask)
color[mask] = [0, 0, 255]

import cv2

cv2.imshow('color', color)
cv2.waitKey(0)
cv2.destroyAllWindows()
