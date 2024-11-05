import os
import torch
import viser
from smplx import MANO
import time

# 初始化Viser服务器
server = viser.ViserServer(host='127.0.0.1', port=8080)

# 初始化MANO手部模型
mano_model = MANO(model_path='assets/Model/mano_v1_2/models/MANO_LEFT.pkl', is_rhand=True, use_pca=False)
pose_params = torch.zeros(1, 48, requires_grad=False)  # 使用requires_grad=False提升性能
shape_params = torch.zeros(1, 10)  # 形状参数通常固定

# 获取手的初始3D网格顶点和面片（用来在Viser中可视化）
output = mano_model(global_orient=pose_params[:, :3], hand_pose=pose_params[:, 3:], betas=shape_params)
vertices = output.vertices[0].detach().numpy()  # 提取顶点数据
faces = mano_model.faces  # 提取面片数据

# 将手的初始姿势网格加载到Viser
server.scene.add_mesh_simple(
    "mano_hand",
    vertices,
    faces,
    color=(102, 192, 255),
    opacity=0.8
)

# 定义更新函数，使用给定的姿势参数更新手的网格
def update():
    with torch.no_grad():  # 禁用梯度计算以提升效率
        output = mano_model(global_orient=pose_params[:, :3], hand_pose=pose_params[:, 3:], betas=shape_params)
        vertices = output.vertices[0].cpu().numpy()
        server.scene.add_mesh_simple(
            "mano_hand",
            vertices,
            faces,
            color=(102, 192, 255),
            opacity=0.8
        )

# 为每个关节添加滑块
gui_joints = []
for i in range(48):  # MANO的手势参数维度是48
    slider = server.gui.add_slider(
        label=f"Joint {i + 1}",
        min=-3.14,  # 设置合理的旋转角度范围（弧度）
        max=3.14,
        step=0.01,
        initial_value=0.0
    )
    # 更新滑块的值，并触发实时更新
    def on_update_factory(index):
        def on_update(value):
            pose_params[0, index] = value  # 实时更新pose_params中的值
            update()  # 每次滑块更新后重新渲染手部
        return on_update
    
    slider.on_update(on_update_factory(i))  # 创建单独的更新函数避免闭包问题
    gui_joints.append(slider)

# 保持程序运行
while True:
    time.sleep(1)
