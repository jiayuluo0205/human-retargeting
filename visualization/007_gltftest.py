import open3d as o3d

# 读取并加载 .gltf 文件
mesh = o3d.io.read_triangle_mesh("assets/Model/UDEXREAL_SampleHand_out/UDEXREAL_SampleHand.gltf")

# 可视化
o3d.visualization.draw_geometries([mesh])