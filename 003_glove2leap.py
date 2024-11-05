import socket
import time
import re
import torch
import viser
from utils.hand_model import create_hand_model

# 定义手套数据和 leaphand 的关节映射关系，使用关节名称、偏置和方向反转
joint_mapping = {
    "R4": {"glove_index": 4, "joint_name": "j_thumb1y", "glove_range": (-30.0, 30.0), "reverse": True, "bias": 0.0, "target_range": (0.0, 1.0)},
    "R5": {"glove_index": 5, "joint_name": "j_thumb2y", "glove_range": (-45.0, 0.0), "reverse": False, "bias": 5.0, "target_range": (0.0, 1.57)},
    # 添加其他关节的映射
}

def normalize_value(value, input_range, target_range, bias=0.0, reverse=False):
    """将 value 从 input_range 映射到 target_range，同时应用偏置和方向反转。"""
    # 应用偏置
    value -= bias
    
    # 处理方向反转
    if reverse:
        value = -value
    
    # 执行归一化
    input_min, input_max = input_range
    target_min, target_max = target_range
    return target_min + (value - input_min) * (target_max - target_min) / (input_max - input_min)

def start_server(host='0.0.0.0', port=5555):
    # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Server listening on {host}:{port}")

    # Viser setup for visualization
    hand = create_hand_model('leaphand')
    pk_chain = hand.pk_chain
    lower, upper = pk_chain.get_joint_limits()
    server_vis = viser.ViserServer(host='127.0.0.1', port=8080)

    canonical_trimesh = hand.get_trimesh_q(hand.get_canonical_q())["visual"]
    server_vis.scene.add_mesh_simple(
        "leaphand",
        canonical_trimesh.vertices,
        canonical_trimesh.faces,
        color=(102, 192, 255),
        opacity=0.8
    )

    def update(q):
        trimesh = hand.get_trimesh_q(q)["visual"]
        server_vis.scene.add_mesh_simple(
            "leaphand",
            trimesh.vertices,
            trimesh.faces,
            color=(102, 192, 255),
            opacity=0.8
        )

    # 初始化 leaphand 模型滑条，使用关节名称
    gui_joints = {}
    for i, joint_name in enumerate(hand.get_joint_orders()):
        slider = server_vis.gui.add_slider(
            label=joint_name,
            min=round(lower[i], 2),
            max=round(upper[i], 2),
            step=(upper[i] - lower[i]) / 100,
            initial_value=0 if i < 6 else lower[i] * 0.75 + upper[i] * 0.25,
        )
        slider.on_update(lambda _: update(torch.tensor([slider.value for slider in gui_joints.values()])))
        gui_joints[joint_name] = slider  # 使用 joint_name 作为键

    try:
        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address} established.")
            
            try:
                while True:
                    # 从客户端套接字接收数据
                    data = ""  # 清空data确保每次为新的数据包
                    chunk = client_socket.recv(65536)
                    if not chunk:
                        break
                    data += chunk.decode("utf-8")  # 解码并累积接收到的数据
                    
                    # 匹配右手的所有传感器值
                    right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
                    right_hand_data = [float(value) for value in right_hand_matches[:28]]

                    # 通过关节名称映射来更新滑条
                    for mapping in joint_mapping.values():
                        glove_index = mapping["glove_index"]
                        glove_value = right_hand_data[glove_index]
                        glove_range = mapping["glove_range"]
                        joint_name = mapping["joint_name"]
                        
                        # 获取目标范围
                        target_range = mapping.get("target_range", 
                                                    (lower[hand.get_joint_orders().index(joint_name)], 
                                                     upper[hand.get_joint_orders().index(joint_name)]))
                        
                        # 应用归一化，包括反转和偏置
                        normalized_value = normalize_value(
                            glove_value,
                            glove_range,
                            target_range,
                            bias=mapping.get("bias", 0.0),
                            reverse=mapping.get("reverse", False)
                        )
                        
                        # 更新滑条
                        if joint_name in gui_joints:
                            gui_joints[joint_name].value = normalized_value
                    
                    # 更新三维模型
                    ordered_joint_values = [slider.value for slider in gui_joints.values()]
                    update(torch.tensor(ordered_joint_values))
            
            except ConnectionResetError:
                print("Connection closed by the client.")
            
            # 关闭客户端连接
            client_socket.close()
            print(f"Connection from {client_address} closed.\n")
    
    except KeyboardInterrupt:
        print("Server shutting down.")
    finally:
        # 关闭服务器套接字
        server_socket.close()

if __name__ == "__main__":
    start_server()