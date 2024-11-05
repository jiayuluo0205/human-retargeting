import socket
import time
import re
import torch
import viser
import pytorch_kinematics as pk
from utils.hand_model import HandModel

def start_server(host='0.0.0.0', port=5556):
    # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Server listening on {host}:{port}")

    # Viser setup for visualization
    urdf_path = 'assets/robots/mano/mano.urdf'
    robot_name = 'mano'
    meshes_path = '/home/shaol/gjx/human-retargeting/assets/robots/mano/meshes'
    hand = HandModel(robot_name, urdf_path, meshes_path)
    pk_chain = hand.pk_chain
    lower, upper = pk_chain.get_joint_limits()
    server_vis = viser.ViserServer(host='127.0.0.1', port=8080)

    canonical_trimesh = hand.get_trimesh_q(hand.get_canonical_q())["visual"]
    server_vis.scene.add_mesh_simple(
        robot_name,
        canonical_trimesh.vertices,
        canonical_trimesh.faces,
        color=(102, 192, 255),
        opacity=0.8
    )

    def update(q):
        trimesh = hand.get_trimesh_q(q)["visual"]
        server_vis.scene.add_mesh_simple(
            robot_name,
            trimesh.vertices,
            trimesh.faces,
            color=(102, 192, 255),
            opacity=0.8
        )

    # Initialize sliders and use a dictionary for mapping by joint name
    gui_joints = {}
    for i, joint_name in enumerate(hand.get_joint_orders()):
        slider = server_vis.gui.add_slider(
            label=joint_name,
            min=round(lower[i], 2),
            max=round(upper[i], 2),
            step=(upper[i] - lower[i]) / 100,
            initial_value=0 if i < 6 else lower[i] * 0.75 + upper[i] * 0.25,
        )
        slider.on_update(lambda _: update(torch.tensor([s.value for s in gui_joints.values()])))
        gui_joints[joint_name] = slider  # 以 joint_name 为键存储滑条


    joint_mapping = {
        "R0": {"glove_index": 1, "joint_name": "j_thumb3", "scale": 0.5236, "reverse": True, "bias": 0.0},  # 右手拇指第三关节俯仰角
        "R1": {"glove_index": 2, "joint_name": "j_thumb2", "scale": 0.4363, "reverse": True, "bias": 0.0},  # 右手拇指第二关节俯仰角
        "R2": {"glove_index": 3, "joint_name": "j_thumb1y", "scale": 0.5236, "reverse": True, "bias": 0.0},  # 右手拇指第一关节俯仰角
        "R3": {"glove_index": 4, "joint_name": "j_thumb1z", "scale": 0.3491, "reverse": False, "bias": 0.0},  # 右手拇指第一关节偏航角

        "R4": {"glove_index": 4, "joint_name": "j_index3", "scale": 1.75, "reverse": True, "bias": 0.0}, # 右手食指第三关节俯仰角
        "R5": {"glove_index": 5, "joint_name": "j_index2", "scale": 1.75, "reverse": True, "bias": 0.0}, # 右手食指第二关节俯仰角
        "R6": {"glove_index": 6, "joint_name": "j_index1x", "scale": 1.57, "reverse": True, "bias": 0.0}, # 右手食指第一关节俯仰角
        "R7": {"glove_index": 7, "joint_name": "j_index1y", "scale": 0.35, "reverse": True, "bias": 0.0}, # 右手食指第一关节偏航角

        "R8": {"glove_index": 9, "joint_name": "j_middle3", "scale": 1.75, "reverse": True, "bias": 0.0},  # 右手中指第三关节俯仰角
        "R9": {"glove_index": 10, "joint_name": "j_middle2", "scale": 1.75, "reverse": True, "bias": 0.0},  # 右手中指第二关节俯仰角
        "R10": {"glove_index": 11, "joint_name": "j_middle1x", "scale": 1.57, "reverse": True, "bias": 0.0},  # 右手中指第一关节俯仰角
        "R11": {"glove_index": 12, "joint_name": "j_middle1y", "scale": 0.35, "reverse": True, "bias": 0.0},  # 右手中指第一关节偏航角

        "R12": {"glove_index": 13, "joint_name": "j_ring3", "scale": 0.4363, "reverse": True, "bias": 0.0},  # 右手无名指第三关节俯仰角
        "R13": {"glove_index": 14, "joint_name": "j_ring2", "scale": 0.4363, "reverse": True, "bias": 0.0},  # 右手无名指第二关节俯仰角
        "R14": {"glove_index": 15, "joint_name": "j_ring1x", "scale": 0.2618, "reverse": True, "bias": 0.0},  # 右手无名指第一关节俯仰角
        "R15": {"glove_index": 16, "joint_name": "j_ring1y", "scale": 0.3054, "reverse": True, "bias": 0.0},  # 右手无名指第一关节偏航角

        "R16": {"glove_index": 17, "joint_name": "j_pinky3", "scale": 0.4363, "reverse": True, "bias": 0.0},  # 右手小指第三关节俯仰角
        "R17": {"glove_index": 18, "joint_name": "j_pinky2", "scale": 0.4363, "reverse": True, "bias": 0.0},  # 右手小指第二关节俯仰角
        "R18": {"glove_index": 19, "joint_name": "j_pinky1x", "scale": 0.2618, "reverse": True, "bias": 0.0},  # 右手小指第一关节俯仰角
        "R19": {"glove_index": 20, "joint_name": "j_pinky1y", "scale": 0.4363, "reverse": True, "bias": 0.0},  # 右手小指第一关节偏航角
    }


    def normalize_value(value, scale, bias=0.0, reverse=False):
        """将 value 从 input_range 映射到 target_range，同时应用偏置和方向反转。"""
        # 应用偏置
        value -= bias
        
        # 处理方向反转
        if reverse:
            value = -value
        
        return value/scale

    try:
        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address} established.")
            
            try:
                while True:
                    data = ""  # 清空data确保每次为新的数据包
                    chunk = client_socket.recv(65536)
                    if not chunk:
                        break
                    data += chunk.decode("utf-8")
                    
                    # 匹配右手传感器值
                    right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
                    
                    # 将数据转换为浮点数并截取前28个关节值
                    right_hand_data = [float(value) for value in right_hand_matches[:28]]
                    
                    # 更新 Viser 中的滑条
                    for mapping in joint_mapping.values():
                        glove_index = mapping["glove_index"]
                        glove_value = right_hand_data[glove_index]
                        scale = mapping["scale"]
                        joint_name = mapping["joint_name"]
                        
                        # 仅对 joint_mapping 中指定的滑条进行归一化和更新
                        if joint_name in gui_joints:
                            slider = gui_joints[joint_name]
                            
                            # 调用 normalize_value 函数，传入 bias 和 reverse
                            normalized_value = normalize_value(
                                glove_value,
                                scale,
                                bias=mapping.get("bias", 0.0),
                                reverse=mapping.get("reverse", False)
                            )
                            
                            # 更新仅在 joint_mapping 中定义的滑条值
                            slider.value = normalized_value

                    ordered_joint_values = [slider.value for slider in gui_joints.values()]
                    update(torch.tensor(ordered_joint_values, dtype=torch.float32))


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