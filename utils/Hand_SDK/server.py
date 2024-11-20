import socket
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

def detect_zero_norm(pose24_np):
    for idx, quat in enumerate(pose24_np):
        norm = np.linalg.norm(quat)
        if norm == 0 or np.any(np.isnan(quat)) or np.any(np.isinf(quat)):
            quat = np.array([0, 0, 0, 1])
            pose24_np[idx] = quat
    return pose24_np

def compute_mano_angles(pose15):
    Rotvec_joint_combo = []
    pose15 = detect_zero_norm(pose15)

    for idx, pose in enumerate(pose15):
        Rotvec_joint_i = R.from_quat(pose15[idx]).as_rotvec()
        Rotvec_joint_combo.append(Rotvec_joint_i)
    
    return Rotvec_joint_combo

# 创建 TCP/IP 套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定到本地端口 5555
server_address = ('127.0.0.1', 5555)
print('Starting up on {} port {}'.format(*server_address))
sock.bind(server_address)

# 开始监听连接
sock.listen(1)

print('Waiting for a connection...')

# 接受连接
connection, client_address = sock.accept()

try:
    print('Connection from', client_address)

    # 持续接收数据
    while True:
        data = ""
        data = connection.recv(6553600)
        if data:
            # 解码并打印收到的数据
            data_str = data.decode('utf-8')
            # print(f"Received: {data_str}")

            data_list = [x for x in data_str.split(',') if x.strip() != '']
            data_float = list(map(float, data_list))
            data_array = np.array(data_float).reshape(-1, 4)
            # print(data_array.shape)
            # print(f"Received: {data_array}")
            Rotvec_mano_combo = compute_mano_angles(data_array)
            Rotvec_mano_combo = np.stack(Rotvec_mano_combo, axis=0)
            print(Rotvec_mano_combo.shape)

finally:
    # 关闭连接
    connection.close()
    print("connection close")


