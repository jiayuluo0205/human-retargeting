import sys
sys.path.append('utils')
import rebocap_ws_sdk
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import pybullet_data
import viser
from pathlib import Path

from smpl_visualizer import *

import socket
import time

def detect_zero_norm(pose24_np):
    for idx, quat in enumerate(pose24_np):
        norm = np.linalg.norm(quat)
        if norm == 0 or np.any(np.isnan(quat)) or np.any(np.isinf(quat)):
            quat = np.array([0, 0, 0, 1])
            pose24_np[idx] = quat
    return pose24_np

counter = 0

def main():
    # def print_debug_msg(self: rebocap_ws_sdk.RebocapWsSdk, trans, pose24, static_index, ts):
    #     global counter

    #     is_left_on_floor = 0 <= static_index <= 5
    #     is_right_on_floor = 6 <= static_index <= 11
    #     no_foot_on_ground = static_index < 0
    #     if counter % 60 == 0:
    #         print(f'timestamp:{ts}'
    #               f'current coordinate_type: {self.coordinate_type.name}'
    #               f'root position:{trans} left_on_floor:{is_left_on_floor}  right_on_floor:{is_right_on_floor}')
    #         for i in range(24):
    #             print(f'bone:{rebocap_ws_sdk.REBOCAP_JOINT_NAMES[i]} quaternion w,x,y,z is:{pose24[i]}')
    #         print('\n\n\n\n', flush=True)
    #     counter += 1

    # # 姿态数据回调
    # def pose_msg_callback(self: rebocap_ws_sdk.RebocapWsSdk, tran: list, pose24: list, static_index: int, ts: float):
    #     print(f'X_RecocapLink20: \n{X_RecocapLink20}\n')
    #     print_debug_msg(self, tran, pose24, static_index, ts)
    #     # message = socket.recv()
    #     # 将 X_RecocapLink20 通过 zmq 发送
    #     # socket.send(X_RecocapLink20.tobytes())
    #     pass


    # # 异常断开，这里处理重连或报错
    # def exception_close_callback(self: rebocap_ws_sdk.RebocapWsSdk):
    #     print("exception_close_callback")

    # server initialize
    # 创建 TCP/IP 套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 绑定到本地端口 5555
    server_address = ('0.0.0.0', 5555)
    print('Starting up on {} port {}'.format(*server_address))
    sock.bind(server_address)
    # 开始监听连接
    sock.listen(1)
    print('Waiting for a connection...')
    # 接受连接
    connection, client_address = sock.accept()
    
    # # rebocap initialize
    # # 初始化sdk  这里可以选择控件坐标系， 控件坐标系目前已经测试过的有： 1. UE  2. Unity  3. Blender
    # # 选择输出角度是否是 global 角度，默认是 local 角度【简单解释，global 角度不受父节点影响 local角度受父节点影响， local角度逐级相乘就是 global 角度
    # sdk = rebocap_ws_sdk.RebocapWsSdk(coordinate_type=rebocap_ws_sdk.CoordinateType.DefaultCoordinate, use_global_rotation=False)
    # # 设置姿态回调
    # sdk.set_pose_msg_callback(pose_msg_callback)
    # # 设置异常断开回调
    # sdk.set_exception_close_callback(exception_close_callback)
    # # 开始连接
    # open_ret = sdk.open(7690)
    # # 检查连接状态
    # if open_ret == 0:
    #     print("连接成功")
    # else:
    #     print("连接失败", open_ret)
    #     if open_ret == 1:
    #         print("连接状态错误")
    #     elif open_ret == 2:
    #         print("连接失败")
    #     elif open_ret == 3:
    #         print("认证失败")
    #     else:
    #         print("未知错误", open_ret)
    #     exit(1)

    sv = viser.ViserServer(port=8085)
    
    '''smpl_visualizer.py'''
    model_path = Path("D:\Projects\human-retargeting/assets\Model\smplh\male\model.npz")
    server = viser.ViserServer()
    server.scene.set_up_direction("+y")
    server.gui.configure_theme(control_layout="collapsible")

    # Main loop. We'll read pose/shape from the GUI elements, compute the mesh,
    # and then send the updated mesh in a loop.
    model = SmplHelper(model_path)
    gui_elements = make_gui_elements(
        server,
        num_betas=model.num_betas,
        num_joints=model.num_joints,
        parent_idx=model.parent_idx,
    )
    body_handle = server.scene.add_mesh_simple(
        "/human",
        model.v_template,
        model.faces,
        wireframe=gui_elements.gui_wireframe.value,
        color=gui_elements.gui_rgb.value,
    )
    '''smpl_visualizer.py'''

    try:
        print('Connection from', client_address)
        
        # 持续接收数据
        while True:
            data = ""
            data = connection.recv(65536)
            if data:
                # 解码并打印收到的数据
                print(f"Received: {data.decode('utf-8')}")
                
                time.sleep(0.1)

                '''008_smpl_visualizer.py'''
                if not gui_elements.changed:
                    continue

                gui_elements.changed = False

                # update mano joints
                for idx, Rotvec_Joint_i in enumerate(Euler_Joint_combo):
                    gui_elements.gui_joints[idx].value = tuple(Euler_Joint_i)

                # If anything has changed, re-compute SMPL outputs.
                smpl_outputs = model.get_outputs(
                    betas=np.array([x.value for x in gui_elements.gui_betas]),
                    joint_rotmats=tf.SO3.exp(
                        # (num_joints, 3)
                        np.array([x.value for x in gui_elements.gui_joints])
                    ).as_matrix(),
                )

                # Update the mesh properties based on the SMPL model output + GUI
                # elements.
                body_handle.vertices = smpl_outputs.vertices
                body_handle.wireframe = gui_elements.gui_wireframe.value
                body_handle.color = gui_elements.gui_rgb.value

                # Match transform control gizmos to joint positions.
                for i, control in enumerate(gui_elements.transform_controls):
                    control.position = smpl_outputs.T_parent_joint[i, :3, 3]
                '''008_smpl_visualizer.py'''

    finally:
        # 关闭连接
        connection.close()

    # while True:
    #     global X_WorldLink20

    #     # message_recv = socket.recv()
    #     tran, pose24, static_index, tp = sdk.get_last_msg()

    #     # X_WorldLink9, X_WorldLink16, X_WorldLink17, X_WorldLink18, X_WorldLink19, X_WorldLink20, X_WorldLink21, X_WorldLink22, X_WorldLink23 = get_link(pose24)
        
    #     # arm control
    #     # X_WorldLinks_combo = get_link(pose24)
    #     # add_frames(sv, X_WorldLinks_combo)
    #     # X_WorldLinks_combo = np.stack(X_WorldLinks_combo, axis=0)

    #     Euler_Joint_combo = compute_joints_angles(pose24)
    #     Euler_Joint_combo = np.stack(Euler_Joint_combo, axis=0)
    #     '''008_smpl_visualizer.py'''
    #     if not gui_elements.changed:
    #         continue

    #     gui_elements.changed = False

    #     # update joints
    #     for idx, Euler_Joint_i in enumerate(Euler_Joint_combo):
    #         gui_elements.gui_joints[idx].value = tuple(Euler_Joint_i)

    #     # If anything has changed, re-compute SMPL outputs.
    #     smpl_outputs = model.get_outputs(
    #         betas=np.array([x.value for x in gui_elements.gui_betas]),
    #         joint_rotmats=tf.SO3.exp(
    #             # (num_joints, 3)
    #             np.array([x.value for x in gui_elements.gui_joints])
    #         ).as_matrix(),
    #     )

    #     # Update the mesh properties based on the SMPL model output + GUI
    #     # elements.
    #     body_handle.vertices = smpl_outputs.vertices
    #     body_handle.wireframe = gui_elements.gui_wireframe.value
    #     body_handle.color = gui_elements.gui_rgb.value

    #     # Match transform control gizmos to joint positions.
    #     for i, control in enumerate(gui_elements.transform_controls):
    #         control.position = smpl_outputs.T_parent_joint[i, :3, 3]
    #     '''008_smpl_visualizer.py'''

    #     message_send = Euler_Joint_combo.astype(np.float64).tobytes()
    #     '''
    #     X_WorldLink22and23 = np.stack([X_WorldLink22, X_WorldLink23], axis=0)
    #     message_send = X_WorldLink22and23.tobytes()
    #     '''
    #     # socket.send(message_send)

if __name__ == "__main__":
    main()