import zmq
import sys
sys.path.append('utils')
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import pybullet_data
import viser
from pathlib import Path

from utils.smpl_visualizer import *
import socket

counter = 0

def main():
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
    
    sv = viser.ViserServer(port=8085)
    
    '''smpl_visualizer.py'''
    model_path = Path("")
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

    print('Connection from', client_address)

    while True:
        
        if not gui_elements.changed:
            continue

        gui_elements.changed = False
        # ljy
        data = ""
        data = connection.recv(655360)
        if data:
            # 解码并打印收到的数据
            data_str = data.decode('utf-8')
            quat_joint_combo = np.array([float(num) for num in data_str.replace(",", " ").split()])
            quat_joint_combo = quat_joint_combo.reshape(-1, 4)
        
        for idx, quat_joint_i in enumerate(quat_joint_combo):
            rotvec_joint_i = R.from_quat(quat_joint_i).as_rotvec()
            gui_elements.gui_joints[idx].value = tuple(rotvec_joint_i)
        # ljy

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

        message_send = Euler_Joint_combo.astype(np.float64).tobytes()
        '''
        X_WorldLink22and23 = np.stack([X_WorldLink22, X_WorldLink23], axis=0)
        message_send = X_WorldLink22and23.tobytes()
        '''
        socket.send(message_send)

if __name__ == "__main__":
    main()