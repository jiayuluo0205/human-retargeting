'''running on Linux machine for visualization and control of robot'''
# update 11.29
import sys
sys.path.append('utils')
import rebocap_ws_sdk
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import pybullet_data
import viser
from pathlib import Path

from utils.smpl_visualizer import *

import socket
import time
import zmq


def main():
    '''smpl_visualizer.py'''
    model_path = Path("D:\Projects\human-retargeting/assets\Model\smplh\male\model.npz")
    server = viser.ViserServer(port=8880)
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

    # client initialize
    # windows to linux
    context = zmq.Context()
    # talk to server windows machine
    print("Connecting to windows server...")
    socket = context.socket(zmq.REQ)
    '''tcp:ip adress of windows machine:5555'''
    socket.connect("tcp://localhost:5555")

    # 持续接收数据
    while True:
        socket.send(b"Request")
        message_byte = socket.recv()
        if message_byte == b"Fail":
            continue
        message_np = np.frombuffer(message_byte, dtype=np.float64)
        # print(message_np.shape)
        if message_np.ndim != 1 or message_np.shape[0] != 194:
            print(f"Invalid shape: {message_np.shape}, expected (194,)")
            continue
        # 194 = 32 + 162
        X_combo = message_np[:32]
        Rotvec_combo = message_np[32:]

        Rotvec_combo = Rotvec_combo.reshape((54,3))
        Rotvec_Joint_combo = Rotvec_combo[:24].copy()
        Rotvec_mano_combo = Rotvec_combo[24:].copy()
        print(Rotvec_Joint_combo.shape)
        print(Rotvec_mano_combo.shape)

        '''008_smpl_visualizer.py'''
        if not gui_elements.changed:
            continue

        gui_elements.changed = False
        
        # rebocap update
        for idx, Rotvec_Joint_i in enumerate(Rotvec_Joint_combo):
            gui_elements.gui_joints[idx].value = tuple(Rotvec_Joint_i)

        # mano update
        for i in Rotvec_mano_combo:
            i[1] = -i[1]
        # left index
        gui_elements.gui_joints[22].value = tuple(Rotvec_mano_combo[0])
        gui_elements.gui_joints[23].value = tuple(Rotvec_mano_combo[1])
        gui_elements.gui_joints[24].value = tuple(Rotvec_mano_combo[2])
        # left middle
        gui_elements.gui_joints[25].value = tuple(Rotvec_mano_combo[3])
        gui_elements.gui_joints[26].value = tuple(Rotvec_mano_combo[4])
        gui_elements.gui_joints[27].value = tuple(Rotvec_mano_combo[5])
        # left little
        gui_elements.gui_joints[28].value = tuple(Rotvec_mano_combo[9])
        gui_elements.gui_joints[29].value = tuple(Rotvec_mano_combo[10])
        gui_elements.gui_joints[30].value = tuple(Rotvec_mano_combo[11])
        # left ring
        gui_elements.gui_joints[31].value = tuple(Rotvec_mano_combo[6])
        gui_elements.gui_joints[32].value = tuple(Rotvec_mano_combo[7])
        gui_elements.gui_joints[33].value = tuple(Rotvec_mano_combo[8])
        # left thumb
        gui_elements.gui_joints[34].value = tuple(Rotvec_mano_combo[12])
        gui_elements.gui_joints[35].value = tuple(Rotvec_mano_combo[13])
        gui_elements.gui_joints[36].value = tuple(Rotvec_mano_combo[14])
        
        # right index
        gui_elements.gui_joints[37].value = tuple(-Rotvec_mano_combo[15])
        gui_elements.gui_joints[38].value = tuple(-Rotvec_mano_combo[16])
        gui_elements.gui_joints[39].value = tuple(-Rotvec_mano_combo[17])
        # right middle
        gui_elements.gui_joints[40].value = tuple(-Rotvec_mano_combo[18])
        gui_elements.gui_joints[41].value = tuple(-Rotvec_mano_combo[19])
        gui_elements.gui_joints[42].value = tuple(-Rotvec_mano_combo[20])
        # right little
        gui_elements.gui_joints[43].value = tuple(-Rotvec_mano_combo[24])
        gui_elements.gui_joints[44].value = tuple(-Rotvec_mano_combo[25])
        gui_elements.gui_joints[45].value = tuple(-Rotvec_mano_combo[26])
        # right ring
        gui_elements.gui_joints[46].value = tuple(-Rotvec_mano_combo[21])
        gui_elements.gui_joints[47].value = tuple(-Rotvec_mano_combo[22])
        gui_elements.gui_joints[48].value = tuple(-Rotvec_mano_combo[23])
        # right thumb
        gui_elements.gui_joints[49].value = tuple(-Rotvec_mano_combo[27])
        gui_elements.gui_joints[50].value = tuple(-Rotvec_mano_combo[28])
        gui_elements.gui_joints[51].value = tuple(-Rotvec_mano_combo[29])

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

if __name__ == "__main__":
    main()