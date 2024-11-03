"""
Visualizes hand joint motion within joint range (upper & lower limits) using Viser.
"""

import os
import sys
import time
import argparse
import torch
import viser
import pytorch_kinematics as pk
from utils.hand_model import HandModel

# 设置根目录路径
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)

urdf_path = 'assets/robots/mano/mano.urdf'
robot_name = 'mano'
meshes_path = '/home/shaol/gjx/human-retargeting/assets/robots/mano/meshes'

hand = HandModel(robot_name, urdf_path, meshes_path)

pk_chain = hand.pk_chain
lower, upper = pk_chain.get_joint_limits()

server = viser.ViserServer(host='127.0.0.1', port=8080)

canonical_trimesh = hand.get_trimesh_q(hand.get_canonical_q())["visual"]
server.scene.add_mesh_simple(
    robot_name,
    canonical_trimesh.vertices,
    canonical_trimesh.faces,
    color=(102, 192, 255),
    opacity=0.8
)

def update(q):
    trimesh = hand.get_trimesh_q(q)["visual"]
    server.scene.add_mesh_simple(
        robot_name,
        trimesh.vertices,
        trimesh.faces,
        color=(102, 192, 255),
        opacity=0.8
    )

gui_joints = []
for i, joint_name in enumerate(hand.get_joint_orders()):
    slider = server.gui.add_slider(
        label=joint_name,
        min=round(lower[i], 2),
        max=round(upper[i], 2),
        step=(upper[i] - lower[i]) / 100,
        initial_value=0 if i < 6 else lower[i] * 0.75 + upper[i] * 0.25,
    )
    slider.on_update(lambda _: update(torch.tensor([gui.value for gui in gui_joints])))
    gui_joints.append(slider)

while True:
    time.sleep(1)