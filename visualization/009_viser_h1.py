from __future__ import annotations

import time
from typing import Literal

import numpy as np
import tyro
from robot_descriptions.loaders.yourdfpy import load_robot_description
from pathlib import Path
import os

import viser
from viser.extras import ViserUrdf
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import pybullet as p


def create_robot_control_sliders(
    server: viser.ViserServer, viser_urdf: ViserUrdf
) -> tuple[list[viser.GuiInputHandle[float]], list[float]]:
    """Create slider for each joint of the robot. We also update robot model
    when slider moves."""
    slider_handles: list[viser.GuiInputHandle[float]] = []
    initial_config: list[float] = []
    for joint_name, (
        lower,
        upper,
    ) in viser_urdf.get_actuated_joint_limits().items():
        lower = lower if lower is not None else -np.pi
        upper = upper if upper is not None else np.pi
        initial_pos = 0.0 if lower < 0 and upper > 0 else (lower + upper) / 2.0
        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=1e-3,
            initial_value=initial_pos,
        )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: viser_urdf.update_cfg(
                np.array([slider.value for slider in slider_handles])
            )
        )
        slider_handles.append(slider)
        initial_config.append(initial_pos)
    return slider_handles, initial_config


def main(
    robot_type: Literal[
        "panda",
        "ur10",
        "cassie",
        "allegro_hand",
        "barrett_hand",
        "robotiq_2f85",
        "atlas_drc",
        "g1",
        "h1",
        "anymal_c",
        "go2",
    ] = "h1",
) -> None:
    # Start viser server.
    server = viser.ViserServer()

    # Load URDF.
    #
    # This takes either a yourdfpy.URDF object or a path to a .urdf file.
    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=load_robot_description(robot_type + "_description"),
        # urdf_or_path=Path("human-retargeting/assets/robots/h1_description/urdf/h1.urdf"),
    )
    link_names = ['left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint', 
    'left_knee_joint', 'left_ankle_joint', 'right_hip_yaw_joint', 'right_hip_roll_joint', 
    'right_hip_pitch_joint', 'right_knee_joint', 'right_ankle_joint', 'torso_joint', 
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 
    'left_elbow_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 
    'right_shoulder_yaw_joint', 'right_elbow_joint', 
    'imu_joint', 'logo_joint', 'd435_left_imager_joint', 'd435_rgb_module_joint', 'mid360_joint']
    # for link_name in link_names:
    #     link_transform_quat_wxyz = 
    #     link_transform_xyz = 
    #     server.scene.add_frame(
    #         "frame_" + link_name,
    #         wxyz=link_transform_quat_wxyz,
    #         position=link_transform_xyz,
    #         axes_length=0.05,
    #         axes_radius=0.0025,
    #     )

    q = p.getQuaternionFromEuler([0.43633, 0, 0])
    q = (q[3], q[0], q[1], q[2])
    server.scene.add_frame(
        "frame_left_shoulder_pitch_joint",
        wxyz=q,
        position=(0.0055, 0.15535, 0.42999),
        # axes_length=0.05,
        # axes_radius=0.0025,
    )
    q = p.getQuaternionFromEuler([-0.43633, 0, 0])
    q = (q[3], q[0], q[1], q[2])
    server.scene.add_frame(
        "frame_left_shoulder_roll_joint",
        wxyz=q,
        position=(-0.0055, 0.0565, -0.0165),
        # axes_length=0.05,
        # axes_radius=0.0025,
    )

    # Create sliders in GUI that help us move the robot joints.
    with server.gui.add_folder("Joint position control"):
        (slider_handles, initial_config) = create_robot_control_sliders(
            server, viser_urdf
        )

    # Set initial robot configuration.
    viser_urdf.update_cfg(np.array(initial_config))

    # Create joint reset button.
    reset_button = server.gui.add_button("Reset")

    @reset_button.on_click
    def _(_):
        for s, init_q in zip(slider_handles, initial_config):
            s.value = init_q

    # Sleep forever.
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    tyro.cli(main)