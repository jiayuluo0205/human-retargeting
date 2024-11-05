import time
import torch
import typing
import pickle
import trimesh
import numpy as np
from pathlib import Path
import pytorch_kinematics as pk
from xarm6_interface import XARM6_WO_EE_URDF_PATH
from xarm6_interface.utils import (
    load_link_geometries,
    vis_robot_frames,
)

class RobotArm:

    def __init__(
        self,
        urdf_path: Path,
        load_visual_mesh: bool = True,
        load_col_mesh: bool = True,
        dtype=torch.float,
        device="cuda",
    ):
        self.urdf_path = urdf_path
        self.urdf_path_str = str(urdf_path)
        self.dtype = dtype
        self.device = device
        self.load_visual_mesh = load_visual_mesh
        self.load_col_mesh = load_col_mesh
        self.have_dummy_joints = "dummy" in self.urdf_path.name
        self.joint_start_idx = 6 if self.have_dummy_joints else 0

        self.pk_chain = pk.build_chain_from_urdf(
            open(self.urdf_path_str).read().encode()
        )
        self.pk_chain.to(device=self.device, dtype=self.dtype)

        self.all_joint_names = self.pk_chain.get_joint_parameter_names(
            exclude_fixed=False
        )
        self.actuated_joint_names = self.pk_chain.get_joint_parameter_names(
            exclude_fixed=True
        )
        self.ndof = len(self.actuated_joint_names)
        self.all_link_names = self.pk_chain.get_link_names()
        self.lower_joint_limits, self.upper_joint_limits = (
            self.pk_chain.get_joint_limits()
        )
        self.lower_joint_limits = self.ensure_tensor(self.lower_joint_limits)  # (1, ndof)
        self.upper_joint_limits = self.ensure_tensor(self.upper_joint_limits)  # (1, ndof)
        self.lower_joint_limits_np = self.lower_joint_limits.detach().cpu().numpy()[0]
        self.upper_joint_limits_np = self.upper_joint_limits.detach().cpu().numpy()[0]
        self.load_meshes()
        self.set_reference_joint_values()

    def set_reference_joint_values(self):
        self.reference_joint_values = (self.lower_joint_limits + self.upper_joint_limits) / 2
        self.reference_joint_values_np = self.reference_joint_values.detach().cpu().numpy()[0]

    def load_meshes(self):
        self.link_visuals_dict = load_link_geometries(
            self.urdf_path_str, self.all_link_names
        )
        self.link_collisions_dict = load_link_geometries(
            self.urdf_path_str, self.all_link_names, collision=True
        )

    def get_state_trimesh(
        self,
        joint_pos,
        X_w_b=torch.eye(4),
        visual=True,
        collision=False,
    ):
        """
        Get the trimesh representation of the robotic arm based on the provided joint positions and base transformation.

        Parameters:
        - joint_pos (list of float, or np.array, or torch tensor): Joint positions of the robot hand.
        - X_w_b (torch.tensor): A 4x4 transformation matrix representing the pose of the hand base in the world frame.
        - visual (bool): Whether to return the visual mesh of the hand.
        - collision (bool): Whether to return the collision mesh of the hand.

        Returns:
        - scene (trimesh.Trimesh): A trimesh object representing the robotic hand in its current pose.
        """

        self.current_status = self.pk_chain.forward_kinematics(
            th=self.ensure_tensor(joint_pos)
        )
        return_dict = {}
        face_num_count = 0
        if visual:
            scene = trimesh.Scene()
            for link_name in self.link_visuals_dict:
                mesh_transform_matrix = X_w_b @ self.current_status[
                    link_name
                ].get_matrix().detach().cpu().numpy().reshape(4, 4)
                part_mesh = (
                    self.link_visuals_dict[link_name]
                    .copy()
                    .apply_transform(mesh_transform_matrix)
                )
                part_mesh_face_num = len(part_mesh.faces)
                scene.add_geometry(part_mesh)
                face_num_count += part_mesh_face_num
            return_dict["visual"] = scene
        if collision:
            collision_scene = trimesh.Scene()
            for link_name in self.link_collisions_dict:
                mesh_transform_matrix = X_w_b @ self.current_status[
                    link_name
                ].get_matrix().detach().cpu().numpy().reshape(4, 4)
                part_mesh = (
                    self.link_collisions_dict[link_name]
                    .copy()
                    .apply_transform(mesh_transform_matrix)
                )
                part_mesh_face_num = len(part_mesh.faces)

                collision_scene.add_geometry(part_mesh)
            return_dict["collision"] = collision_scene
        return return_dict
    
    def ensure_tensor(self, th, ensure_batch_dim=True):
        """
        Converts a number of possible types into a tensor. The order of the tensor is determined by the order
        of self.get_joint_parameter_names(). th must contain all joints in the entire chain.
        """
        if isinstance(th, np.ndarray):
            th = torch.tensor(th, device=self.device, dtype=self.dtype)
        elif isinstance(th, list):
            th = torch.tensor(th, device=self.device, dtype=self.dtype)
        if len(th.shape) < 2 and ensure_batch_dim:
            th = th.unsqueeze(0)
        return th
    

class XArm6WOEE(RobotArm):
    def __init__(
        self,
        urdf_path: Path = XARM6_WO_EE_URDF_PATH,
        load_visual_mesh: bool = True,
        load_col_mesh: bool = True,
        dtype=torch.float,
        device="cuda",
    ):
        super().__init__(
            urdf_path=urdf_path,
            load_visual_mesh=load_visual_mesh,
            load_col_mesh=load_col_mesh,
            dtype=dtype,
            device=device,
        )

    def set_reference_joint_values(self):
        self.reference_joint_values = torch.zeros_like(self.lower_joint_limits)
        self.reference_joint_values_np = self.reference_joint_values.detach().cpu().numpy()[0]

if __name__ == "__main__":

    arm = XArm6WOEE()

    import viser 

    sv = viser.ViserServer()

    ''' urdf test '''
    joint_gui_handles = []

    def update_robot_trimesh(joint_values):
        trimesh_dict = arm.get_state_trimesh(
            joint_values,
            visual=True,
            collision=True,
        )
        visual_mesh = trimesh_dict["visual"]
        collision_mesh = trimesh_dict["collision"]
        sv.scene.add_mesh_trimesh("visual_mesh", visual_mesh)
        sv.scene.add_mesh_trimesh("collision_mesh", collision_mesh)
        vis_robot_frames(sv, arm.current_status, axes_length=0.15, axes_radius=0.005)

    for joint_name, lower, upper, initial_angle in zip(
        arm.actuated_joint_names, arm.lower_joint_limits_np, arm.upper_joint_limits_np, arm.reference_joint_values_np
    ):
        lower = float(lower) if lower is not None else -np.pi
        upper = float(upper) if upper is not None else np.pi
        slider = sv.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=0.05,
            initial_value=float(initial_angle),
        )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: update_robot_trimesh([gui.value for gui in joint_gui_handles])
        )
        joint_gui_handles.append(slider)

    update_robot_trimesh(arm.reference_joint_values_np)
    while True:
        time.sleep(1)

