from typing import List

import numpy as np
import numpy.typing as npt
import pybullet as p

class Robot:
    def __init__(self, urdf_path: str, use_Collision=False, use_visual=False):
        # Create robot body
        self.body: int = p.loadURDF(urdf_path)

        if use_visual or use_Collision:
            raise NotImplementedError
        
        self.num_joints = p.getNumJoints(self.body)
        self.q0 = np.zeros(self.num_joints)
        self.jointnames = []
        # Vector of dimension of the joint configuration subspace
        self.nqs = []
        self.linknames = []
        self.upper = []
        self.lower = []

        # get neutral configuration; store in q0
        for joint_index in range(self.num_joints):
            joint_info = p.getJointInfo(self.body, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            link_name = joint_info[12].decode('utf-8')
            joint_type = joint_info[2]
            joint_upper = joint_info[9]
            joint_lower = joint_info[8]

            self.jointnames.append(joint_name)
            self.linknames.append(link_name)
            self.upper.append(joint_upper)
            self.lower.append(joint_lower)

            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                self.nqs.append(1)
            elif joint_type == p.JOINT_FIXED:
                self.nqs.append(0)
            # elif joint_type == p.JOINT_SPHERICAL:
            #     self.nqs.append(3)
            # else:
            #     self.nqs.append(1)

            self.q0[joint_index] = p.getJointState(self.body, joint_index)[0]

        # Dimension of the configuration vector representation
        self.nq = sum(self.nqs)
    # -------------------------------------------------------------------------- #
    # Robot property
    # -------------------------------------------------------------------------- #
    @property
    def joint_names(self) -> List[str]:
        return self.jointnames

    @property
    def dof_joint_names(self) -> List[str]:
        return [name for i, name in enumerate(self.jointnames) if self.nqs[i] > 0]
    
    @property
    def dof(self) -> int:
        return self.nq
    
    @property
    def link_names(self) -> List[str]:
        return self.linknames
    
    @property
    def joint_limits(self):
        return np.stack([self.lower, self.upper], axis=1)
    # -------------------------------------------------------------------------- #
    # Query function
    # -------------------------------------------------------------------------- #
    def get_joint_index(self, name: str):
        return self.dof_joint_names.index(name)

    def get_link_index(self, name: str):
        if name not in self.link_names:
            raise ValueError(f"{name} is not a link name. Valid link names: \n{self.link_names}")
        return self.link_names.index(name)

    def get_joint_parent_child_frames(self, joint_name: str):
        joint_id = self.joint_names.index(joint_name)
        joint_info = p.getJointInfo(self.body, joint_id)
        parent_id = joint_info[16]
        # child_id = joint_id
        child_id = -1
        for idx, joint in enumerate(self.joint_names):
            if joint_id == p.getJointInfo(self.body, idx)[16]:
                child_id = idx
        if child_id == -1:
            raise ValueError(f"Cannot find child link of {joint_name}")
        return parent_id, child_id
    # -------------------------------------------------------------------------- #
    # Kinematics function
    # -------------------------------------------------------------------------- #
    def compute_forward_kinematics(self, qpos: npt.NDArray):
        assert len(qpos) == self.num_joints, "the length of qpos does not match the joint number of the robot"

        # set joint pose
        p.setJointMotorControlArray(self.body, list(range(self.num_joints)), controlMode=POSITION_CONTROL, taughtPositions=qpos.tolist())
        
        # compute forward kinematics
        for link_index in range(self.num_joints):
            p.getLinkState(self.body, link_index, computeForwardKinematics=True)
    
    def get_link_pose(self, link_id: int) -> npt.NDArray:
        link_state = p.getLinkState(self.body, link_id, computeForwardKinematics=True)
        link_world_position = link_state[4] # vec3
        link_world_orientation = link_state[5] # vec4 quaternion
        rot_matrix = p.getMatrixFromQuaternion(link_world_orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rot_matrix
        homogeneous_matrix[:3, 3] = link_world_position
        return homogeneous_matrix
    
    def get_link_pose_inv(self, link_id: int) -> npt.NDArray:
        link_state = p.getLinkState(self.body, link_id, computeForwardKinematics=True)
        link_world_position = link_state[4] # vec3
        link_world_orientation = link_state[5] # vec4 quaternion
        rot_matrix = p.getMatrixFromQuaternion(link_world_orientation)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rot_matrix
        homogeneous_matrix[:3, 3] = link_world_position
        homogeneous_matrix_inv = np.linalg.inv(homogeneous_matrix)
        return homogeneous_matrix_inv
    
    def compute_single_link_local_jacobian(self, qpos, link_id: int) ->npt.NDArray:
        zero_vec = [0.0] * self.num_joints

        # centor of mass
        link_com_position = p.getLinkState(self.body, link_id, computeForwardKinematics=True)[2]

        jacobian = p.calculateJacobian(
            self.body,
            link_id,
            link_com_position,
            qpos.tolist(),
            zero_vec,
            zero_vec
        )

        linear_jacobian = np.array(jacobian[0])
        angular_jacobian = np.array(jacobian[1])

        J = np.vstack((linear_jacobian, angular_jacobian))

        return J