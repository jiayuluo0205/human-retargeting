from abc import abstractmethod
from typing import List
import numpy as np
import numpy.typing as npt

from humanoidrobot import Robot

class KinematicsAdaptor:
    def __init__(
        self, 
        robot: Robot, 
        target_joint_names: List[str],
        ):
        self.robot = robot
        self.target_joint_names = target_joint_names

        # indices of target_joint_names in Pybullet
        self.idx_pb2target = np.array([robot.get_joint_index(joint_name) for joint_name in target_joint_names])

    @abstractmethod
    def forward_qpos(self, qpos: npt.NDArray) -> npt.NDArray:
        """
        Adapt the joint position for different kinematics constraints.
        Note that the joint order of this qpos is consistent with Pybullet.
        Return:
            the adapted qpos with the same shape as input

        """
        pass

    @abstractmethod
    def backward_jacobian(self, jacobian: npt.NDArray) -> npt.NDArray:
        """
        Adapt the jacobian for different kinematics applications.
        Note that the joint order of this jacobian is consistent with Pybullet.
        Return:
            the adapted jacobian with the same shape as input

        """
        pass

class H1KinematicsAdaptor(KinematicsAdaptor):
    def __init__(
        self,
        robot: Robot,
        target_joint_names: List[str],
        # source_joint_names: List[str],
    ):
        super().__init__(robot, target_joint_names)

        # # Indices in Pybullet
        # self.idx_pb2source = np.array([robot.get_joint_index(joint_name) for joint_name in source_joint_names])

        # # Indices in the output results
        # self.idx_target2source = np.array([self.target_joint_names.index(joint_name) for joint_name in source_joint_names])

        # # Dimension check
        # len_source = self.idx_target2source.shape[0]
        self.num_active_joints = len(robot.dof_joint_names)
    
    def forward_qpos(self, pb_qpos: npt.NDArray) -> npt.NDArray:
        return pin_qpos
    
    def backward_jacobian(self, jacobian: npt.NDArray) -> npt.NDArray:
        target_jacobian = jacobian[..., self.idx_pb2target]
        return target_jacobian