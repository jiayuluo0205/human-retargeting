from abc import abstractmethod
from typing import List, Optional

import nlopt
import numpy as np
import torch

from kinematics_adaptor import *
from humanoidrobot import *

class Optimizer:
    def __init__(
        self,
        robot: Robot,
        target_joint_names: List[str],
        target_link_human_indices: np.ndarray
    ):
        self.robot = robot
        self.num_joints = robot.dof ###

        joint_names = robot.dof_joint_names
        idx_pb2target = []
        for target_joint_name in target_joint_names:
            if target_joint_name not in joint_names:
                raise ValueError(f"Joint {target_joint_name} given does not appear to be in robot XML.")
            idx_pb2target.append(joint_names.index(target_joint_name))
        self.target_joint_names = target_joint_names
        self.idx_pb2target = np.array(idx_pb2target)

        self.idx_pb2fixed = np.array([i for i in range(robot.dof) if i not in idx_pb2target], dtype=int)
        self.opt = nlopt.opt(nlopt.LD_SLSQP, len(idx_pb2target))
        self.opt_dof = len(idx_pb2target)

        # Target
        self.target_link_human_indices = target_link_human_indices

        # Free joint
        link_names = robot.link_names
        self.has_free_joint = len([name for name in link_names if "dummy" in name]) >= 6

        # Kinematics adaptor
        self.adaptor: Optional[KinematicsAdaptor] = None
    
    def set_joint_limit(self, joint_limits: np.ndarray, epsilon=1e-3):
        if joint_limits.shape != (self.opt_dof, 2):
            raise ValueError(f"Expect joint limits have shape: {(self.opt_dof, 2)}, got {joint_limits.shape}")
        self.opt.set_lower_bounds((joint_limits[:, 0] - epsilon).tolist())
        self.opt.set_upper_bounds((joint_limits[:, 1] + epsilon).tolist())
    
    def get_link_indices(self, target_link_names):
        return [self.robot.get_link_index(link_name) for link_name in target_link_names]
    
    def set_kinematics_adaptor(self, adaptor: KinematicsAdaptor):
        self.adaptor = adaptor

    def retarget(self, ref_value, fixed_qpos, last_qpos):
        """
        Compute the retargeting results using non-linear optimization
        Args:
            ref_value: the reference value in cartesian space as input, different optimizer has different reference
            fixed_qpos: the fixed value (not optimized) in retargeting, consistent with self.fixed_joint_names
            last_qpos: the last retargeting results or initial value, consistent with function return

        Returns: joint position of robot, the joint order and dim is consistent with self.target_joint_names

        """
        objective_fn = self.get_objective_function(ref_value, fixed_qpos, np.array(last_qpos).astype(np.float32))
        
        self.opt.set_min_objective(objective_fn)
        try:
            qpos = self.opt.optimize(last_qpos)
            return np.array(qpos, dtype=np.float32)
        except RuntimeError as e:
            print(e)
            return np.array(last_qpos, dtype=np.float32)
    
    @abstractmethod
    def get_objective_function(self, ref_value: np.ndarray, fixed_qpos: np.ndarray, last_qpos: np.ndarray):
        pass

    @property
    def fixed_joint_names(self): ###
        joint_names = self.robot.dof_joint_names
        return [joint_names[i] for i in self.idx_pb2fixed]

class VectorOptimizer(Optimizer):
    def __init__(
        self,
        robot: Robot,
        target_joint_names: List[str],
        target_origin_link_names: List[str],
        target_task_link_names: List[str],
        target_link_human_indices: np.ndarray,
        huber_delta=0.02,
        norm_delta=4e-3,
        scaling=1.0,
    ):
        super().__init__(robot, target_joint_names, target_link_human_indices)
        self.origin_link_names = target_origin_link_names
        self.task_link_names = target_task_link_names
        self.huber_loss = torch.nn.SmoothL1Loss(beta=huber_delta, reduction="mean")
        self.norm_delta = norm_delta
        self.scaling = scaling

        # Compute cache for better performance
        # For links used multiple times, we dont compute multiple times
        self.computed_link_names = list(set(target_origin_link_names).union(set(target_task_link_names)))
        self.origin_link_indices = torch.tensor(
            [self.computed_link_names.index(name) for name in target_origin_link_names]
        )
        self.task_link_indices = torch.tensor(
            [self.computed_link_names.index(name) for name in target_task_link_names]
        )

        # cache link indices that will involve in kinematics computation
        self.computed_link_indices = self.get_link_indices(self.computed_link_names)

        self.opt.set_ftol_abs(1e-6)

    def get_objective_function(self, ref_value: np.ndarray, fixed_qpos: np.ndarray, last_qpos: np.ndarray):
        qpos = np.zeros(self.num_joints)
        qpos[self.idx_pb2fixed] = fixed_qpos
        torch_target_vec = torch.as_tensor(target_vector) * self.scaling
        torch_target_vec.requires_grad_(False)

        def objective(x: np.ndarray, grad: np.ndarray) -> float:
            qpos[self.idx_pb2target] = x

            # Kinematics forwarding for qpos
            if self.adaptor is not None:
                qpos[:] = self.adaptor.forward_qpos(qpos)[:]
            
            self.robot.compute_forward_kinematics(qpos)
            target_link_poses = [self.robot.get_link_pose(index) for index in self.computed_link_indices]
            body_pos = np.array([pose[:3, 3] for pose in target_link_poses])

            # Torch computation for accurate loss and grad
            torch_body_pos = torch.as_tensor(body_pos)
            torch_body_pos.requires_grad_()

            # Index link for computation
            origin_link_pos = torch_body_pos[self.origin_link_indices, :]
            task_link_pos = torch_body_pos[self.task_link_indices, :]
            robot_vec = task_link_pos - origin_link_pos

            # Loss term for kinematics retargeting based on 3D position error
            vec_dist = torch.norm(robot_vec - torch_target_vec, dim=1, keepdim=False)
            huber_distance = self.huber_loss(vec_dist, torch.zeros_like(vec_dist))
            result = huber_distance.cpu().detach().item()

            if grad.size > 0:
                jacobians = []
                for i, index in enumerate(self.computed_link_indices):
                    link_body_jacobian = self.robot.compute_single_link_local_jacobian(qpos, index)[:3, ...]
                    link_pose = target_link_poses[i]
                    link_rot = link_pose[:3, :3]
                    link_kinematics_jacobian = link_rot @ link_body_jacobian
                    jacobians.append(link_kinematics_jacobian)
                
                # Note: the joint order in this jacobian is consistent with pybullet
                jacobians = np.stack(jacobians, axis=0)
                huber_distance.backward()
                grad_pos = torch_body_pos.grad.cpu().numpy()[:, None, :]

                # Convert the jacobian from pybullet order to target order
                if self.adaptor is not None:
                    jacobians = self.adaptor.backward_jacobian(jacobians)
                else:
                    jacobians = jacobians[..., self.idx_pin2target]
                
                grad_qpos = np.matmul(grad_pos, np.array(jacobians))
                grad_qpos = grad_qpos.mean(1).sum(0)
                grad_qpos += 2 * self.norm_delta * (x - last_qpos)

                grad[:] = grad_qpos[:]
            
            return result
        
        return objective