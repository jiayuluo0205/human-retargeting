from __future__ import annotations

from tqdm import tqdm
import mplib
from loguru import logger as lgr
import numpy as np
import sapien.core as sapien
from sapien.utils.viewer import Viewer

from pathlib import Path    
from dataclasses import dataclass
from xarm6_interface import XARM6_WO_EE_URDF_PATH, XARM6_WO_EE_SRDF_PATH
from xarm6_interface.envs.table_and_workspace_pc import WoodenTableMount, create_bounding_box_pc, create_plane_pc, env_pc_post_process
from dataclasses import dataclass, field
from typing import List
import time 
from mplib.pymp.collision_detection.fcl import Convex

@dataclass
class XARM6PlannerCfg:
    urdf_path: Path = XARM6_WO_EE_URDF_PATH.resolve()
    srdf_path: Path = XARM6_WO_EE_SRDF_PATH.resolve()

    move_group: str = "link_eef"
    vis: bool = True
    n_env_pc: int = 10000
    
    timestep: float = 1 / 240.0
    ground_height: float = 0.0
    static_friction: float = 1.0
    dynamic_friction: float = 1.0 
    restitution: float = 0.0
    ambient_light: List[float] = field(default_factory=lambda: [0.5, 0.5, 0.5])
    shadow: bool = True
    direction_lights: List[List[List[float]]] = field(default_factory=lambda: [[[0, 1, -1], [0.5, 0.5, 0.5]]])
    point_lights: List[List[List[float]]] = field(default_factory=lambda: [
        [[1, 2, 2], [1, 1, 1]],
        [[1, -2, 2], [1, 1, 1]],
        [[-1, 0, 1], [1, 1, 1]]
    ])
    
    camera_xyz_x: float = 1.2
    camera_xyz_y: float = 0.25
    camera_xyz_z: float = 0.4
    camera_rpy_r: float = 0
    camera_rpy_p: float = -0.4
    camera_rpy_y: float = 2.7

    joint_stiffness: float = 1000
    joint_damping: float = 200

    def get(self, key, default):
        if hasattr(self, key):
            return getattr(self, key)
        else:
            return default
    
class XARM6Planner:
    def __init__(self, cfg: XARM6PlannerCfg):
        self.cfg = cfg
        self.scene = sapien.Scene()
        if cfg.vis:
            self.set_viewer()

        # set simulation timestep
        self.scene.set_timestep(self.cfg.timestep)
        # add ground to scene
        self.scene.add_ground(self.cfg.ground_height)
        # set default physical material
        self.scene.default_physical_material = self.scene.create_physical_material(
            self.cfg.static_friction,
            self.cfg.dynamic_friction,
            self.cfg.restitution,
        )
        
        loader: sapien.URDFLoader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.sp_robot: sapien.Articulation = loader.load(str(cfg.urdf_path))
        self.sp_robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))
        self.active_joints = self.sp_robot.get_active_joints()
        for joint in self.active_joints:
            joint.set_drive_property(
                stiffness=self.cfg.get("joint_stiffness", 1000),
                damping=self.cfg.get("joint_damping", 200),
            )

        self.sp_link_names = [link.get_name() for link in self.sp_robot.get_links()]
        self.sp_joint_names = [joint.get_name() for joint in self.sp_robot.get_active_joints()]

        self.mplib_planner = mplib.Planner(
            urdf=str(cfg.urdf_path),
            srdf=str(cfg.srdf_path),
            user_link_names=self.sp_link_names,
            user_joint_names=self.sp_joint_names,
            move_group=cfg.move_group,
            joint_vel_limits=np.ones(6),
            joint_acc_limits=np.ones(6))
        
        self.joint_limits = np.concatenate(self.mplib_planner.pinocchio_model.get_joint_limits())
        self.sp_joint_lower_limits = self.joint_limits[:, 0]
        self.sp_joint_upper_limits = self.joint_limits[:, 1]

        # give some white ambient light of moderate intensity
        self.scene.set_ambient_light(self.cfg.ambient_light)
        # default enable shadow unless specified otherwise
        shadow = self.cfg.shadow
        # default spotlight angle and intensity
        direction_lights = self.cfg.direction_lights
        for direction_light in direction_lights:
            self.scene.add_directional_light(
                direction_light[0], direction_light[1], shadow=shadow
            )
        # default point lights position and intensity
        point_lights = self.cfg.point_lights
        for point_light in point_lights:
            self.scene.add_point_light(point_light[0], point_light[1], shadow=shadow)

    def set_viewer(self):
        # Ensure that self.cfg is an instance of XARM6PlannerCfg or similar
        if not isinstance(self.cfg, XARM6PlannerCfg):
            raise TypeError("self.cfg should be an instance of XARM6PlannerCfg")

        # initialize viewer with camera position and orientation
        self.viewer = Viewer()
        self.viewer.set_scene(self.scene)
        self.viewer.set_camera_xyz(
            x=self.cfg.camera_xyz_x,
            y=self.cfg.camera_xyz_y,
            z=self.cfg.camera_xyz_z,
        )
        self.viewer.set_camera_rpy(
            r=self.cfg.camera_rpy_r,
            p=self.cfg.camera_rpy_p,
            y=self.cfg.camera_rpy_y,
        )

    def mplib_add_point_cloud(self, point_cloud, name="env"):
        """
        :param point_cloud: (N, 3) numpy array in arm base frame
        """
        self.mplib_planner.update_point_cloud(point_cloud, name=name)
        return

    def mplib_update_attached_object(
            self,
            attached_trimesh, 
            X_EeAttached,
        ):
        attached_trimesh_cvx_hull = attached_trimesh.convex_hull
        cvx_vertices = attached_trimesh_cvx_hull.vertices
        cvx_faces = attached_trimesh_cvx_hull.faces
        fcl_cvx = Convex(cvx_vertices, cvx_faces)
        sapien_pose = sapien.Pose(X_EeAttached)
        self.mplib_planner.update_attached_object(fcl_cvx, sapien_pose)

    def mplib_sample_joint_values(self, n_samples, collision_check=True):
        # sample joint values in the joint space according to the joint limits
        joint_values = np.random.uniform(
            low=self.sp_joint_lower_limits,
            high=self.sp_joint_upper_limits,
            size=(n_samples, len(self.sp_joint_lower_limits)),
        )
        joint_values_to_save = []
        # check for collision
        if collision_check:
            for i in tqdm(range(n_samples)):
                sc_result = self.mplib_planner.check_for_self_collision(joint_values[i])
                envc_result = self.mplib_planner.check_for_env_collision(joint_values[i])
                if not sc_result and not envc_result:
                    joint_values_to_save.append(joint_values[i])
        else:
            joint_values_to_save = joint_values
        lgr.info(f"Number of valid samples: {len(joint_values_to_save)}")
        return joint_values_to_save
    
    def mplib_plan_qpos(self, current_qpos, target_qpos_list):
        planning_result = self.mplib_planner.plan_qpos(
            target_qpos_list,
            current_qpos,
            time_step=self.cfg.timestep,
        )
        return planning_result
    
    def mplib_plan_screw(self, current_qpos, target_pose):
        target_pose = sapien.Pose(target_pose)
        planning_result = self.mplib_planner.plan_screw(
            target_pose,
            current_qpos,
            time_step=self.cfg.timestep,
        )
        return planning_result

    def mplib_plan_pose(self, current_qpos, target_pose, return_closest=False):
        target_pose = sapien.Pose(target_pose)
        planning_result = self.mplib_planner.plan_pose(
            target_pose,
            current_qpos,
            time_step=self.cfg.timestep,
            verbose=True,
            planning_time=10,
            # return_closest=return_closest,
        )
        return planning_result

    def mplib_ik(self, current_qpos, target_pose, mask=[], return_closest=False):
        # target_pose = sapien.Pose(target_pose)
        target_pose = mplib.Pose(target_pose)
        status, q_goals = self.mplib_planner.IK(
            target_pose,
            current_qpos,
            verbose=True,
            mask=mask,
            return_closest=return_closest
        )
        return status, q_goals

    def sp_follow_path(self, result):
        """Helper function to follow a path generated by the planner"""
        # number of waypoints in the path
        n_step = result["position"].shape[0]
        # this makes sure the robot stays neutrally boyant instead of sagging
        # under gravity
        for i in range(n_step):
            qf = self.sp_robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=True
            )
            self.sp_robot.set_qf(qf)
            # set the joint positions and velocities for move group joints only.
            # The others are not the responsibility of the planner
            for j in range(len(self.mplib_planner.move_group_joint_indices)):
                self.active_joints[j].set_drive_target(result["position"][i][j])
                self.active_joints[j].set_drive_velocity_target(
                    result["velocity"][i][j]
                )
            # simulation step
            self.scene.step()
            if self.cfg.vis:
                self.scene.update_render()
                self.viewer.render()


    def move_to_pose(self, pose):
        """
        :param pose: 7D numpy array in arm base frame
        """
        return self.mplib_planner.plan(pose, self.robot.get_qpos(), time_step=1 / 250,
                                   use_point_cloud=True)

    def move_to_qpos(self, target_qpos, mask=[],
                     time_step=0.1,
                     rrt_range=0.1,
                     planning_time=1,
                     fix_joint_limits=True,
                     use_point_cloud=False,
                     use_attach=False,
                     verbose=False):
        target_qpos = np.array(target_qpos)
        current_qpos = self.robot.get_qpos()
        self.planner.planning_world.set_use_point_cloud(use_point_cloud)
        self.planner.planning_world.set_use_attach(use_attach)
        n = current_qpos.shape[0]
        if fix_joint_limits:
            for i in range(n):
                if current_qpos[i] < self.planner.joint_limits[i][0]:
                    current_qpos[i] = self.planner.joint_limits[i][0] + 1e-3
                if current_qpos[i] > self.planner.joint_limits[i][1]:
                    current_qpos[i] = self.planner.joint_limits[i][1] - 1e-3
        self.planner.robot.set_qpos(current_qpos, True)
        collisions = self.planner.planning_world.collide_full()
        if len(collisions) != 0:
            print("Invalid start state!")
            for collision in collisions:
                print("%s and %s collide!" % (collision.link_name1, collision.link_name2))

        idx = self.planner.move_group_joint_indices

        self.planner.robot.set_qpos(current_qpos, True)
        status, path = self.planner.planner.plan(
            current_qpos[idx],
            [target_qpos[idx]],
            range=rrt_range,
            verbose=verbose,
            time=planning_time,
        )
        if status == "Exact solution":
            times, pos, vel, acc, duration = self.planner.TOPP(path, time_step)
            return 0, {
                "status": "Success",
                "time": times,
                "position": pos,
                "velocity": vel,
                "acceleration": acc,
                "duration": duration,
            }
        else:
            return -1, None


def min_jerk_interpolator_with_alpha(waypt_joint_values_np, planner_timestep, cmd_timestep, alpha=0.2):
    """
    Min Jerk Interpolator with alpha to generate smooth trajectory command values.
    
    Args:
    - waypt_joint_values_np: np.ndarray of shape (n_sparse_wp, n_dof), waypoint joint values.
    - planner_timestep: float, the timestep of the planner, e.g., 1.0/20.0.
    - cmd_timestep: float, the timestep of the command, e.g., 1.0/500.0.
    - alpha: float, tuning parameter for the interpolation interval (0 < alpha <= 1), default is 0.33.
    
    Returns:
    - cmd_joint_values_np: np.ndarray, interpolated joint values for each command timestep.
    """
    
    n_sparse_wp, n_dof = waypt_joint_values_np.shape
    n_steps = int(planner_timestep / cmd_timestep)  # Number of interpolation steps

    # Calculate time fractions for interpolation (t')
    t = np.linspace(0, 1, n_steps)
    t_prime = np.clip(t / alpha, 0, 1)  # Apply alpha scaling and clipping

    # Min jerk interpolation formula using t'
    t_hat = 10 * t_prime**3 - 15 * t_prime**4 + 6 * t_prime**5  

    # Initialize the array for command joint values
    cmd_joint_values_np = []

    # Vectorized interpolation between waypoints
    for i in range(n_sparse_wp - 1):
        start_wp = waypt_joint_values_np[i]
        end_wp = waypt_joint_values_np[i + 1]
        
        # Interpolating values for the current segment using broadcasting
        interpolated_values = (1 - t_hat[:, np.newaxis]) * start_wp + t_hat[:, np.newaxis] * end_wp
        cmd_joint_values_np.append(interpolated_values)

    # Stack all interpolated segments together
    cmd_joint_values_np = np.vstack(cmd_joint_values_np)
    
    return cmd_joint_values_np


if __name__ == "__main__":
    
    ''' setup xarm '''
    xarm6_planner_cfg = XARM6PlannerCfg()
    xarm6_planner = XARM6Planner(xarm6_planner_cfg)
    
    ''' setup env '''
    env_params = WoodenTableMount()
    workspace_pc = create_bounding_box_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    table_plane_pc = create_plane_pc(env_params.table_plane_xmin, env_params.table_plane_ymin, env_params.table_plane_zmin, env_params.table_plane_xmax, env_params.table_plane_ymax, env_params.table_plane_zmax, xarm6_planner_cfg.n_env_pc)
    workspace_xmin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmin, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymin_pc = create_plane_pc(env_params.xmin, env_params.ymin, env_params.zmin, env_params.xmax, env_params.ymin, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    workspace_ymax_pc = create_plane_pc(env_params.xmin, env_params.ymax, env_params.zmin, env_params.xmax, env_params.ymax, env_params.zmax, xarm6_planner_cfg.n_env_pc)
    env_pc = np.concatenate([workspace_pc, table_plane_pc, workspace_xmin_pc, workspace_ymin_pc, workspace_ymax_pc], axis=0)
    env_pc = env_pc_post_process(env_pc, filter_norm_thresh=0.1, n_save_pc=None)
    xarm6_planner.mplib_add_point_cloud(env_pc)
    
    sampled_joint_values = xarm6_planner.mplib_sample_joint_values(1000, collision_check=True)

    for sample_id, joint_values in enumerate(sampled_joint_values):
        current_joint_values = xarm6_planner.sp_robot.get_qpos()
        planning_result = xarm6_planner.mplib_plan_qpos(current_joint_values, [joint_values])
        is_success = planning_result['status'] == 'Success'
        lgr.info(f"Sample {sample_id}: {is_success}")
        if not is_success:
            lgr.warning(f"Sample {sample_id} failed!")
        else:
            xarm6_planner.sp_follow_path(planning_result)

    