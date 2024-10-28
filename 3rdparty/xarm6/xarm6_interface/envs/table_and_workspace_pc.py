import numpy as np
import trimesh
from dataclasses import dataclass


@dataclass
class WoodenTableMount:
    '''
    The xarm is mounted on one side of the wooden table. 
    '''
    ''' the workspace boundary '''
    xmin:float = -0.7
    ymin:float = -0.3
    zmin:float = -0.04  # a little below the table
    xmax:float = 0.8
    ymax:float = 0.3
    zmax:float = 0.75

    ''' the table plane'''
    table_plane_xmin:float = -0.15
    table_plane_ymin:float = -0.35
    table_plane_zmin:float = 0.01
    table_plane_xmax:float = 0.8
    table_plane_ymax:float = 0.35
    table_plane_zmax:float = 0.01


def create_bounding_box_pc(xmin, ymin, zmin, xmax, ymax, zmax, n_pc):
    box = trimesh.primitives.Box(extents=[xmax - xmin, ymax - ymin, zmax - zmin])
    box.apply_translation([(xmax + xmin) / 2, (ymax + ymin) / 2, (zmax + zmin) / 2])
    pc = box.sample(n_pc)
    return pc

def create_plane_pc(xmin, ymin, zmin, xmax, ymax, zmax, n_pc):
    pc = np.zeros([n_pc, 3])
    pc[:, 0] = np.random.uniform(xmin, xmax, size=n_pc)
    pc[:, 1] = np.random.uniform(ymin, ymax, size=n_pc)
    pc[:, 2] = np.random.uniform(zmin, zmax, size=n_pc)
    return pc

def env_pc_post_process(env_pc, filter_norm_thresh=0.1, n_save_pc=10000):
    env_pc_norm = np.linalg.norm(env_pc, axis=-1)
    env_pc_norm_mask = env_pc_norm > filter_norm_thresh
    env_pc = env_pc[env_pc_norm_mask]
    if n_save_pc == None:
        return env_pc
    n_random_ids = np.random.choice(env_pc.shape[0], n_save_pc, replace=False)
    env_pc = env_pc[n_random_ids]
    return env_pc
