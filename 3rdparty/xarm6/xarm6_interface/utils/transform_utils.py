import torch
import numpy as np
from scipy.spatial.transform import Rotation as R

def convert_camera_pose_z_forward_to_sapien(pose_z_forward):
    """
    Convert a camera pose from Z-forward convention to SAPIEN convention.

    :param pose_z_forward: A 4x4 numpy array representing the camera pose in the Z-forward convention.
    :return: A 4x4 numpy array representing the camera pose in the SAPIEN convention.
    """

    # Define the transformation matrix from Z-forward to SAPIEN convention
    # SAPIEN Convention: Forward: +X, Up: +Z, Right: -Y
    # Z-forward Convention: Forward: +Z, Up: -Y, Right: +X

    # Transformation matrix
    T = np.array([[0, -1, 0, 0],   # New X (Forward) = -Old Y
                  [0, 0, -1, 0],   # New Y (Right) = -Old Z
                  [1, 0, 0, 0],    # New Z (Up) = Old X
                  [0, 0, 0, 1]])  # X_ON

    # Apply the transformation
    pose_sapien = pose_z_forward @ T

    return pose_sapien


def add_noise_to_transform(transform, rotation_noise_angle_std=np.deg2rad(1), translation_noise_std=0.01):
    """
    Apply random noise to a 4x4 transformation matrix by rotating around a random axis.

    :param transform: A 4x4 numpy array representing the original transformation matrix.
    :param rotation_noise_angle_std: Standard deviation of the random angle for rotation noise (in radians).
    :param translation_noise_std: Standard deviation of the random noise for translation.
    :return: A 4x4 numpy array representing the noisy transformation matrix.
    """
    # Extract the original rotation matrix (3x3) and translation vector (3x1)
    R_orig = transform[:3, :3]
    t_orig = transform[:3, 3]

    # Generate a random unit vector (axis of rotation)
    random_axis = np.random.normal(size=3)
    random_axis /= np.linalg.norm(random_axis)  # Normalize to get a unit vector

    # Generate a random angle for rotation
    random_angle = np.random.normal(0, rotation_noise_angle_std)

    # Create a rotation matrix from the random axis and angle
    rotation_noise = R.from_rotvec(random_axis * random_angle).as_matrix()

    # Apply the rotation noise to the original rotation matrix
    R_noisy = R_orig @ rotation_noise

    # Generate random noise for the translation vector
    t_noisy = t_orig + np.random.normal(0, translation_noise_std, 3)

    # Construct the noisy transformation matrix
    noisy_transform = np.eye(4)
    noisy_transform[:3, :3] = R_noisy
    noisy_transform[:3, 3] = t_noisy

    return noisy_transform


def compute_rotation_matrix_from_ortho6d(poses):
    """
    Code from
    https://github.com/papagina/RotationContinuity
    On the Continuity of Rotation Representations in Neural Networks
    Zhou et al. CVPR19
    https://zhouyisjtu.github.io/project_rotation/rotation.html
    """
    x_raw = poses[:, 0:3]  # batch*3
    y_raw = poses[:, 3:6]  # batch*3
        
    x = normalize_vector(x_raw)  # batch*3
    z = cross_product(x, y_raw)  # batch*3
    z = normalize_vector(z)  # batch*3
    y = cross_product(z, x)  # batch*3
        
    x = x.view(-1, 3, 1)
    y = y.view(-1, 3, 1)
    z = z.view(-1, 3, 1)
    matrix = torch.cat((x, y, z), 2)  # batch*3*3
    return matrix

def robust_compute_rotation_matrix_from_ortho6d(poses):
    """
    Instead of making 2nd vector orthogonal to first
    create a base that takes into account the two predicted
    directions equally
    """
    x_raw = poses[:, 0:3]  # batch*3
    y_raw = poses[:, 3:6]  # batch*3

    x = normalize_vector(x_raw)  # batch*3
    y = normalize_vector(y_raw)  # batch*3
    middle = normalize_vector(x + y)
    orthmid = normalize_vector(x - y)
    x = normalize_vector(middle + orthmid)
    y = normalize_vector(middle - orthmid)
    # Their scalar product should be small !
    # assert torch.einsum("ij,ij->i", [x, y]).abs().max() < 0.00001
    z = normalize_vector(cross_product(x, y))

    x = x.view(-1, 3, 1)
    y = y.view(-1, 3, 1)
    z = z.view(-1, 3, 1)
    matrix = torch.cat((x, y, z), 2)  # batch*3*3
    # Check for reflection in matrix ! If found, flip last vector TODO
    assert (torch.stack([torch.det(mat) for mat in matrix ])< 0).sum() == 0
    return matrix


def normalize_vector(v):
    batch = v.shape[0]
    v_mag = torch.sqrt(v.pow(2).sum(1))  # batch
    v_mag = torch.max(v_mag, v.new([1e-8]))
    v_mag = v_mag.view(batch, 1).expand(batch, v.shape[1])
    v = v/v_mag
    return v


def cross_product(u, v):
    batch = u.shape[0]
    i = u[:, 1] * v[:, 2] - u[:, 2] * v[:, 1]
    j = u[:, 2] * v[:, 0] - u[:, 0] * v[:, 2]
    k = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
        
    out = torch.cat((i.view(batch, 1), j.view(batch, 1), k.view(batch, 1)), 1)
        
    return out
