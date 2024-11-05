import trimesh
import numpy as np
import matplotlib.cm as cm
from scipy.spatial.transform import Rotation as R
import viser
from matplotlib import pyplot as plt
from xarm6_interface.utils.mesh_and_urdf_utils import as_mesh

def transform_matrix_to_viser_pose(transform_matrix):
    """
    Convert a 4x4 transformation matrix to a viser pose.

    Parameters:
    - transform_matrix (np.ndarray): The 4x4 transformation matrix.

    Returns:
    - viser_pose (dict): The viser pose.
    """
    viser_pose = {
        "pos": transform_matrix[:3, 3],
        "wxyz": trimesh.transformations.quaternion_from_matrix(
            transform_matrix[:3, :3]
        ),
    }
    return viser_pose


def vis_robot_frames(
    server,
    current_status,
    axes_length=0.05,
    axes_radius=0.0025,
):
    """
    Visualize the robot frames.
    """

    for link_name, link_transform in current_status.items():
        link_transform_np = (
            link_transform.get_matrix().detach().cpu().numpy().reshape(4, 4)
        )
        link_transform_quat_wxyz = trimesh.transformations.quaternion_from_matrix(
            link_transform_np
        )
        link_transform_xyz = link_transform_np[:3, 3]

        server.scene.add_frame(
            "frame_" + link_name,
            wxyz=link_transform_quat_wxyz,
            position=link_transform_xyz,
            axes_length=axes_length,
            axes_radius=axes_radius,
        )


def vis_pc_heatmap(server, pc, hmap, name="pc_hmap", radius=0.01, visible=True) -> None:
    """
    Draw the predicted heatmap and point cloud.

    Args:
    - pc: The point cloud. np, shape [N, 3].
    - hmap: The heatmap. np, shape [N].
    """
    colormap = cm.get_cmap("viridis")
    normalized_hmap = (hmap - hmap.min()) / (hmap.max() - hmap.min())
    normalized_hmap = normalized_hmap.reshape(
        -1,
    )
    hmap_colored = colormap(normalized_hmap)
    hmap_rgb = hmap_colored[:, :3]
    hmap_rgb_uint8 = (hmap_rgb * 255).astype("uint8")
    server.add_point_cloud(
        name,
        points=pc,
        point_size=radius,
        point_shape="circle",
        colors=hmap_rgb_uint8,
        visible=visible,
    )


def normalize(x):
    """
    Normalize the input vector. If the magnitude of the vector is zero, a small value is added to prevent division by zero.

    Parameters:
    - x (np.ndarray): Input vector to be normalized.

    Returns:
    - np.ndarray: Normalized vector.
    """
    if len(x.shape) == 1:
        mag = np.linalg.norm(x)
        if mag == 0:
            mag = mag + 1e-10
        return x / mag
    else:
        norms = np.linalg.norm(x, axis=1, keepdims=True)
        norms = np.where(norms == 0, 1e-10, norms)
        return x / norms


def sample_transform_w_normals(
    new_palm_center,
    new_face_vector,
    sample_roll,
    ori_face_vector=np.array([1.0, 0.0, 0.0]),
):
    """
    Compute the transformation matrix from the original palm pose to a new palm pose.

    Parameters:
    - new_palm_center (np.ndarray): The point of the palm center [x, y, z].
    - new_face_vector (np.ndarray): The direction vector representing the new palm facing direction.
    - sample_roll (float): The roll angle in range [0, 2*pi).
    - ori_face_vector (np.ndarray): The original direction vector representing the palm facing direction. Default is [1.0, 0.0, 0.0].

    Returns:
    - rst_transform (np.ndarray): A 4x4 transformation matrix.
    """

    rot_axis = np.cross(ori_face_vector, normalize(new_face_vector))
    rot_axis = rot_axis / (np.linalg.norm(rot_axis) + 1e-16)
    rot_ang = np.arccos(np.clip(np.dot(ori_face_vector, new_face_vector), -1.0, 1.0))

    if rot_ang > 3.1415 or rot_ang < -3.1415:
        rot_axis = (
            np.array([1.0, 0.0, 0.0])
            if not np.isclose(ori_face_vector, np.array([1.0, 0.0, 0.0])).all()
            else np.array([0.0, 1.0, 0.0])
        )

    rot = R.from_rotvec(rot_ang * rot_axis).as_matrix()
    roll_rot = R.from_rotvec(sample_roll * new_face_vector).as_matrix()

    final_rot = roll_rot @ rot
    rst_transform = np.eye(4)
    rst_transform[:3, :3] = final_rot
    rst_transform[:3, 3] = new_palm_center
    return rst_transform

def sample_transform_w_normals_batch(
    new_palm_centers,
    new_face_vectors,
    sample_rolls,
    ori_face_vector=np.array([1.0, 0.0, 0.0]),
):
    """
    Compute the transformation matrix from the original palm pose to a new palm pose for a batch of inputs.

    Parameters:
    - new_palm_centers (np.ndarray): The points of the palm center [B, 3].
    - new_face_vectors (np.ndarray): The direction vectors representing the new palm facing direction [B, 3].
    - sample_rolls (np.ndarray): The roll angles in range [0, 2*pi) [B].
    - ori_face_vector (np.ndarray): The original direction vector representing the palm facing direction. Default is [1.0, 0.0, 0.0].

    Returns:
    - rst_transforms (np.ndarray): A batch of 4x4 transformation matrices [B, 4, 4].
    """

    B = new_palm_centers.shape[0]
    normalized_new_face_vectors = normalize(new_face_vectors)
    rot_axes = np.cross(np.tile(ori_face_vector, (B, 1)), normalized_new_face_vectors)
    rot_axes = rot_axes / (np.linalg.norm(rot_axes, axis=1, keepdims=True) + 1e-16)
    dot_products = np.einsum('ij,j->i', normalized_new_face_vectors, ori_face_vector)
    rot_angs = np.arccos(np.clip(dot_products, -1.0, 1.0))

    mask = (rot_angs > 3.1415) | (rot_angs < -3.1415)
    alt_rot_axes = np.where(
        np.isclose(ori_face_vector, np.array([1.0, 0.0, 0.0])).all(),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 0.0, 0.0])
    )
    rot_axes[mask] = alt_rot_axes

    # Use Rotation.from_rotvec with batch processing
    rots = R.from_rotvec(rot_angs[:, np.newaxis] * rot_axes).as_matrix()
    roll_rots = R.from_rotvec(sample_rolls[:, np.newaxis] * new_face_vectors).as_matrix()

    final_rots = np.einsum('bij,bjk->bik', roll_rots, rots)
    rst_transforms = np.zeros((B, 4, 4))
    rst_transforms[:, :3, :3] = final_rots
    rst_transforms[:, :3, 3] = new_palm_centers
    rst_transforms[:, 3, 3] = 1.0

    return rst_transforms

def vis_vector(
    start_point,
    vector,
    length=0.1,
    cyliner_r=0.003,
    color=[255, 255, 100, 245],
    no_arrow=False,
):
    """
    start_points: np.ndarray, shape=(3,)
    vectors: np.ndarray, shape=(3,)
    length: cylinder length
    """
    normalized_vector = normalize(vector)
    end_point = start_point + length * normalized_vector

    # create a mesh for the force
    force_cylinder = trimesh.creation.cylinder(
        radius=cyliner_r, segment=np.array([start_point, end_point])
    )

    # create a mesh for the arrowhead
    cone_transform = sample_transform_w_normals(
        end_point, normalized_vector, 0, ori_face_vector=np.array([0.0, 0.0, 1.0])
    )
    arrowhead_cone = trimesh.creation.cone(
        radius=2 * cyliner_r, height=4 * cyliner_r, transform=cone_transform
    )
    # combine the two meshes into one
    if not no_arrow:
        force_mesh = force_cylinder + arrowhead_cone
    else:
        force_mesh = force_cylinder
    force_mesh.visual.face_colors = color

    return force_mesh

def update_viser_mp_result(sv:viser.ViserServer, arm_pk, current_joint_values, target_joint_values, waypt_joint_values_np, n_vis_waypt=5):
    # Load the current and target arm meshes
    current_arm_mesh = arm_pk.get_state_trimesh(current_joint_values, visual=True, collision=False)["visual"]
    target_arm_mesh = as_mesh(arm_pk.get_state_trimesh(target_joint_values, visual=True, collision=False)["visual"])

    # Add the current mesh to the scene
    sv.scene.add_mesh_trimesh("current_arm_mesh", current_arm_mesh)

    # Get the 'rainbow' colormap
    colormap = plt.get_cmap('rainbow')

    # Set color for the target mesh using the 'rainbow' colormap (choose a specific color, e.g., 0.8)
    target_color = colormap(0.9)[:3]  # Extract RGB values from the colormap

    # Add the target mesh to the scene with the specified color
    sv.scene.add_mesh_simple("target_arm_mesh", target_arm_mesh.vertices, target_arm_mesh.faces, color=target_color, opacity=0.9)

    # Return if there are no waypoints
    if len(waypt_joint_values_np) == 0:
        return

    # Determine waypoints to visualize
    n_total_waypt = len(waypt_joint_values_np)  # Total number of waypoints
    vis_waypt_idx = np.linspace(0, n_total_waypt - 1, num=n_vis_waypt + 2, dtype=int)[1:-1]
    vis_waypt_joint_values_np = waypt_joint_values_np[vis_waypt_idx]

    # Generate colors for each waypoint from the 'rainbow' colormap
    colors = colormap(np.linspace(0.1, 0.9, len(vis_waypt_joint_values_np)))  # Generate colors for each waypoint

    # Add each waypoint mesh to the scene with a color from the colormap
    for waypt_idx, vis_waypt_joint_values in enumerate(vis_waypt_joint_values_np):
        vis_waypt_mesh = as_mesh(arm_pk.get_state_trimesh(vis_waypt_joint_values, visual=True, collision=False)["visual"])
        waypoint_color = colors[waypt_idx][:3]  # Extract RGB values for this waypoint

        sv.scene.add_mesh_simple(f"vis_waypt_mesh_{waypt_idx}", vis_waypt_mesh.vertices, vis_waypt_mesh.faces, color=waypoint_color, opacity=0.5)
