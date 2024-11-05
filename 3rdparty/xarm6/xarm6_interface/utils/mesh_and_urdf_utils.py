import os
import torch
import trimesh
import fpsample  # pip install -U fpsample
import numpy as np
from typing import List, Union
import pytorch_kinematics as pk
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R


def as_mesh(scene_or_mesh):
    """
    Convert a possible scene to a mesh.

    If conversion occurs, the returned mesh has only vertex and face data.
    """
    if isinstance(scene_or_mesh, trimesh.Scene):
        if len(scene_or_mesh.geometry) == 0:
            mesh = None  # empty scene
        else:
            # we lose texture information here
            mesh = trimesh.util.concatenate(
                tuple(
                    trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                    for g in scene_or_mesh.geometry.values()
                )
            )
    else:
        assert isinstance(scene_or_mesh, trimesh.Trimesh)
        mesh = scene_or_mesh
    return mesh


def ray_mesh_intersections(mesh, ray_interval=0.005, random_rotation=True, n_points=None):
    # 0. get a random rotation of the mesh
    if random_rotation:
        T_random = trimesh.transformations.random_rotation_matrix()
    else:
        T_random = np.eye(4)
    mesh_rotated = mesh.copy().apply_transform(T_random)

    # 1. Get the bounding box of the mesh
    bounds = mesh_rotated.bounds

    # 2. & 3. Generate rays for each face of the bounding box
    rays_origins = []
    rays_directions = []

    for i in range(3):  # Loop over x, y, z directions
        for direction in [-1, 1]:  # Negative and positive directions
            # Create a grid of points (ray origins)
            if i == 0:
                starting_x = bounds[0][0] if direction == 1 else bounds[1][0]
                grid = np.array(
                    [
                        [starting_x, y, z]
                        for y in np.arange(bounds[0][1], bounds[1][1], ray_interval)
                        for z in np.arange(bounds[0][2], bounds[1][2], ray_interval)
                    ]
                )
            elif i == 1:
                starting_y = bounds[0][1] if direction == 1 else bounds[1][1]
                grid = np.array(
                    [
                        [x, starting_y, z]
                        for x in np.arange(bounds[0][0], bounds[1][0], ray_interval)
                        for z in np.arange(bounds[0][2], bounds[1][2], ray_interval)
                    ]
                )
            elif i == 2:
                starting_z = bounds[0][2] if direction == 1 else bounds[1][2]
                grid = np.array(
                    [
                        [x, y, starting_z]
                        for x in np.arange(bounds[0][0], bounds[1][0], ray_interval)
                        for y in np.arange(bounds[0][1], bounds[1][1], ray_interval)
                    ]
                )

            # Calculate ray directions
            directions = np.zeros_like(grid)
            directions[:, i] = direction

            rays_origins.append(grid)
            rays_directions.append(directions)

    # Combine all rays from all faces
    rays_origins = np.vstack(rays_origins)
    rays_directions = np.vstack(rays_directions)

    # 4. Compute ray-mesh intersections
    rmi = trimesh.ray.ray_pyembree.RayMeshIntersector(mesh_rotated, scale_to_box=False)
    locations, index_ray, index_tri = rmi.intersects_location(
        rays_origins, rays_directions, multiple_hits=False
    )

    # 5. Get normals for the intersection points
    normals = mesh.face_normals[index_tri]

    # 6. Transform the intersection points back to the original mesh
    T_random_inv = np.linalg.inv(T_random)
    locations_h = np.hstack((locations, np.ones((locations.shape[0], 1))))
    locations_transformed = np.dot(T_random_inv, locations_h.T).T [:, :3]

    # 7. If n_points is specified, do FPS on the intersection points
    if n_points is not None:
        fps_samples_idx = fpsample.fps_npdu_kdtree_sampling(locations_transformed, n_points)  
        # NOTE: don't use fpsample.bucket_fps_kdtree_sampling and fpsample.bucket_fps_kdline_sampling
        locations_transformed = locations_transformed[fps_samples_idx]
        index_tri = index_tri[fps_samples_idx]
        normals = normals[fps_samples_idx]
        
    return locations_transformed, index_tri, normals


def extract_colors_from_urdf(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    global_materials = {}
    for material in root.findall("material"):
        name = material.attrib["name"]
        color_elem = material.find("color")
        if color_elem is not None and "rgba" in color_elem.attrib:
            rgba = [float(c) for c in color_elem.attrib["rgba"].split()]
            global_materials[name] = rgba

    link_colors = {}

    for link in root.iter("link"):
        link_name = link.attrib["name"]
        visual = link.find("./visual")
        if visual is not None:
            material = visual.find("./material")
            if material is not None:
                color = material.find("color")
                if color is not None and "rgba" in color.attrib:
                    rgba = [float(c) for c in color.attrib["rgba"].split()]
                    link_colors[link_name] = rgba
                elif "name" in material.attrib:
                    material_name = material.attrib["name"]
                    if material_name in global_materials:
                        link_colors[link_name] = global_materials[material_name]

    return link_colors


def build_urdf_visualizer(
    urdf_path, load_visual_mesh=True, load_collision_mesh=True, sample_mesh_point_num=0
):
    urdf_dir_path = os.path.abspath(os.path.dirname(urdf_path))
    chain = pk.build_chain_from_urdf(open(urdf_path).read().encode())

    def build_mesh_recurse(body, meshes_dict, mesh_type="visuals"):
        """load visuals or collisions mesh"""
        if mesh_type != "visuals":
            print(dir(body.link))
        if (
            len(getattr(body.link, mesh_type)) > 0
            and getattr(body.link, mesh_type)[0].geom_type is not None
        ):
            link_vertices = []
            link_faces = []
            n_link_vertices = 0
            for visual in getattr(body.link, mesh_type):
                scale = torch.tensor([1, 1, 1], dtype=torch.float)
                if visual.geom_type == "box":
                    link_mesh = trimesh.primitives.Box(
                        extents=np.array(visual.geom_param)
                    )
                elif visual.geom_type == "capsule":
                    link_mesh = trimesh.primitives.Capsule(
                        radius=visual.geom_param[0], height=visual.geom_param[1] * 2
                    ).apply_translation((0, 0, -visual.geom_param[1]))
                elif visual.geom_type == "sphere":
                    link_mesh = trimesh.primitives.Capsule(
                        radius=visual.geom_param, height=0
                    ).apply_translation((0, 0, 0))
                elif visual.geom_type == "mesh":
                    mesh_path = None
                    if isinstance(visual.geom_param, tuple):
                        mesh_path = visual.geom_param[0]
                    elif isinstance(visual.geom_param, str):
                        mesh_path = visual.geom_param
                    else:
                        raise ValueError("Unknown type of visual.geom_param")
                    if os.path.isabs(mesh_path):
                        link_mesh = as_mesh(trimesh.load_mesh(mesh_path, process=False))
                    else:
                        link_mesh = as_mesh(
                            trimesh.load_mesh(
                                os.path.join(urdf_dir_path, mesh_path), process=False
                            )
                        )

                vertices = torch.tensor(link_mesh.vertices, dtype=torch.float)
                faces = torch.tensor(link_mesh.faces, dtype=torch.float)
                pos = visual.offset
                vertices = vertices * scale
                vertices = pos.transform_points(vertices)
                link_vertices.append(vertices)
                link_faces.append(faces + n_link_vertices)
                n_link_vertices += len(vertices)
            link_vertices = torch.cat(link_vertices, dim=0)
            link_faces = torch.cat(link_faces, dim=0)
            link_mesh = trimesh.Trimesh(vertices=link_vertices, faces=link_faces)
            meshes_dict[body.link.name] = {
                "mesh": link_mesh,
                "vertices": link_vertices,
                "faces": link_faces,
                "offset": pos.get_matrix()[:, :3, 3],
            }
            if sample_mesh_point_num > 0:
                link_mesh_sample_points, link_mesh_sample_face_indices = (
                    trimesh.sample.sample_surface(link_mesh, sample_mesh_point_num)
                )
                link_mesh_sample_normals = link_mesh.face_normals[
                    link_mesh_sample_face_indices
                ]
                meshes_dict[body.link.name]["sample_points"] = link_mesh_sample_points
                meshes_dict[body.link.name]["sample_normals"] = link_mesh_sample_normals

        for children in body.children:
            build_mesh_recurse(children, meshes_dict)

    visual_meshes_dict = {}
    collision_mesh_dict = {}

    if load_visual_mesh:
        build_mesh_recurse(chain._root, visual_meshes_dict, "visuals")
        link_colors_from_urdf = extract_colors_from_urdf(urdf_path)
        for link_name in visual_meshes_dict.keys():
            visual_meshes_dict[link_name]["colors"] = link_colors_from_urdf.get(
                link_name, [0.8, 0.2, 0.2, 1.0]
            )

    if load_collision_mesh:
        build_mesh_recurse(chain._root, collision_mesh_dict, "collision")
        link_colors_from_urdf = extract_colors_from_urdf(urdf_path)
        for link_name in collision_mesh_dict.keys():
            collision_mesh_dict[link_name]["colors"] = link_colors_from_urdf.get(
                link_name, [0.8, 0.2, 0.2, 1.0]
            )

    return visual_meshes_dict, collision_mesh_dict


def parse_origin(element):
    """Parse the origin element for translation and rotation."""
    origin = element.find("origin")
    xyz = np.zeros(3)
    rotation = np.eye(3)
    if origin is not None:
        xyz = np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(
            origin.attrib.get("rpy", "0 0 0"), sep=" "
        )  # Roll, pitch, yaw
        rotation = R.from_euler("xyz", rpy).as_matrix()
    return xyz, rotation


def apply_transform(mesh, translation, rotation):
    """Apply translation and rotation to a mesh."""
    # mesh.apply_translation(-mesh.centroid)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    mesh.apply_transform(transform)
    return mesh


def create_primitive_mesh(geometry, translation, rotation):
    """Create a trimesh object from primitive geometry definitions with transformations."""
    if geometry.tag.endswith("box"):
        size = np.fromstring(geometry.attrib["size"], sep=" ")
        mesh = trimesh.creation.box(extents=size)
    elif geometry.tag.endswith("sphere"):
        radius = float(geometry.attrib["radius"])
        mesh = trimesh.creation.icosphere(radius=radius)
    elif geometry.tag.endswith("cylinder"):
        radius = float(geometry.attrib["radius"])
        length = float(geometry.attrib["length"])
        mesh = trimesh.creation.cylinder(radius=radius, height=length)
    else:
        raise ValueError(f"Unsupported geometry type: {geometry.tag}")
    return apply_transform(mesh, translation, rotation)


def load_link_geometries(urdf_path, link_names, collision=False):
    """Load geometries (trimesh objects) for specified links from a URDF file, considering origins."""
    urdf_dir = os.path.dirname(urdf_path)
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    link_geometries = {}
    link_colors_from_urdf = extract_colors_from_urdf(urdf_path)

    for link in root.findall("link"):
        link_name = link.attrib["name"]
        link_color = link_colors_from_urdf.get(link_name, None)
        if link_name in link_names:
            geom_index = "visual"
            if collision:
                geom_index = "collision"
            link_mesh = []
            for visual in link.findall(".//" + geom_index):
                geometry = visual.find("geometry")
                xyz, rotation = parse_origin(visual)
                try:
                    if geometry[0].tag.endswith("mesh"):
                        mesh_filename = geometry[0].attrib["filename"]
                        full_mesh_path = os.path.join(urdf_dir, mesh_filename)
                        mesh = as_mesh(trimesh.load(full_mesh_path))
                        scale = np.fromstring(geometry[0].attrib.get("scale", "1 1 1"), sep=" ")
                        mesh.apply_scale(scale)
                        link_mesh.append(apply_transform(mesh, xyz, rotation))
                    else:  # Handle primitive shapes
                        mesh = create_primitive_mesh(geometry[0], xyz, rotation)
                        link_mesh.append(mesh)

                except Exception as e:
                    print(f"Failed to load geometry for {link_name}: {e}")
            if len(link_mesh) == 0:
                continue
            elif len(link_mesh) > 1:
                # link_geometries[link_name] = trimesh.boolean.union(
                #     link_mesh, engine="blender"
                # )  # union has some issues here
                link_trimesh = as_mesh(trimesh.Scene(link_mesh))
            elif len(link_mesh) == 1:
                link_trimesh = link_mesh[0]
                
            if link_color is not None:
                link_trimesh.visual.face_colors = np.array(link_color)
            link_geometries[link_name] = link_trimesh

    return link_geometries


def load_collision_balls_radius(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    link_name_to_collision_balls_radius = {}

    for link in root.findall("link"):
        link_name = link.attrib["name"]
        
        if "_collision_ball_" in link_name:
            link_name_stripped = link_name.split("_collision_ball_")[0]
            if link_name_stripped not in list(link_name_to_collision_balls_radius.keys()):
                link_name_to_collision_balls_radius[link_name_stripped] = []

            collision_shape =  link.findall(".//" + "collision")[0]
            geometry = collision_shape.find("geometry")[0]
            
            if geometry.tag.endswith("sphere"):
                radius = float(geometry.attrib["radius"])
                link_name_to_collision_balls_radius[link_name_stripped].append(radius)
            else:
                raise ValueError(f"Unsupported geometry type: {geometry.tag}")

    return link_name_to_collision_balls_radius

def joint_values_order_mapping(
        src_joint_values,
        src_joint_names_order: List[str],
        target_joint_names_order: List[str]
): 
    """
    - args:
        src_joint_values: np.array, shape=(b, n) or (n,), joint values in source order
        src_joint_names_order: list, joint names in source order
        target_joint_names_order: list, joint names in target order
    """
    if len(src_joint_values.shape) == 1:
        if isinstance(src_joint_values, torch.Tensor):
            src_joint_values = src_joint_values.unsqueeze(0)
        elif isinstance(src_joint_values, np.ndarray):
            src_joint_values = src_joint_values[None, :]
        else:
            raise ValueError("Unsupported type of src_joint_values")
    
    joint_mapping = np.array([src_joint_names_order.index(name) for name in target_joint_names_order]).astype(int)
    target_joint_values = src_joint_values[:, joint_mapping]

    if target_joint_values.shape[0] == 1: 
        target_joint_values = target_joint_values[0]
        
    return target_joint_values

def get_hand_per_link_collision_pc_and_normals(
        hand,
        n_samples=100,
        n_save_point: Union[int, List[int]] = 1024,
    ):

    sampled_joint_values = hand.sample_random_joint_values(n=n_samples)
    sampled_joint_values_np = sampled_joint_values.detach().cpu().numpy()
    collision_free_joint_values = []
    for i in range(n_samples):
        joint_values = sampled_joint_values[i]
        is_collision_flag, collision_points = hand.get_collision_results_pb(
            joint_values
        )
        if not is_collision_flag:
            collision_free_joint_values.append(joint_values)

    link_name_to_link_pc = {key: [] for key in hand.link_collisions_dict.keys()}
    link_name_to_link_normal = {key: [] for key in hand.link_collisions_dict.keys()}

    for collision_free_joint_value in collision_free_joint_values:
        current_status = hand.pk_chain.forward_kinematics(
            th=hand.ensure_tensor(collision_free_joint_value)
        )

        return_trimesh_dict = hand.get_hand_trimesh(
            collision_free_joint_value, visual=False, collision=True
        )
        visual_mesh = as_mesh(return_trimesh_dict["collision"])

        faces_range_dict = return_trimesh_dict["faces_range_dict"]
        locations, index_tri, normals = ray_mesh_intersections(visual_mesh, ray_interval=0.003)

        temp_link_name_to_pc_index = {key: [] for key in hand.link_collisions_dict.keys()}
        for i, face_index in enumerate(index_tri):
            for link_name, (start, end) in faces_range_dict.items():
                if start <= face_index < end:
                    temp_link_name_to_pc_index[link_name].append(i)
                    break

        for link_name, pc_index in temp_link_name_to_pc_index.items():
            link_pc_world = locations[pc_index]
            link_pc_world_h = np.hstack((link_pc_world, np.ones((link_pc_world.shape[0], 1))))
            link_normal_world = normals[pc_index]
            link_transform = (
                current_status[link_name]
                .get_matrix()
                .detach()
                .cpu()
                .numpy()
                .reshape(4, 4)
            )
            link_transform_inv = np.linalg.inv(link_transform)
            link_pc_local = np.dot(link_transform_inv, link_pc_world_h.T).T[:, :3]  # _local means in the frame of the
            link_normal_local = np.dot(link_transform_inv[:3, :3], link_normal_world.T).T
            link_name_to_link_pc[link_name].append(link_pc_local)
            link_name_to_link_normal[link_name].append(link_normal_local)

    # concatenate all the point clouds and normals
    for link_name, link_pc_list in link_name_to_link_pc.items():
        link_name_to_link_pc[link_name] = np.vstack(link_pc_list)
        link_name_to_link_normal[link_name] = np.vstack(link_name_to_link_normal[link_name])

    # do fps sampling on the point clouds
    # so that the sum of all the point clouds of all links is save_n_point
    if isinstance(n_save_point, int):
        n_save_point = [n_save_point]
    return_dict = {}
    for n_save_point_current in n_save_point:
        n_all_pc = 0
        for link_name, link_pc in link_name_to_link_pc.items():
            n_all_pc += link_pc.shape[0]
        fps_ratio = n_save_point_current / n_all_pc
        n_link_pc = {key: int(fps_ratio * value.shape[0]) for key, value in link_name_to_link_pc.items()}
        n_save_pc_current = 0
        link_pc_to_save = {key: [] for key in hand.link_collisions_dict.keys()}
        link_normal_to_save = {key: [] for key in hand.link_collisions_dict.keys()}
        for link_name, link_pc in link_name_to_link_pc.items():
            n_link_save_pc = n_link_pc[link_name]
            if link_name == list(link_name_to_link_pc.keys())[-1]:
                n_link_save_pc = n_save_point_current - n_save_pc_current
            # do fps sampling
            if n_link_save_pc < 1:
                continue
            link_fps_samples_idx = fpsample.fps_npdu_kdtree_sampling(link_pc, n_link_save_pc)  
            # NOTE: don't use fpsample.bucket_fps_kdtree_sampling and fpsample.bucket_fps_kdline_sampling
            link_pc_to_save[link_name] = link_pc[link_fps_samples_idx]
            link_normal_to_save[link_name] = link_name_to_link_normal[link_name][link_fps_samples_idx]
            n_save_pc_current += len(link_fps_samples_idx)

        # remove the key with empty values
        link_pc_to_save = {key: value for key, value in link_pc_to_save.items() if len(value) > 0}
        link_normal_to_save = {key: value for key, value in link_normal_to_save.items() if len(value) > 0}
        return_dict[n_save_point_current] = (link_pc_to_save, link_normal_to_save)

    return return_dict