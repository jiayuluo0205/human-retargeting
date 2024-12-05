import numpy as np
import pyrender
import trimesh
from body_visualizer.tools.vis_tools import colors
from human_body_prior.body_model.body_model import BodyModel
from human_body_prior.tools.omni_tools import copy2cpu as c2c
import torch

comp_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

bdata = np.load('3rdparty/wholebody/amass/support_data/github_data/dmpl_sample.npz')
bm_fname = '3rdparty/wholebody/amass/support_data/body_models/smplh/female/model.npz'
dmpl_fname = '3rdparty/wholebody/amass/support_data/body_models/dmpls/female/model.npz'

num_betas=16
num_dmpls=8

bm = BodyModel(bm_fname=bm_fname, model_type="smplh", num_betas=16, num_dmpls=8, dmpl_fname=dmpl_fname).to(comp_device)
faces = c2c(bm.f)

time_length = len(bdata['trans'])

body_parms = {
    'root_orient': torch.Tensor(bdata['poses'][:, :3]).to(comp_device), # controls the global root orientation
    'pose_body': torch.Tensor(bdata['poses'][:, 3:66]).to(comp_device), # controls the body
    'pose_hand': torch.Tensor(bdata['poses'][:, 66:]).to(comp_device), # controls the finger articulation
    'trans': torch.Tensor(bdata['trans']).to(comp_device), # controls the global body position
    'betas': torch.Tensor(np.repeat(bdata['betas'][:num_betas][np.newaxis], repeats=time_length, axis=0)).to(comp_device), # controls the body shape. Body shape is static
    'dmpls': torch.Tensor(bdata['dmpls'][:, :num_dmpls]).to(comp_device) # controls soft tissue dynamics
}
print('Body parameter vector shapes: \n{}'.format(' \n'.join(['{}: {}'.format(k,v.shape) for k,v in body_parms.items()])))
print('time_length = {}'.format(time_length))

torch.set_printoptions(threshold=float('inf'))
print(body_parms['root_orient'][0])
print(body_parms['pose_body'][0])

body_pose_beta = bm(**{k:v for k,v in body_parms.items() if k in ['pose_body', 'betas']})
print(body_pose_beta.v.shape)

body_mesh = trimesh.Trimesh(
        vertices=c2c(body_pose_beta.v[0]), 
        faces=faces, 
        vertex_colors=np.tile(colors['grey'], (6890, 1))
        )

pyrender_mesh = pyrender.Mesh.from_trimesh(body_mesh)

scene = pyrender.Scene(bg_color=colors['white'], ambient_light=(0.3, 0.3, 0.3))
mesh_node = scene.add(pyrender_mesh)
print(scene.get_pose(mesh_node))
viewer = pyrender.Viewer(scene, use_raymond_lighting=True, viewport_size=(1600, 1600), cull_faces=False, run_in_thread=True)

# while True:
#     body_mesh = trimesh.Trimesh(
#         vertices=c2c(body_pose_beta.v[0]), 
#         faces=faces, 
#         vertex_colors=np.tile(colors['grey'], (6890, 1))
#         )
#     pyrender_mesh = pyrender.Mesh.from_trimesh(body_mesh)
#     scene.remove_node(mesh_node)
#     mesh_node = scene.add(pyrender_mesh)