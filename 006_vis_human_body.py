import torch
import numpy as np

from human_body_prior.tools.omni_tools import copy2cpu as c2c
from os import path as osp

support_dir = '3rdparty/wholebody/amass/support_data'

# Choose the device to run the body model on.
comp_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

amass_npz_fname = osp.join(support_dir, 'github_data/dmpl_sample.npz') # the path to body data
bdata = np.load(amass_npz_fname)

# you can set the gender manually and if it differs from data's then contact or interpenetration issues might happen
subject_gender = bdata['gender']

# print('Data keys available:%s'%list(bdata.keys()))

# print('The subject of the mocap sequence is  {}.'.format(subject_gender))

from human_body_prior.body_model.body_model import BodyModel

bm_fname = '3rdparty/wholebody/amass/support_data/body_models/smplh/female/model.npz'
dmpl_fname = '3rdparty/wholebody/amass/support_data/body_models/dmpls/female/model.npz'

num_betas = 16 # number of body parameters
num_dmpls = 8 # number of DMPL parameters

bm = BodyModel(bm_fname=bm_fname, model_type="smplh", num_betas=num_betas, num_dmpls=num_dmpls, dmpl_fname=dmpl_fname).to(comp_device)

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

# print('Body parameter vector shapes: \n{}'.format(' \n'.join(['{}: {}'.format(k,v.shape) for k,v in body_parms.items()])))
# print('time_length = {}'.format(time_length))

import trimesh
from body_visualizer.tools.vis_tools import colors
from body_visualizer.mesh.mesh_viewer import MeshViewer
from body_visualizer.mesh.sphere import points_to_spheres
from body_visualizer.tools.vis_tools import show_image
from pyrender import Scene, Viewer

imw, imh=1600, 1600
mv = MeshViewer(width=imw, height=imh, use_offscreen=True)

body_pose_beta = bm(**{k:v for k,v in body_parms.items() if k in ['pose_body', 'betas']})

def vis_body_pose_beta(fId = 0):
    body_mesh = trimesh.Trimesh(vertices=c2c(body_pose_beta.v[fId]), faces=faces, vertex_colors=np.tile(colors['grey'], (6890, 1)))
    mv.set_static_meshes([body_mesh])
    body_image = mv.render(render_wireframe=False)
    show_image(body_image)

scene = Scene(bg_color=colors['white'], ambient_light=(0.3, 0.3, 0.3))

viewer = Viewer(scene, use_raymond_lighting=True, viewport_size=(1600, 1600), cull_faces=False, run_in_thread=True)

# body_mesh = trimesh.Trimesh(vertices=c2c(body_pose_beta.v[0]), faces=faces, vertex_colors=np.tile(colors['grey'], (6890, 1)))

# print(f"body mesh: {body_mesh}")