import matplotlib.pyplot as plt
import numpy as np
import torch
import nvdiffrast.torch as dr
import trimesh

def K_to_projection(K, H, W, n=0.001, f=10.0):  # near and far plane
    fu, fv, cu, cv = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    proj = torch.tensor([[2 * fu / W, 0, -2 * cu / W + 1, 0],
                         [0, 2 * fv / H, 2 * cv / H - 1, 0],
                         [0, 0, -(f + n) / (f - n), -2 * f * n / (f - n)],
                         [0, 0, -1, 0]]).cuda().float()
    return proj

def transform_pos(mtx, pos):
    ''' mtx: 4x4, pos: Nx3 
    mtx means the transformation from world to camera space
    pos means the position in world space
    '''
    t_mtx = torch.from_numpy(mtx).cuda() if isinstance(mtx, np.ndarray) else mtx
    # (x,y,z) -> (x,y,z,1)
    posw = torch.cat([pos, torch.ones([pos.shape[0], 1]).cuda()], axis=1)
    return torch.matmul(posw, t_mtx.t())[None, ...]

class NVDiffrastRenderer:
    def __init__(self, image_size):
        """
        image_size: H,W
        """
        # self.
        self.H, self.W = image_size
        self.resolution = image_size
        blender2opencv = torch.tensor([[1, 0, 0, 0],
                                       [0, -1, 0, 0],
                                       [0, 0, -1, 0],
                                       [0, 0, 0, 1]]).float().cuda()
        self.opencv2blender = torch.inverse(blender2opencv)
        self.glctx = dr.RasterizeCudaContext()

    def render_mask(self, verts, faces, K, object_pose, anti_aliasing=True):
        """
        @param verts: N,3, torch.tensor, float, cuda
        @param faces: M,3, torch.tensor, int32, cuda
        @param K: 3,3 torch.tensor, float ,cuda
        @param object_pose: 4,4 torch.tensor, float, cuda
        @return: mask: 0 to 1, HxW torch.cuda.FloatTensor
        """
        proj = K_to_projection(K, self.H, self.W)

        pose = self.opencv2blender @ object_pose

        pos_clip = transform_pos(proj @ pose, verts)

        rast_out, _ = dr.rasterize(self.glctx, pos_clip, faces, resolution=self.resolution)
        if anti_aliasing:
            vtx_color = torch.ones(verts.shape, dtype=torch.float, device=verts.device)
            color, _ = dr.interpolate(vtx_color[None, ...], rast_out, faces)
            color = dr.antialias(color, rast_out, pos_clip, faces)
            mask = color[0, :, :, 0]
        else:
            mask = rast_out[0, :, :, 2] > 0
        mask = torch.flip(mask, dims=[0])
        return mask

    def batch_render_mask(self, verts, faces, K, anti_aliasing=True):
        """
        @param batch_verts: N,3, torch.tensor, float, cuda
        @param batch_faces: M,3, torch.tensor, int32, cuda
        @param K: 3,3 torch.tensor, float ,cuda
        # @param batch_object_poses: N,4,4 torch.tensor, float, cuda
        @return: mask: 0 to 1, HxW torch.cuda.FloatTensor
        """
        proj = K_to_projection(K, self.H, self.W)

        pose = self.opencv2blender

        pos_clip = transform_pos(proj @ pose, verts)

        rast_out, _ = dr.rasterize(self.glctx, pos_clip, faces, resolution=self.resolution)
        if anti_aliasing:
            vtx_color = torch.ones(verts.shape, dtype=torch.float, device=verts.device)
            color, _ = dr.interpolate(vtx_color[None, ...], rast_out, faces)
            color = dr.antialias(color, rast_out, pos_clip, faces)
            mask = color[0, :, :, 0]
        else:
            mask = rast_out[0, :, :, 2] > 0
        mask = torch.flip(mask, dims=[0])
        return mask



if __name__ == '__main__':

    from loguru import logger as lgr
    from scipy.spatial.transform import Rotation as R
    from xarm6_interface.utils.realsense import Realsense
    from xarm6_interface.arm_pk import XArm6WOEE
    from xarm6_interface.utils import as_mesh
 
    # setup camera 
    camera = Realsense()
    H, W = camera.h, camera.w
    K = camera.K
    # handcraft a transformation matrix
    X_BaseCamera = np.eye(4)
    X_BaseCamera[0:3, 0:3] = R.from_euler('x', -90-65, degrees=True).as_matrix()
    X_BaseCamera[0:3, 3] = np.array([0.5, -0.5, 1.1])
    X_CameraBase = np.linalg.inv(X_BaseCamera)
    lgr.info("X_BaseCamera: \n{}".format(X_BaseCamera))
    lgr.info("X_CameraBase: \n{}".format(X_CameraBase))

    arm = XArm6WOEE()
    arm_visual_mesh = as_mesh(arm.get_state_trimesh(arm.reference_joint_values)['visual'])

    renderer = NVDiffrastRenderer([H, W])
    mask = renderer.render_mask(torch.from_numpy(arm_visual_mesh.vertices).cuda().float(),
                                torch.from_numpy(arm_visual_mesh.faces).cuda().int(),
                                torch.from_numpy(K).cuda().float(),
                                torch.from_numpy(X_CameraBase).cuda().float())
    
    plt.imshow(mask.detach().cpu())
    plt.show()

