import numpy as np 
from diff_robot_hand.hand_model import LeapHandRight
from diff_robot_hand.utils.mesh_and_urdf_utils import joint_values_order_mapping
import time 
import viser 
# from leaphand_rw import LeapNode
from leaphand_rw.leaphand_rw import LeapNode, leap_from_rw_to_sim, leap_from_sim_to_rw

from loguru import logger as lgr    

if __name__ == "__main__":
    rw_hand = LeapNode()    
    sim_hand = LeapHandRight(load_visual_mesh=True, load_col_mesh=False, load_balls_urdf=False, load_n_collision_point=0)

    sv = viser.ViserServer()
    
    while True: 
        rw_joint_values = rw_hand.read_pos()
        lgr.info(f"rw_joint_values: {rw_joint_values}")
        sim_joint_values = leap_from_rw_to_sim(rw_joint_values, sim_hand.actuated_joint_names)
        sim_to_rw_joint_values = leap_from_sim_to_rw(sim_joint_values, sim_hand.actuated_joint_names)
        assert np.allclose(sim_to_rw_joint_values, rw_joint_values)
        
        hand_mesh = sim_hand.get_hand_trimesh(sim_joint_values)["visual"]
        sv.scene.add_mesh_trimesh("hand_mesh", hand_mesh)
        time.sleep(0.1)
    