import numpy as np 
from diff_robot_hand.hand_model import LeapHandRight
from diff_robot_hand.utils.mesh_and_urdf_utils import joint_values_order_mapping
import time 
import viser 
from leaphand_rw import LeapNode
from loguru import logger as lgr    

def leap_from_rw_to_sim(rw_joint_values, sim_joint_orders):
    rw_joint_need_a_bias_of = np.pi * np.array([
        -0.5, -1, -1, -1, -0.5, -1, -1, -1, -0.5, -1, -0.5  , -1, -1, -1, -1, -1
    ])
    rw_joint_names = [
        "9", "10", "11", "4", "5", "6", "7", "0", "1", "2", "3", "12", "13", "14", "8", "15"
    ]
    rw_joint_values = np.array(rw_joint_values)
    ret_joints = rw_joint_values + rw_joint_need_a_bias_of
    sim_joint_values = joint_values_order_mapping(ret_joints, rw_joint_names, sim_joint_orders) 
    return sim_joint_values

def leap_from_sim_to_rw(sim_joint_values, sim_joint_orders):
    rw_joint_need_a_bias_of = np.pi * np.array([
        -0.5, -1, -1, -1, -0.5, -1, -1, -1, -0.5, -1, -0.5  , -1, -1, -1, -1, -1
    ])
    rw_joint_names = [
        "9", "10", "11", "4", "5", "6", "7", "0", "1", "2", "3", "12", "13", "14", "8", "15"
    ]
    sim_joint_values = np.array(sim_joint_values)
    ret_joints = joint_values_order_mapping(sim_joint_values, sim_joint_orders, rw_joint_names) 
    return ret_joints - rw_joint_need_a_bias_of

def update_hand_trimesh(sv, hand, joint_values):                
    trimesh_dict = hand.get_hand_trimesh(
        joint_values,
        visual=True,
        collision=False,
        balls=False,
    )
    hand_visual_mesh = trimesh_dict["visual"]
    sv.scene.add_mesh_trimesh("hand_visual_mesh", hand_visual_mesh)


def add_hand_controller(sv:viser.ViserServer, hand,):
    gui_joints = []
    initial_angles = hand.reference_joint_values.detach().cpu().numpy()[0].tolist()
    for joint_name, lower, upper, initial_angle in zip(
        hand.actuated_joint_names, hand.lower_joint_limits.detach().cpu().numpy()[0], hand.upper_joint_limits.detach().cpu().numpy()[0], initial_angles
    ):
        lower = float(lower) if lower is not None else -np.pi
        upper = float(upper) if upper is not None else np.pi
        if "translation" not in joint_name:
            slider = sv.gui.add_slider(
                label=joint_name,
                min=lower,
                max=upper,
                step=0.1,
                initial_value=initial_angle
            )
        else:
            slider = sv.gui.add_slider(
                label=joint_name,
                min=-0.2,
                max=0.2,
                step=0.002,
                initial_value=initial_angle,
            )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: update_hand_trimesh(sv, hand, np.array([gui.value for gui in gui_joints]))
        )
        gui_joints.append(slider)
    return gui_joints
        
if __name__ == "__main__":
    rw_hand = LeapNode()    
    
    sim_hand = LeapHandRight(load_visual_mesh=True, load_col_mesh=False, load_balls_urdf=False, load_n_collision_point=0)

    sv = viser.ViserServer()
    

    gui_joints = add_hand_controller(sv, sim_hand)
    
    
    go_button = sv.gui.add_button("GO!") 
    go_button.on_click(
        lambda _: rw_hand.set_leap((leap_from_sim_to_rw(np.array([gui.value for gui in gui_joints]), sim_hand.actuated_joint_names)))
    )
    
    while True: 
        time.sleep(0.1)
    