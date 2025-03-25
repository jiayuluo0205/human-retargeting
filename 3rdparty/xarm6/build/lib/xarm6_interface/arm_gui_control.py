import time
import numpy as np
from xarm6_interface.arm_pk import XArm6WOEE
from xarm6_interface.arm_rw import XArm6RealWorld
from xarm6_interface.utils import vis_robot_frames

if __name__ == "__main__":

    arm = XArm6WOEE()
    arm_rw = XArm6RealWorld()

    import viser 

    sv = viser.ViserServer()

    ''' urdf test '''
    joint_gui_handles = []

    def update_robot_trimesh(joint_values):
        trimesh_dict = arm.get_state_trimesh(
            joint_values,
            visual=True,
            collision=True,
        )
        visual_mesh = trimesh_dict["visual"]
        collision_mesh = trimesh_dict["collision"]
        sv.scene.add_mesh_trimesh("visual_mesh", visual_mesh)
        sv.scene.add_mesh_trimesh("collision_mesh", collision_mesh)
        vis_robot_frames(sv, arm.current_status, axes_length=0.15, axes_radius=0.005)
        arm_rw.set_joint_values(joint_values)

    init_arm_joint_values = arm_rw.get_joint_values()
    for joint_name, lower, upper, initial_angle in zip(
        arm.actuated_joint_names, arm.lower_joint_limits_np, arm.upper_joint_limits_np, init_arm_joint_values
    ):
        lower = float(lower) if lower is not None else -np.pi
        upper = float(upper) if upper is not None else np.pi
        slider = sv.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=0.05,
            initial_value=float(initial_angle),
        )
        slider.on_update(  # When sliders move, we update the URDF configuration.
            lambda _: update_robot_trimesh([gui.value for gui in joint_gui_handles])
        )
        joint_gui_handles.append(slider)

    update_robot_trimesh(init_arm_joint_values)
    while True:
        time.sleep(1)

