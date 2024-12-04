from leaphand_rw.leaphand_rw import LeapNode
from xarm6_interface.arm_mplib import min_jerk_interpolator_with_alpha
import numpy as np
import time

if __name__ == "__main__":
    
    leaphand = LeapNode()
    # end_joint_positions = leaphand.open_pos
    # achieve_time = 2.0
    # leaphand.go_to_a_pos_during(end_joint_positions, achieve_time)
    
    while True:
        time.sleep(1)
    