from leaphand_rw.leaphand_rw import LeapNode
from xarm6_interface.arm_mplib import min_jerk_interpolator_with_alpha
import numpy as np
import time

if __name__ == "__main__":
    
    leaphand = LeapNode(torque_enable=True)
    # end_joint_positions = leaphand.open_pos
    # achieve_time = 7.0
    # leaphand.go_to_a_pos_during(end_joint_positions, achieve_time)
    
    while True:
        time.sleep(1)
#         pos = leaphand
#         print(pos = leaphand.read_pos())
#         # [3.2274957 4.2706027 4.9133406 4.0297675 3.1354568 4.693981  4.1662917
# #  5.005379  3.041884  4.246059  4.9746995 3.8901753 5.1526413 4.8105636
# #  3.3808937 5.013049 ]

#         print(leaphand.read_vel())

#         print(leaphand.read_cur())
        leaphand.set_leap(leaphand.read_pos())

    