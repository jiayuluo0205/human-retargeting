from leaphand_rw.leaphand_rw import LeapNode
from xarm6_interface.arm_mplib import min_jerk_interpolator_with_alpha
import numpy as np
import time

if __name__ == "__main__":
    
    leaphand = LeapNode(torque_enable=False)
    # end_joint_positions = leaphand.open_pos
    # achieve_time = 7.0
    # leaphand.go_to_a_pos_during(end_joint_positions, achieve_time)
    
    while True:
        time.sleep(1)
        print(leaphand.read_pos())
        # [3.4131072  3.1017091  1.1013982  1.4327381  3.4437869  6.2555737
#  2.7074761  2.9345052  3.267379   0.00920388 2.6890683  1.2317866
#  4.4424086  6.1988163  3.2366996  3.0894372 ]

        print(leaphand.read_vel())

        print(leaphand.read_cur())

    