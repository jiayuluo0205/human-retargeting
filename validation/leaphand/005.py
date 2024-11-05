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
        # [1.5876701 2.969787  3.3072627 3.2474372 1.6168158 3.028078  3.112447
#  3.1385248 1.6106799 3.038816  1.5846021 3.0879033 4.562059  3.2136898
#  3.0863693 2.9973984]

        print(leaphand.read_vel())
        # [0.         0.02398082 0.         0.02398082 0.         0.02398082
#  0.         0.         0.         0.02398082 0.02398082 0.
#  0.         0.         0.         0.        ]
        print(leaphand.read_cur())

# [ 18.76   9.38   0.   -20.1   24.12  10.72  -1.34 -16.08  20.1    8.04
#   -1.34  21.44   1.34  13.4  -24.12   4.02]
    