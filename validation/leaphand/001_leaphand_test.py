from leaphand_rw.leaphand_rw import LeapNode
import time
import numpy as np

leap_hand = LeapNode()
while True:
    x = time.time()
    # leap_hand.set_allegro([-0.0, -0.22, -0.0, -0.0, -0.0, 0.0004, 0.03207547169811321, 0.0325, -0.0, 0.11714285714285713, -0.0, -0.0, 0.06333333333333332, 0.29500000000000004, 0.096875, -0.0])
    leap_hand.set_allegro(np.zeros(16))
    str(leap_hand.read_pos_vel_cur())  
    print(time.time() - x)