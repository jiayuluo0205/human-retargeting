from leaphand_rw.leaphand_rw import LeapNode
import time
import numpy as np

leap_hand = LeapNode()
while True:
    x = time.time()
    leap_hand.set_allegro(np.zeros(16))
    str(leap_hand.read_pos_vel_cur())
    #print("Position: " + )    
    print(time.time() - x)