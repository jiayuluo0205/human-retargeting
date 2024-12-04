from leaphand_rw.leaphand_rw import LeapNode
from xarm6_interface.arm_mplib import min_jerk_interpolator_with_alpha
import numpy as np
import time

def count_break_down_motor(leaphand: LeapNode):
    
    motor_current_list = []
    for _ in range(10):
        motor_current_list.append(leaphand.read_cur())
        time.sleep(0.1)
    motor_current_list = np.array(motor_current_list)  # shape = (10, 16)
    
    motor_current_list = motor_current_list.T
    motor_current_list = np.abs(motor_current_list)
    motor_current_list = np.mean(motor_current_list, axis=1)
    num_break_down_motor = np.sum(motor_current_list < 0.01)
    break_down_motor_list = np.where(motor_current_list < 0.01)
    
    return num_break_down_motor, break_down_motor_list

if __name__ == "__main__":
    
    leaphand = LeapNode()
    # test result: if achieve_time is too small (for now, smaller than 5), 
    # the speed is too fast, and the hand will crash.
    # recommend to set achieve_time to 10.
    # achieve_time = 5
    # open_pos = np.array([3.2934568, 3.2888548, 2.808719, 3.34868, 3.110913, 3.2919228, 2.888486,
    #              3.2474372, 2.9789908, 3.282719, 3.126253, 2.876214, 3.4146411, 4.735399,
    #              2.6185052, 3.5557675])
    time.sleep(4)
    open_pos = np.array([3.14159] * 16)
    leaphand.set_leap(open_pos)
    # leaphand.set_allegro(np.zeros(16))
    # leaphand.go_to_a_pos_with_curr_limit(end_joint_positions, achieve_time)
    time.sleep(4)
    # leaphand.go_to_a_pos_during(end_joint_positions, achieve_time)
    # Close tight
    close_pos = np.array([3.1293209, 4.471554, 3.4790685, 3.8748355, 3.1308548, 4.9210105, 3.2167578,
                 3.591049, 3.0955732, 5.1342335, 3.3931656, 3.0633597, 5.0851464, 4.3365636,
                 2.3592625, 3.7398453])
    
    # leaphand.go_to_a_pos_with_curr_limit(close_pos, achieve_time)
    leaphand.set_leap(close_pos)
    time.sleep(5)
    # leaphand.go_to_a_pos_during(close_pos, achieve_time)
    
    num_break_down_motor, break_down_motor_list = count_break_down_motor(leaphand)
    print(f"num_break_down_motor = {num_break_down_motor}")
    print(f"break_down_motor_list = {break_down_motor_list}")