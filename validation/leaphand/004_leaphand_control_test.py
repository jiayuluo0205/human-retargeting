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
    end_joint_positions = leaphand.open_pos
    # test result: if achieve_time is too small (for now, smaller than 5), 
    # the speed is too fast, and the hand will crash.
    # recommend to set achieve_time to 10.
    achieve_time = 5
    leaphand.set_leap(end_joint_positions)
    # leaphand.go_to_a_pos_with_curr_limit(end_joint_positions, achieve_time)
    time.sleep(4)
    # leaphand.go_to_a_pos_during(end_joint_positions, achieve_time)
    # Close tight
    close_pos = np.array(
        [3.7153015, 4.4347386, 3.0050683, 3.426913,  
         3.7291074 ,3.9009132 ,3.5266218,3.5741751 ,
         3.8058064 -1.57,3.517418  ,2.3239808 ,4.31202 ,  
         3.2106218, 2.475845,3.1507967 ,4.9854374]
    )
    open_pos = np.array(
        [1.57079633, 3.14159265, 3.14159265, 3.14159265, 
        1.57079633, 3.14159265, 3.14159265, 3.14159265, 
        0.0, 3.14159265, 1.57079633, 3.14159265, 
        3.14159265, 3.14159265, 3.14159265, 3.14159265]
    )
    close_pos = np.array(
        [3.6907578, 3.360952, 4.543651, 3.2888548,
         3.8165443, 3.433049, 3.83035, 3.4667966, 
         8.441496 - 3.1415926 * 2, 3.6309326, 1.9849712, 3.7168355, 
         5.1326995, 4.143282, 3.262777, 3.8640976]
    )
    close_pos = np.array(
        [3.6968937, 3.109379, 4.4270687, 3.2060199,
         3.7183695, 3.2382333, 4.3657093, 3.19835, 
         1.9788352, 3.8502917, 2.072408, 3.0648937, 
         5.641981, 4.3196898, 3.2581751, 4.6586995]
    )
    
    # leaphand.go_to_a_pos_with_curr_limit(close_pos, achieve_time)
    leaphand.set_leap(close_pos)
    time.sleep(5)
    # leaphand.go_to_a_pos_during(close_pos, achieve_time)
    
    num_break_down_motor, break_down_motor_list = count_break_down_motor(leaphand)
    print(f"num_break_down_motor = {num_break_down_motor}")
    print(f"break_down_motor_list = {break_down_motor_list}")