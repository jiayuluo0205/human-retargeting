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
    # end_joint_positions = leaphand.open_pos
    # left hand
    leaphand.open_pos = np.array(
        [3.1646023, 1.6367575, 3.0618258, 3.0740974,
        3.181476, 1.6352235, 3.0863693, 3.044952,
        3.1308548, 1.6060779, 3.0940392, 3.04802,
        6.2785835, 4.735399, 3.2581751, 3.0081363]
    )
    end_joint_positions = leaphand.open_pos
    
    # test result: if achieve_time is too small (for now, smaller than 5), 
    # the speed is too fast, and the hand will crash.
    # recommend to set achieve_time to 10.
    achieve_time = 2
    leaphand.set_leap(end_joint_positions)
    time.sleep(2)
    # leaphand.go_to_a_pos_with_curr_limit(end_joint_positions, achieve_time)
    # leaphand.go_to_a_pos_during(end_joint_positions, achieve_time)
    # Close tight
    close_pos = np.array(
        [3.9024472, 3.0602918, 4.790622, 3.9085832,
        3.202952, 2.9145634, 5.0575347, 3.339476,
        2.9713209, 2.9851265, 5.0652046, 3.273515,
        0.09203885, 3.9254568, 4.8090296, 5.031457]
    )
    
    # leaphand.go_to_a_pos_with_curr_limit(close_pos, achieve_time)
    leaphand.set_leap(close_pos)
    time.sleep(2)
    
    # leaphand.go_to_a_pos_during(close_pos, achieve_time)
    
    num_break_down_motor, break_down_motor_list = count_break_down_motor(leaphand)
    print(f"num_break_down_motor = {num_break_down_motor}")
    print(f"break_down_motor_list = {break_down_motor_list}")