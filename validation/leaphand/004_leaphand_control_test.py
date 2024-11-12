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
    open_pos = np.array([3.4131072, 3.1017091, 1.1013982, 1.4327381, 3.4437869, 6.2555737,
                           2.7074761, 2.9345052, 3.267379, 0.00920388, 2.6890683, 1.2317866,
                           4.4424086, 6.1988163, 3.2366996, 3.0894372])
    leaphand.set_leap(open_pos)
    # leaphand.go_to_a_pos_with_curr_limit(end_joint_positions, achieve_time)
    time.sleep(4)
    # leaphand.go_to_a_pos_during(end_joint_positions, achieve_time)
    # Close tight
    close_pos = np.array([3.2075539, 4.1172047, 3.4545248, 2.3393207, 2.8301945, 7.751205,
                             4.9455543, 3.3762918, 3.0495539, 1.6045439, 4.724661, 2.0325246,
                             4.3273597, 5.916564, 5.016117, 4.9194765])
    
    # leaphand.go_to_a_pos_with_curr_limit(close_pos, achieve_time)
    leaphand.set_leap(close_pos)
    time.sleep(5)
    # leaphand.go_to_a_pos_during(close_pos, achieve_time)
    
    num_break_down_motor, break_down_motor_list = count_break_down_motor(leaphand)
    print(f"num_break_down_motor = {num_break_down_motor}")
    print(f"break_down_motor_list = {break_down_motor_list}")