import time
import numpy as np
from loguru import logger as lgr
from leaphand_rw.leaphand_rw import LeapNode
from matplotlib import pyplot as plt

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

# Plotting function for each motor's data
def plot_motor_data(timesteps, data_list, ylabel, title):
    plt.figure(figsize=(10, 6))
    for i in range(data_list.shape[1]):  # Loop over 16 motors
        plt.plot(timesteps, data_list[:, i], label=f'Motor {i+1}')
    
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend(loc='best', bbox_to_anchor=(1.05, 1), fontsize='small')
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    close_pos = np.array(
        [3.7153015, 4.4347386, 3.0050683, 3.426913,  
         3.7291074 ,3.9009132 ,3.5266218,3.5741751 ,
         3.8058064-1.57, 3.517418  ,2.3239808 ,4.31202 ,  
         3.2106218, 2.475845,3.1507967 ,4.9854374]
    )
    
    # test 1 : default (recommended) parameters
    
    grasp_time = 2
    check_timestep = 0.001
    kP = 600
    kD = 200
    curr_lim = 350
    cmd_timestep = 1.0/500.0
    
    leaphand = LeapNode(
        kP=kP,
        kD=kD,
        curr_lim=curr_lim,
        cmd_timestep=cmd_timestep,
    )

    leaphand.set_leap(leaphand.open_pos)
    num_check = grasp_time // check_timestep
    check_motor_current_list = []
    check_motor_position_list = []
    check_motor_velocity_list = []
    for _ in range(int(num_check)):
        check_motor_current_list.append(leaphand.read_cur())
        check_motor_position_list.append(leaphand.read_pos())
        check_motor_velocity_list.append(leaphand.read_vel())
        time.sleep(check_timestep)
        
    leaphand.set_leap(close_pos)
    for _ in range(int(num_check)):
        check_motor_current_list.append(leaphand.read_cur())
        check_motor_position_list.append(leaphand.read_pos())
        check_motor_velocity_list.append(leaphand.read_vel())
        time.sleep(check_timestep)

    check_motor_current_list = np.array(check_motor_current_list)  # shape = (num_check*2, 16)
    check_motor_position_list = np.array(check_motor_position_list)  # shape = (num_check*2, 16)
    check_motor_velocity_list = np.array(check_motor_velocity_list)  # shape = (num_check*2, 16)
    check_timesteps = np.arange(check_motor_current_list.shape[0]) * check_timestep  # shape = (num_check*2, )
    
    num_break_down_motor, break_down_motor_list = count_break_down_motor(leaphand)
    lgr.info(f"num_break_down_motor: {num_break_down_motor}")
    lgr.info(f"break_down_motor_list: {break_down_motor_list}")
    
    
    # Plot motor current
    plot_motor_data(check_timesteps, check_motor_current_list, ylabel='Current (A)', title='Motor Currents over Time')

    # Plot motor position
    plot_motor_data(check_timesteps, check_motor_position_list, ylabel='Position (rad)', title='Motor Positions over Time')

    # Plot motor velocity
    plot_motor_data(check_timesteps, check_motor_velocity_list, ylabel='Velocity (rad/s)', title='Motor Velocities over Time')
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    # end_joint_positions = leaphand.open_pos
    
    
    
    
    
    # leaphand.go_to_a_pos_during(end_joint_positions, grasp_time)



    # leaphand.go_to_a_pos_during(close_pos, grasp_time)
    

    