import numpy as np

from leaphand_rw.leap_hand_utils.dynamixel_client import *
import leaphand_rw.leap_hand_utils.leap_hand_utils as lhu
import time
#######################################################
"""This can control and query the LEAP Hand

I recommend you only query when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

#Allegro hand conventions:
#0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
#http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

#LEAP hand conventions:
#180 is flat out for the index, middle, ring, fingers, and positive is closing more and more.

"""
from diff_robot_hand.utils.mesh_and_urdf_utils import joint_values_order_mapping

def leap_from_rw_to_sim(rw_joint_values, sim_joint_orders):
    # rw_joint_need_a_bias_of = np.pi * np.array([
    #     -0.5, -1, -1, -1, -0.5, -1, -1, -1, -0.5, -1, -0.5  , -1, -1, -1, -1, -1
    # ])
    rw_joint_need_a_bias_of = np.pi * np.array([
        # -0.5, -1, -1, -1, -0.5, -1, -1, -1, -0.5, -1, -0.5  , -1, -1, -1, -1, -1
        -0.5, -1, -1, -1, 
        -0.5, -1, -1, -1, 
        0.0, -1, -0.5, -1, 
        -1.0, -1, -1, -1
    ])
    rw_joint_names = [
        "9", "10", "11", "4", "5", "6", "7", "0", "1", "2", "3", "12", "13", "14", "8", "15"
    ]
    rw_joint_values = np.array(rw_joint_values)
    ret_joints = rw_joint_values + rw_joint_need_a_bias_of
    sim_joint_values = joint_values_order_mapping(ret_joints, rw_joint_names, sim_joint_orders) 
    return sim_joint_values

def leap_from_sim_to_rw(sim_joint_values, sim_joint_orders):
    # rw_joint_need_a_bias_of = np.pi * np.array([
    #     -0.5, -1, -1, -1, -0.5, -1, -1, -1, -0.5, -1, -0.5  , -1, -1, -1, -1, -1
    # ])
    rw_joint_need_a_bias_of = np.pi * np.array([
        # -0.5, -1, -1, -1, -0.5, -1, -1, -1, -0.5, -1, -0.5  , -1, -1, -1, -1, -1
        -0.5, -1, -1, -1, 
        -0.5, -1, -1, -1, 
        0.0, -1, -0.5, -1, 
        -1.0, -1, -1, -1
    ])
    rw_joint_names = [
        "9", "10", "11", "4", "5", "6", "7", "0", "1", "2", "3", "12", "13", "14", "8", "15"
    ]
    sim_joint_values = np.array(sim_joint_values)
    ret_joints = joint_values_order_mapping(sim_joint_values, sim_joint_orders, rw_joint_names) 
    return ret_joints - rw_joint_need_a_bias_of

def min_jerk_interpolator_with_alpha(waypt_joint_values_np, planner_timestep, cmd_timestep, alpha=0.2):
    """
    Min Jerk Interpolator with alpha to generate smooth trajectory command values.
    
    Args:
    - waypt_joint_values_np: np.ndarray of shape (n_sparse_wp, n_dof), waypoint joint values.
    - planner_timestep: float, the timestep of the planner, e.g., 1.0/20.0.
    - cmd_timestep: float, the timestep of the command, e.g., 1.0/500.0.
    - alpha: float, tuning parameter for the interpolation interval (0 < alpha <= 1), default is 0.33.
    
    Returns:
    - cmd_joint_values_np: np.ndarray, interpolated joint values for each command timestep.
    """
    
    n_sparse_wp, n_dof = waypt_joint_values_np.shape
    n_steps = int(planner_timestep / cmd_timestep)  # Number of interpolation steps

    # Calculate time fractions for interpolation (t')
    t = np.linspace(0, 1, n_steps)
    t_prime = np.clip(t / alpha, 0, 1)  # Apply alpha scaling and clipping

    # Min jerk interpolation formula using t'
    t_hat = 10 * t_prime**3 - 15 * t_prime**4 + 6 * t_prime**5  

    # Initialize the array for command joint values
    cmd_joint_values_np = []

    # Vectorized interpolation between waypoints
    for i in range(n_sparse_wp - 1):
        start_wp = waypt_joint_values_np[i]
        end_wp = waypt_joint_values_np[i + 1]
        
        # Interpolating values for the current segment using broadcasting
        interpolated_values = (1 - t_hat[:, np.newaxis]) * start_wp + t_hat[:, np.newaxis] * end_wp
        cmd_joint_values_np.append(interpolated_values)

    # Stack all interpolated segments together
    cmd_joint_values_np = np.vstack(cmd_joint_values_np)
    
    return cmd_joint_values_np

########################################################
class LeapNode:
    def __init__(self,
                kP=600,
                kI = 0,
                kD = 200,
                curr_lim=150,
                # kP=1000,
                # kD=100,
                # curr_lim=150,
                cmd_timestep=1.0/50.0,
                torque_enable = True):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.curr_lim = curr_lim
        self.cmd_timestep = cmd_timestep
        self.pos_lim = 0.1
        self.torque_enable = torque_enable
        # right hand
        # self.open_pos = np.array([1.57079633, 3.14159265, 3.14159265, 3.14159265, 
        #                           1.57079633, 3.14159265, 3.14159265, 3.14159265, 
        #                           1.57079633, 3.14159265, 1.57079633, 3.14159265, 
        #                           3.14159265, 3.14159265, 3.14159265, 3.14159265])
        self.open_pos = np.array([1.57079633, 3.14159265, 3.14159265, 3.14159265, 
                                  1.57079633, 3.14159265, 3.14159265, 3.14159265, 
                                  0.0, 3.14159265, 1.57079633, 3.14159265, 
                                  3.14159265, 3.14159265, 3.14159265, 3.14159265])
        
        # reset the thumb pose
        self.open_pos = np.array(
                                [1.6827769, 2.888486, 3.04035, 3.044952, 
                                1.6843109, 2.8209906, 3.0510879, 2.9759228,
                                0.10584468, 2.9145634, 1.4971652, 3.2520392,
                                4.6418257, 3.103243, 3.268913, 2.7841752]
                            )
        
        # # left hand
        # self.open_pos = np.array(
        #     [3.1646023, 1.6367575, 3.0618258, 3.0740974,
        #     3.181476, 1.6352235, 3.0863693, 3.044952,
        #     3.1308548, 1.6060779, 3.0940392, 3.04802,
        #     6.2785835, 4.735399, 3.2581751, 3.0081363]
        # )
        
        # You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                try:
                    self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 4000000)
                    self.dxl_client.connect()
                except Exception:
                    self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB3', 4000000)
                    self.dxl_client.connect()
                
        
        self.prev_pos = self.pos = self.curr_pos = self.dxl_client.read_pos() # let LEAP init to current position
        
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, self.torque_enable)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([3, 7, 14], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([3, 7, 14], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    
    def go_to_a_pos_during(self, pose, duration):
        start_joint_positions = self.read_pos()
        end_joint_positions = pose
        waypoints = np.array([start_joint_positions, end_joint_positions])
        achieve_time = duration
        cmd_joint_positions = min_jerk_interpolator_with_alpha(
            waypoints, achieve_time, self.cmd_timestep, alpha=0.2
        )
        for i in range(cmd_joint_positions.shape[0]):
            self.set_leap(cmd_joint_positions[i])
            time.sleep(self.cmd_timestep)
            
    def go_to_a_pos_with_curr_limit(self, pose, duration, verbose=True):
        start_joint_positions = self.read_pos()
        end_joint_positions = pose
        waypoints = np.array([start_joint_positions, end_joint_positions])
        achieve_time = duration
        cmd_joint_positions = min_jerk_interpolator_with_alpha(
            waypoints, achieve_time, self.cmd_timestep, alpha=0.2
        )
        for i in range(cmd_joint_positions.shape[0]):
            t1 = time.time()
            # limited_joint_position = cmd_joint_positions[i]
            
            # pos_now = self.read_pos()
            # for j in range(len(pos_now)):
            #     if limited_joint_position[j] - pos_now[j] > self.pos_lim:
            #         print(f"limited_joint_position[j] - pos_now[j] = {limited_joint_position[j] - pos_now[j]}")
            #         limited_joint_position[j] = pos_now[j] + self.pos_lim
            #     elif limited_joint_position[j] - pos_now[j] < -self.pos_lim:
            #         print(f"limited_joint_position[j] - pos_now[j] = {limited_joint_position[j] - pos_now[j]}")
            #         limited_joint_position[j] = pos_now[j] - self.pos_lim

            # Assuming pos_now and cmd_joint_positions are numpy arrays, and pos_lim is a scalar
            limited_joint_position = np.copy(cmd_joint_positions[i])  # Copy to avoid modifying the original data

            # Read current positions
            pos_now = self.read_pos()  # Assuming this returns a numpy array

            # Calculate the difference between limited_joint_position and pos_now
            diff = limited_joint_position - pos_now

            # Apply constraints using numpy where function
            limited_joint_position = np.where(diff > self.pos_lim, pos_now + self.pos_lim, limited_joint_position)
            limited_joint_position = np.where(diff < -self.pos_lim, pos_now - self.pos_lim, limited_joint_position)

            # Print differences where changes were made (optional for debugging)
            over_limit = np.where(diff > self.pos_lim)[0]
            under_limit = np.where(diff < -self.pos_lim)[0]

            if verbose:
                for idx in over_limit:
                    print(f"limited_joint_position[{idx}] - pos_now[{idx}] = {diff[idx]}")
                    print(f"limited_joint_position = {limited_joint_position}")
                    print(f"pos_now = {pos_now}")
                    print(f"new_diff = {limited_joint_position - pos_now}")
                for idx in under_limit:
                    print(f"limited_joint_position[{idx}] - pos_now[{idx}] = {diff[idx]}")
                    print(f"limited_joint_position = {limited_joint_position}")
                    print(f"pos_now = {pos_now}")
                    print(f"new_diff = {limited_joint_position - pos_now}")

            self.set_leap(limited_joint_position)
            
            t2 = time.time()
            time_read_pos = t2 - t1
            time_sleep = self.cmd_timestep - time_read_pos
            if time_sleep < 0:
                continue
            time.sleep(time_sleep)



            