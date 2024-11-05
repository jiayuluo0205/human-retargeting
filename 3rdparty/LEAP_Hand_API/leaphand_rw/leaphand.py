#!/usr/bin/env python3
import numpy as np

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import time

class LeapNode:
    def __init__(self, left=False):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 700
        self.kI = 0
        self.kD = 300
        self.curr_lim = 100
        self.lefthand = left
        if left:
            pos = lhu.allegro_to_LEAPhand(np.zeros(16))
            pos = self.right_pose_to_left(pos)
            print(f'Init leaphand to left init pose: {pos}')
        else:
            pos = lhu.allegro_to_LEAPhand(np.zeros(16))
            print(f'Init leaphand to right init pose: {pos}')
        self.prev_pos = self.pos = self.curr_pos = pos

        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        if left:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 3000000)
                self.dxl_client.connect()
                print('Connected to /dev/ttyUSB2')
            except Exception:
                try:
                    self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 3000000)
                    self.dxl_client.connect()
                    print('Connected to /dev/ttyUSB1')
                except Exception:
                    try:
                        self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 3000000)
                        self.dxl_client.connect()
                        print('Connected to /dev/ttyUSB0')
                    except Exception:
                        pass

        else:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB2', 3000000)
                self.dxl_client.connect()
                print('Connected to /dev/ttyUSB2')
            except Exception:
                try:
                    self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 3000000)
                    self.dxl_client.connect()
                    print('Connected to /dev/ttyUSB1')
                except Exception:
                    try:
                        self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 3000000)
                        self.dxl_client.connect()
                        print('Connected to /dev/ttyUSB0')
                    except Exception:
                        pass
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def right_pose_to_left(self, pose):
        for i in [0,4,8]:
            pose[i] = 6.28 - pose[i]
        pose[12] = 9.42 - pose[12]
        pose[13] = 9.42 - pose[13]
        return pose

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        if self.lefthand:
            self.curr_pos = self.right_pose_to_left(self.curr_pos)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        if self.lefthand:
            self.curr_pos = self.right_pose_to_left(self.curr_pos)
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