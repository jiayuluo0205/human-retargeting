import zmq
import numpy as np
import pybullet as p
import pybullet_data
from scipy.spatial.transform import Rotation as R

def initAxis(center, rot_mat):
      rotmat = np.array(rot_mat).reshape((3, 3))
      p.addUserDebugLine(lineFromXYZ=center,lineToXYZ=center+ rotmat[:3,0] * 0.1,lineColorRGB=[1,0,0],lineWidth=10)
      p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 1] * 0.1, lineColorRGB=[0, 1, 0], lineWidth=10)
      p.addUserDebugLine(lineFromXYZ=center, lineToXYZ=center + rotmat[:3, 2] * 0.1, lineColorRGB=[0, 0, 1], lineWidth=10)


#load xarm
# physicsclient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -10)
# planeID = p.loadURDF("plane.urdf")
startPos = [0, 0, 0]
#xarmID = p.loadURDF("xarm/xarm6_robot.urdf")
#p.resetBasePositionAndOrientation(xarmID, startPos, [0,0,0,1])
#p.changeVisualShape(xarmID,6,rgbaColor=[0,0,0,1])
#xarm_EE_init = np.array(p.getLinkState(xarmID,6)[0])

context =zmq.Context()
# talk to server windows machine
print("Connecting to windows server...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://172.25.104.11:5555")

memory = []
lwrist_position = np.array([0.698, 1.52, 0.0]) # meter
# rwrist_position = [-0.698, 0.56, 0.0]
initial_position = None
initial_ori = None
initial_pose = None
joint_indices = [1, 2, 3, 4, 5, 6]

i = 0
EE_np_init = np.array([0,0,0])




## init xArm
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from scipy.spatial.transform import Slerp


ip = "192.168.1.232"

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# arm.move_gohome(wait=True)
arm.set_position(x=400, y=0, z=350, roll=-180, pitch=0, yaw=0, speed=100, is_radian=False, wait=True)
xarm_init_pos = [400, 0, 350]
xarm_init_rotmatrix = R.from_euler('ZYX', np.array([-180, 0, 0])).as_matrix()
# print('*'*50)
# print(xarm_init_rotmatrix)
# exit()
# pose = arm.get_position()
# print(f"pose = {pose}")

# print('=' * 50)
# print('version:', arm.get_version())
# print('state:', arm.get_state())
# print('cmdnum:', arm.get_cmdnum())
# print('err_warn_code:', arm.get_err_warn_code())
# print('position(°):', arm.get_position(is_radian=False))
# print('position(radian):', arm.get_position(is_radian=True))
# print('angles(°):', arm.get_servo_angle(is_radian=False))
# print('angles(radian):', arm.get_servo_angle(is_radian=True))
# print('angles(°)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=False))
# print('angles(radian)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=True))
# exit()

# set mode: cartesian online trajectory planning mode
# the running command will be interrupted when the next command is received
arm.set_mode(7)
arm.set_state(0)
time.sleep(1)

speed = 150
scale = 1.5

# for i in range(10):
#     # run on mode(7)
#     # the running command will be interrupted, and run the new command
#     arm.set_position(x=400, y=-150, z=150, roll=-180, pitch=0, yaw=0, speed=speed, wait=False)
#     time.sleep(1)
#     # the running command will be interrupted, and run the new command
#     arm.set_position(x=400, y=100, z=150, roll=-180, pitch=0, yaw=0, speed=speed, wait=False)
#     time.sleep(1)



while True:
# for i in range(20):
    socket.send(b"request")
    print(f"request sent")
    # Get from server
    pose24_byte = socket.recv()
    # print(pose24_byte)
    pose24_np = np.frombuffer(pose24_byte, dtype=np.float64)
    # # memory.append(pose24_np)
    pose24_np = np.reshape(pose24_np,[4,4])
    # print("EE_shape")
    # print(pose24_np) # wxyz
    
    EE_np = pose24_np[:3,3]
    EE_rot_np = pose24_np[:3,:3]
    
    if i == 0:
      EE_np_init = EE_np
      EE_rot_np_init = EE_rot_np
      i = 1
      continue
    else:
      EE_rel = EE_np - EE_np_init
      EE_rot = EE_rot_np.copy()
      EE_euler = R.from_matrix(EE_rot).as_euler('xyz', degrees=True)
      
    #   EE_rel_rotation = EE_rot_np @ EE_rot_np_init.transpose()
    #   EE_rel_rotation = R.from_matrix(EE_rel_rotation).as_rotvec()
    #   EE_rel_new = np.array([EE_rel_rotation[2], EE_rel_rotation[0], EE_rel_rotation[1]])
    #   EE_rel_new = R.from_rotvec(EE_rel_new).as_matrix()
    #   EE_rel_new = EE_rel_new @ xarm_init_rotmatrix
    #   EE_rot_np_new = EE_rot_np.copy()
    #   EE_rel_new_euler = R.from_matrix(EE_rel_new).as_euler('ZYX', degrees=True)
      
    xarm_EE_init = np.array([0,0,0.5])
    
    # targetPosition = EE_rel + xarm_EE_init
    # print(targetPosition)
    
    # xarm [2] --- hand [1]
    # xarm [0] --- hand [2]0
    # xarm [1] --- hand [0]
    
    xarm_target_pose = xarm_init_pos + EE_rel
    # xarm_target_pose[0] = xarm_init_pos[0] + EE_rel[2]*1000
    # xarm_target_pose[1] = xarm_init_pos[1] + EE_rel[0]*1000
    # xarm_target_pose[2] = xarm_init_pos[2] + EE_rel[1]*1000
    
    xarm_target_pose[0] = xarm_init_pos[0] - scale * EE_rel[1]*1000
    xarm_target_pose[1] = xarm_init_pos[1] + scale * EE_rel[0]*1000
    xarm_target_pose[2] = xarm_init_pos[2] + scale * EE_rel[2]*1000
    # print(xarm_target_pose)
    print(EE_euler)
    # arm.set_position(x=xarm_target_pose[0], y=xarm_target_pose[1], z=xarm_target_pose[2],
                    #  roll=EE_rel_new_euler[0], pitch=EE_rel_new_euler[1], yaw=EE_rel_new_euler[2],
                    #  speed=speed, wait=False)
    # arm.set_position(x=xarm_target_pose[0], y=xarm_target_pose[1], z=xarm_target_pose[2], 
    #                  roll=-180, pitch=0, yaw=0, 
    #                  speed=speed, wait=False)
    
    arm.set_position(x=xarm_target_pose[0], y=xarm_target_pose[1], z=xarm_target_pose[2], 
                      roll=EE_euler[0]+90, pitch=EE_euler[1], yaw=EE_euler[2]+45, 
                      speed=speed, wait=False)
    # arm.set_position(x=xarm_target_pose[0], y=xarm_target_pose[1], z=xarm_target_pose[2], 
    #                   roll=EE_euler[0], pitch=EE_euler[1], yaw=EE_euler[2], 
    #                   speed=speed, wait=False)
    
    
    

    #initAxis(targetPosition,np.eye(3))#pose24_np[:3,:3])
    

    # targetOri = [pose24_np[1], pose24_np[2], pose24_np[3], pose24_np[0]]
    # # print(targetOri)
    # # rot_mat = p.getMatrixFromQuaternion(targetOri)
    # rot_mat = Rotation.from_quat(targetOri)
    # # rot_mat = np.array(rot_mat).reshape(3, 3)
    
    # if initial_position is None:
    #     # initial_position = rot_mat @ lwrist_position
    #     initial_position = rot_mat.apply(lwrist_position)
    #     print(f"initial position: {initial_position}")
    #     # initial_ori = Rotation.from_matrix(rot_mat).as_quat()
    #     # initial_pose = targetOri
    #     # initial_position = np.array([initial_position[2], initial_position[0], initial_position[1]])
    # else:
    #     position = rot_mat.apply(lwrist_position)
    #     # print(f"position: {position}")
    #     # position = np.array([position[2], position[0], position[1]])
    #     residual = position - initial_position
    #     residual = np.array([residual[2], residual[0], residual[1]])
    #     initial_position = position
    #     # print(initial_position)
    #     ee_position = p.getLinkState(xarmID, 6)[4]
    #     # print(f"res: {residual}")
    #     # print(f"ee_position: {ee_position}")
    #     qs_pos = p.calculateInverseKinematics(xarmID, 6, targetPosition = ee_position+residual, targetOrientation=targetOri)
    #     # print(joint_indices)
    #     # print(qs_pos)
    #     p.setJointMotorControlArray(xarmID, joint_indices, p.POSITION_CONTROL, targetPositions=qs_pos)

# print(f"{memory}")
# print(f"initial ori: {initial_ori}")
# print(f"initial pose: {initial_pose}")




