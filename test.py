import pybullet as p
import time
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,0)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,2]
startOrientation = p.getQuaternionFromEuler([0,0,0])

# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# robotID = p.loadURDF("human-retargeting/assets/robots/h1_description/urdf/h1.urdf", startPos, startOrientation)
robotID = p.loadURDF("xarm/xarm6_robot.urdf")

#set the center of mass frame (loadURDF sets base link frame)
p.resetBasePositionAndOrientation(robotID, startPos, startOrientation)
# p.resetJointState(robotID, 12, 1.55)
# p.resetJointState(robotID, 16, -1.55)
# p.resetJointState(robotID, 14, 1.55)
# p.resetJointState(robotID, 18, 1.55)
num_joints = p.getNumJoints(robotID) # 24
joint_names = []
for i in range(num_joints):
    joint_names.append(p.getJointInfo(robotID, i)[1])

print(joint_names)

quaternion = [0, 0, np.sqrt(2)/2, np.sqrt(2)/2]
rot = Rotation.from_quat(quaternion).as_matrix()
print(rot @ np.array([0.689, 1.52, 0.0]))
# quaternion = [0.0, -0.0, -0.0, 1.0]
# axis, angle = p.getAxisAngleFromQuaternion(quaternion)
# q = p.getQuaternionFromAxisAngle(axis, angle)
# print(f"axis is {axis}")
# print(f"angle is {angle}")
# print(f"new quaternion is {q}")

# joint_memory = {
#     "L_Shoulder": ([1.0, 0.0, 0.0], [11, 12, 13], [(-2.87, 2.87), (-0.34, 3.11), (-1.3, 4.45)])
# }

# joint_name = "L_Shoulder"

# x1, x2, x3 = 1, 2, 3
# result = x1, x2, x3
# opt = np.array(result) - np.array(joint_memory[f"{joint_name}"][0])
# l = list(opt)
# print(opt)

# print(joint_memory[f"{joint_name}"][0])
# print(joint_memory[f"{joint_name}"][1][-1])
# print(joint_memory[f"{joint_name}"][2])


# p.setJointMotorControl2(robotID, 3, controlMode=p.POSITION_CONTROL, targetPosition=angle)
# p.setJointMotorControlArray(robotID, list(range(num_joints)), POSITION_CONTROL, targetPositions = positions)

# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(robotID)
# # print(cubePos,cubeOrn)
# p.disconnect()