import multiprocessing
import time
import tyro
import sys
sys.path.append('utils')
import rebocap_ws_sdk
from queue import Empty
from calculator import calculate_joint, joint_memory
import pybullet as p
import pybullet_data

from loguru import logger

counter = 0

def start_retargeting(queue: multiprocessing.Queue, robot_dir: str):
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,0)
    planeID = p.loadURDF("plane.urdf")
    startPos = [0,0,2]
    startOrientation = p.getQuaternionFromEuler([0,0,0])

    robotID = p.loadURDF(robot_dir)
    p.resetBasePositionAndOrientation(robotID, startPos, startOrientation)
    # set TPose of robot
    # left_shoulder_row_joint 1.55
    p.resetJointState(robotID, 12, 1.55)
    # right_shoulder_row_joint -1.55
    p.resetJointState(robotID, 16, -1.55)
    # elbow 1.55
    p.resetJointState(robotID, 14, 1.55)
    p.resetJointState(robotID, 18, 1.55)

    retargeting_joints_smpl = ["Pelvis",  "L_Hip",  "R_Hip",  "Spine1",  
    "L_Knee",  "R_Knee",  "Spine2",  "L_Ankle",  "R_Ankle",  
    "Spine3","L_Foot",  "R_Foot",  "Neck",  "L_Collar",  
    "R_Collar",  "Head","L_Shoulder",  "R_Shoulder",  "L_Elbow",  
    "R_Elbow",  "L_Wrist", "R_Wrist",  "L_Hand",  "R_Hand"
    ]
    # num_joints = p.getNumJoints(robotID)
    # joint_names = [p.getJointInfo(robotID, idx)[1] for idx in range(num_joints)]
    # h1 humanoid only has 19 joints, the 5 rest are fixed
    # [b'left_hip_yaw_joint', b'left_hip_roll_joint', b'left_hip_pitch_joint', 
    # b'left_knee_joint', b'left_ankle_joint', b'right_hip_yaw_joint', b'right_hip_roll_joint', 
    # b'right_hip_pitch_joint', b'right_knee_joint', b'right_ankle_joint', b'torso_joint', 
    # b'left_shoulder_pitch_joint', b'left_shoulder_roll_joint', b'left_shoulder_yaw_joint', 
    # b'left_elbow_joint', b'right_shoulder_pitch_joint', b'right_shoulder_roll_joint', 
    # b'right_shoulder_yaw_joint', b'right_elbow_joint', 
    # b'imu_joint', b'logo_joint', b'd435_left_imager_joint', b'd435_rgb_module_joint', b'mid360_joint']
    # joint_names = [name.decode('utf-8') for name in joint_names]
    # joint_indices = list(range(num_joints))

    # we need to find a mapping from smpl (rebocap output) to h1 humanoid (joints in 2 frames are different!)
    # idx_rebocap2pb = 

    while True:
        p.stepSimulation()
        time.sleep(1./240.)
        try:
            tran = queue.get(timeout=5)
            pose24 = queue.get(timeout=5)
        except Empty:
            logger.error(f"Fail to fetch tran and pose from device in 5 secs. Please check your device.")
            return
        
        if pose24 is None:
            logger.warning(f"human body is not detected.")
        else:
            for idx, quaternion in enumerate(pose24):
                if retargeting_joints_smpl[idx] in joint_memory.keys():
                    qs_opt = calculate_joint(robotID, quaternion, joint_memory, retargeting_joints_smpl[idx])
                    p.setJointMotorControlArray(robotID, joint_memory[f"{retargeting_joints_smpl[idx]}"][1], p.POSITION_CONTROL, targetPositions=qs_opt)

    pass

def read_rebocap(queue: multiprocessing.Queue):
    def print_debug_msg(self: rebocap_ws_sdk.RebocapWsSdk, trans, pose24, static_index, ts):
        global counter

        is_left_on_floor = 0 <= static_index <= 5
        is_right_on_floor = 6 <= static_index <= 11
        no_foot_on_ground = static_index < 0
        if counter % 60 == 0:
            print(f'timestamp:{ts}'
                  f'current coordinate_type: {self.coordinate_type.name}'
                  f'root position:{trans} left_on_floor:{is_left_on_floor}  right_on_floor:{is_right_on_floor}')
            for i in range(24):
                print(f'bone:{rebocap_ws_sdk.REBOCAP_JOINT_NAMES[i]} quaternion w,x,y,z is:{pose24[i]}')
            print('\n\n\n\n', flush=True)
        counter += 1

    # human pose callback
    def pose_msg_callback(self: rebocap_ws_sdk.RebocapWsSdk, tran: list, pose24: list, static_index: int, ts: float):
        print_debug_msg(self, tran, pose24, static_index, ts)
        pass
    
    # 异常断开，这里处理重连或报错
    def exception_close_callback(self: rebocap_ws_sdk.RebocapWsSdk):
        print("exception_close_callback")

    # 初始化sdk  这里可以选择控件坐标系， 控件坐标系目前已经测试过的有： 1. UE  2. Unity  3. Blender
    # 选择输出角度是否是 global 角度，默认是 local 角度【简单解释，global 角度不受父节点影响 local角度受父节点影响， local角度逐级相乘就是 global 角度
    sdk = rebocap_ws_sdk.RebocapWsSdk(coordinate_type=rebocap_ws_sdk.CoordinateType.DefaultCoordinate, use_global_rotation=True)
    # 设置姿态回调
    sdk.set_pose_msg_callback(pose_msg_callback)
    # 设置异常断开回调
    sdk.set_exception_close_callback(exception_close_callback)
    # 开始连接
    open_ret = sdk.open(7690)
    # 检查连接状态
    if open_ret == 0:
        print("连接成功")
    else:
        print("连接失败", open_ret)
        if open_ret == 1:
            print("连接状态错误")
        elif open_ret == 2:
            print("连接失败")
        elif open_ret == 3:
            print("认证失败")
        else:
            print("未知错误", open_ret)
        exit(1)
    # 维持启动10秒

    while sdk:
        tran, pose24, static_index, tp = sdk.get_last_msg()
        queue.put(tran)
        queue.put(pose24)
        print(f'trans is:{tran}', flush=True)
        time.sleep(1.0)
    # time.sleep(10)
    # 断开连接
    # sdk.close()


def main():
    """
    Detects human body using Rebocap and translates the human pose into humanoid pose.
    Args:
    """
    robot_dir = "human-retargeting/assets/robots/h1_description/urdf/h1.urdf"

    queue = multiprocessing.Queue(maxsize=1000)
    producer_process = multiprocessing.Process(target=read_rebocap, args=(queue))
    consumer_process = multiprocessing.Process(target=start_retargeting, args=(queue, robot_dir))

    producer_process.start()
    consumer_process.start()

    producer_process.join()
    consumer_process.join()
    time.sleep(5)

    print("done")

if __name__=="__main__":
    tyro.cli(main)