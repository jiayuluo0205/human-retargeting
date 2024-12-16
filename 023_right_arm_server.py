import zmq
import sys
sys.path.append('utils')
import rebocap_ws_sdk
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import pybullet_data
import viser

def get_link(sdk):
    tran, pose24, static_index, tp = sdk.get_last_msg()

    pose24_np = np.array(pose24)#.tobytes()

    X_WorldRebocap = np.eye(4)
    X_WorldRebocap[:3, :3] = R.from_euler("x",[90.0] , degrees=True).as_matrix()
    
    X_RebocapLink9 = np.eye(4)
    X_RebocapLink9[:3, :3] = R.from_quat(pose24_np[9]).as_matrix()
    X_RebocapLink9[1, 3] = 1.25  # chest2ground height 

    X_RebocapLink16 = np.eye(4)
    X_Link9Link16 = np.eye(4)
    X_Link9Link16[:3, :3] = R.from_quat(pose24_np[16]).as_matrix()
    X_Link9Link16[0, 3] = 0.17  # shoulder width 
    X_Link9Link16[1, 3] = 0.242  # shoulder height 

    X_RebocapLink17 = np.eye(4)
    X_Link9Link17 = np.eye(4)
    X_Link9Link17[:3, :3] = R.from_quat(pose24_np[17]).as_matrix()
    X_Link9Link17[0, 3] = -0.17  # shoulder width 
    X_Link9Link17[1, 3] = 0.242  # shoulder height 

    X_RebocapLink18 = np.eye(4)
    X_Link16Link18 = np.eye(4)
    X_Link16Link18[:3, :3] = R.from_quat(pose24_np[18]).as_matrix()
    X_Link16Link18[0, 3] = 0.257  # upper arm length

    X_RebocapLink19 = np.eye(4)
    X_Link17Link19 = np.eye(4)
    X_Link17Link19[:3, :3] = R.from_quat(pose24_np[19]).as_matrix()
    X_Link17Link19[0, 3] = -0.257  # upper arm length

    X_RebocapLink20 = np.eye(4)
    X_Link18Link20 = np.eye(4)
    X_Link18Link20[:3, :3] = R.from_quat(pose24_np[20]).as_matrix()
    X_Link18Link20[0, 3] = 0.266  # lower arm length 

    X_RebocapLink21 = np.eye(4)
    X_Link19Link21 = np.eye(4)
    X_Link19Link21[:3, :3] = R.from_quat(pose24_np[21]).as_matrix()
    X_Link19Link21[0, 3] = -0.266  # lower arm length 

    X_RebocapLink22 = np.eye(4)
    X_Link20Link22 = np.eye(4)
    X_Link20Link22[:3, :3] = R.from_quat(pose24_np[22]).as_matrix()
    X_Link20Link22[0, 3] = 0.04  # hand length

    X_RebocapLink23 = np.eye(4)
    X_Link21Link23 = np.eye(4)
    X_Link21Link23[:3, :3] = R.from_quat(pose24_np[23]).as_matrix()
    X_Link21Link23[0, 3] = -0.04  # hand length 

    X_RebocapLink16 = X_RebocapLink9 @ X_Link9Link16
    X_RebocapLink17 = X_RebocapLink9 @ X_Link9Link17
    X_RebocapLink18 = X_RebocapLink16 @ X_Link16Link18
    X_RebocapLink19 = X_RebocapLink17 @ X_Link17Link19
    X_RebocapLink20 = X_RebocapLink18 @ X_Link18Link20
    X_RebocapLink21 = X_RebocapLink19 @ X_Link19Link21
    X_RebocapLink22 = X_RebocapLink20 @ X_Link20Link22
    X_RebocapLink23 = X_RebocapLink21 @ X_Link21Link23

    X_WorldLink9 = X_WorldRebocap @ X_RebocapLink9
    X_WorldLink16 = X_WorldRebocap @ X_RebocapLink16
    X_WorldLink17 = X_WorldRebocap @ X_RebocapLink17
    X_WorldLink18 = X_WorldRebocap @ X_RebocapLink18
    X_WorldLink19 = X_WorldRebocap @ X_RebocapLink19
    X_WorldLink20 = X_WorldRebocap @ X_RebocapLink20
    X_WorldLink21 = X_WorldRebocap @ X_RebocapLink21
    X_WorldLink22 = X_WorldRebocap @ X_RebocapLink22
    X_WorldLink23 = X_WorldRebocap @ X_RebocapLink23
    
    return X_WorldLink22, X_WorldLink23

counter = 0

def main():
    def print_debug_msg(self: rebocap_ws_sdk.RebocapWsSdk, trans, pose24, static_inlocaldex, ts):
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

    # 姿态数据回调
    def pose_msg_callback(self: rebocap_ws_sdk.RebocapWsSdk, tran: list, pose24: list, static_index: int, ts: float):
        print_debug_msg(self, tran, pose24, static_index, ts)
        pass


    # 异常断开，这里处理重连或报错
    def exception_close_callback(self: rebocap_ws_sdk.RebocapWsSdk):
        print("exception_close_callback")
    # server initialize
    context = zmq.Context()
    # # talk to client linux machine
    socket = context.socket(zmq.REP) # we r server
    socket.bind("tcp://*:5555")
    
    # rebocap initialize
    # 初始化sdk  这里可以选择控件坐标系， 控件坐标系目前已经测试过的有： 1. UE  2. Unity  3. Blender
    # 选择输出角度是否是 global 角度，默认是 local 角度【简单解释，global 角度不受父节点影响 local角度受父节点影响， local角度逐级相乘就是 global 角度
    sdk = rebocap_ws_sdk.RebocapWsSdk(coordinate_type=rebocap_ws_sdk.CoordinateType.DefaultCoordinate, use_global_rotation=False)
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

    while True:
        message_recv = socket.recv()
        print(message_recv)
        
        X_WorldLink22, X_WorldLink23 = get_link(sdk)
        X_WorldLink22and23 = np.stack([X_WorldLink22, X_WorldLink23], axis=0)
        message_send = X_WorldLink22and23.tobytes()

        '''
        Pick & Place
          Expert  Rookie  Phantom
        1 8.69s   13.17s fail 17.41s
        2 7.09s   33.72s      14.82s
        3 7.04s   11.46s fail 15.85s
        4 8.77s   22.20s      10.56s
        5 7.96s   22.04s      9.11s
        6 7.31s   19.43s      
        7 8.88s   21.84s
        8 7.57s   22.13s
        9 7.16s   8.91s fail
        10 6.63s  9.43s fail
        0.6 - 1.0
        23.56s - 13.55s

        Hang
          Expert  Rookie            Phantom
        1 16.33s  39.50s fail       28.72s
        2 27.98s  28.98s            29.08s
        3 19.69s  1.02.73s          28.08s
        4 16.78s  49.36s fail       38.00s
        5 fail 14.30s  38.61s fail  30.28s
        6 10.37s  16.42s fail
        7 14.05s  24.48s
        8 14.66s  23.57s
        9 14.89s  17.49s
        10 16.25s  18.53s
        0.6 - 1.0
        29.30s - 30.832s

        Pour
          Expert  Rookie        Phantom
        1 29.39s  56.17s        49.96s
        2 27.90s  36.04s        41.34s fail
        3 26.65s  38.28s        35.15s
        4 23.76s  40.58s        30.27s
        5 23.65s  23.73s fail   29.15s
        6 23.14s  53.11s
        7 22.33s  41.25s
        8 20.25s  45.95s
        9 20.30s  40.58s
        10 21.23s  36.87s
        0.9 - 0.8
        43.20s - 36.1325s

        Box Rotate
          Expert  Rookie        Phantom
        1 10.27s  1.01.12s      21.27s
        2 15.57s  12.57s fail   13.40s
        3 16.46s  18.16s        11.66s fail
        4 15.16s  24.02s        20.48s
        5 12.33s  22.23s fail   21.34s
        6 15.35s  36.75s fail
        7 12.58s  32.79s fail
        8 16.84s  26.71s
        9 13.80s  28.70s
        10 13.42s  24.12s
        0.6 - 0.8
        30.47s - 19.1225s

        2 Cup Stacking
          Expert  Rookie        Phantom
        1 16.82s  14.46s fail   20.48s
        2 13.26s  33.23s fail   17.61s
        3 15.86s  29.12s        17.20s
        4 18.03s  18.99s fail   19.98s
        5 16.99s  34.78s        19.30s
        6 14.79s  32.65s
        7 17.28s  16.76s fail
        8 14.22s  21.55s
        9 10.84s  25.90s fail
        10 10.62s  39.59s
        0.5 - 1.0
        31.538s - 18.914s

        '''
        socket.send(message_send)

if __name__ == "__main__":
    main()