import zmq
import sys
sys.path.append('utils')
import rebocap_ws_sdk
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import pybullet_data
import viser
from pathlib import Path

from smpl_visualizer import *

def get_link(pose24):
    # tran, pose24, static_index, tp = sdk.get_last_msg()

    pose24_np = np.array(pose24)#.tobytes()

    X_WorldLinks_combo = []

    # X_RebocapWorld = np.eye(4)
    # # X_RebocapWorld[:3, :3] = R.from_euler("x", -90, degrees=True).as_matrix()
    # X_RebocapWorld[:3, :3] = R.from_euler("ZX", [-90, -90], degrees=True).as_matrix()
    X_WorldRebocap = np.eye(4)
    X_WorldRebocap[:3, :3] = R.from_euler("yx",[90.0, 90.0] , degrees=True).as_matrix()
    
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

    # X_WorldLink9 = np.linalg.inv(X_RebocapWorld) @ X_RebocapLink9
    # X_WorldLink16 = np.linalg.inv(X_RebocapWorld) @ X_RebocapLink16
    # X_WorldLink17 = np.linalg.inv(X_RebocapWorld) @ X_RebocapLink17
    # X_WorldLink18 = np.linalg.inv(X_RebocapWorld) @ X_RebocapLink18
    # X_WorldLink19 = np.linalg.inv(X_RebocapWorld) @ X_RebocapLink19
    # X_WorldLink20 = np.linalg.inv(X_RebocapWorld) @ X_RebocapLink20
    # X_WorldLink21 = np.linalg.inv(X_RebocapWorld) @ X_RebocapLink21

    X_WorldLink9 = X_WorldRebocap @ X_RebocapLink9
    X_WorldLink16 = X_WorldRebocap @ X_RebocapLink16
    X_WorldLink17 = X_WorldRebocap @ X_RebocapLink17
    X_WorldLink18 = X_WorldRebocap @ X_RebocapLink18
    X_WorldLink19 = X_WorldRebocap @ X_RebocapLink19
    X_WorldLink20 = X_WorldRebocap @ X_RebocapLink20
    X_WorldLink21 = X_WorldRebocap @ X_RebocapLink21
    X_WorldLink22 = X_WorldRebocap @ X_RebocapLink22
    X_WorldLink23 = X_WorldRebocap @ X_RebocapLink23

    X_WorldLinks_combo.append(X_WorldLink9)
    X_WorldLinks_combo.append(X_WorldLink16)
    X_WorldLinks_combo.append(X_WorldLink17)
    X_WorldLinks_combo.append(X_WorldLink18)
    X_WorldLinks_combo.append(X_WorldLink19)
    X_WorldLinks_combo.append(X_WorldLink20)
    X_WorldLinks_combo.append(X_WorldLink21)
    X_WorldLinks_combo.append(X_WorldLink22)
    X_WorldLinks_combo.append(X_WorldLink23)

    return X_WorldLinks_combo
    # return X_WorldLink9, X_WorldLink16, X_WorldLink17, X_WorldLink18, X_WorldLink19, X_WorldLink20, X_WorldLink21, X_WorldLink22, X_WorldLink23

def add_frames(sv, X_WorldLinks_combo):
    X_WorldLink9, X_WorldLink16, X_WorldLink17, X_WorldLink18, X_WorldLink19, X_WorldLink20, X_WorldLink21, X_WorldLink22, X_WorldLink23 = X_WorldLinks_combo
    sv.scene.add_frame(
        "Link9", wxyz=R.from_matrix(X_WorldLink9[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink9[:3, 3]
    )
    sv.scene.add_frame(
        "Link16", wxyz=R.from_matrix(X_WorldLink16[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink16[:3, 3]
    )
    sv.scene.add_frame(
        "Link17", wxyz=R.from_matrix(X_WorldLink17[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink17[:3, 3]
    )
    sv.scene.add_frame(
        "Link18", wxyz=R.from_matrix(X_WorldLink18[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink18[:3, 3]
    )
    sv.scene.add_frame(
        "Link19", wxyz=R.from_matrix(X_WorldLink19[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink19[:3, 3]
    )
    sv.scene.add_frame(
        "Link20", wxyz=R.from_matrix(X_WorldLink20[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink20[:3, 3]
    )
    sv.scene.add_frame(
        "Link21", wxyz=R.from_matrix(X_WorldLink21[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink21[:3, 3]
    )
    sv.scene.add_frame(
        "Link22", wxyz=R.from_matrix(X_WorldLink22[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink22[:3, 3]
    )
    sv.scene.add_frame(
        "Link23", wxyz=R.from_matrix(X_WorldLink23[:3, :3]).as_quat()[[3, 0, 1, 2]], position=X_WorldLink23[:3, 3]
    )

def compute_joints_angles(pose24):
    pose24_np = np.array(pose24)
    Euler_Joint_combo = []

    pose24_np = detect_zero_norm(pose24_np)
    # Euler_Joint0 = R.from_quat(pose24_np[0]).as_euler('xyz')
    # Euler_Joint1 = R.from_quat(pose24_np[1]).as_euler('xyz')
    # Euler_Joint2 = R.from_quat(pose24_np[2]).as_euler('xyz')
    # Euler_Joint3 = R.from_quat(pose24_np[3]).as_euler('xyz')
    # Euler_Joint4 = R.from_quat(pose24_np[4]).as_euler('xyz')
    # Euler_Joint5 = R.from_quat(pose24_np[5]).as_euler('xyz')
    # Euler_Joint6 = R.from_quat(pose24_np[6]).as_euler('xyz')
    # Euler_Joint7 = R.from_quat(pose24_np[7]).as_euler('xyz')
    # Euler_Joint8 = R.from_quat(pose24_np[8]).as_euler('xyz')
    # Euler_Joint9 = R.from_quat(pose24_np[9]).as_euler('xyz')
    # Euler_Joint10 = R.from_quat(pose24_np[10]).as_euler('xyz')
    # Euler_Joint11 = R.from_quat(pose24_np[11]).as_euler('xyz')
    # Euler_Joint12 = R.from_quat(pose24_np[12]).as_euler('xyz')
    # Euler_Joint13 = R.from_quat(pose24_np[13]).as_euler('xyz')
    # Euler_Joint14 = R.from_quat(pose24_np[14]).as_euler('xyz')
    # Euler_Joint15 = R.from_quat(pose24_np[15]).as_euler('xyz')
    # Euler_Joint16 = R.from_quat(pose24_np[16]).as_euler('xyz')
    # Euler_Joint17 = R.from_quat(pose24_np[17]).as_euler('xyz')
    # Euler_Joint18 = R.from_quat(pose24_np[18]).as_euler('xyz')
    # Euler_Joint19 = R.from_quat(pose24_np[19]).as_euler('xyz')
    # Euler_Joint20 = R.from_quat(pose24_np[20]).as_euler('xyz')
    # Euler_Joint21 = R.from_quat(pose24_np[21]).as_euler('xyz')
    # Euler_Joint22 = R.from_quat(pose24_np[22]).as_euler('xyz')
    # Euler_Joint23 = R.from_quat(pose24_np[23]).as_euler('xyz')

    Euler_Joint0 = R.from_quat(pose24_np[0]).as_rotvec()
    Euler_Joint1 = R.from_quat(pose24_np[1]).as_rotvec()
    Euler_Joint2 = R.from_quat(pose24_np[2]).as_rotvec()
    Euler_Joint3 = R.from_quat(pose24_np[3]).as_rotvec()
    Euler_Joint4 = R.from_quat(pose24_np[4]).as_rotvec()
    Euler_Joint5 = R.from_quat(pose24_np[5]).as_rotvec()
    Euler_Joint6 = R.from_quat(pose24_np[6]).as_rotvec()
    Euler_Joint7 = R.from_quat(pose24_np[7]).as_rotvec()
    Euler_Joint8 = R.from_quat(pose24_np[8]).as_rotvec()
    Euler_Joint9 = R.from_quat(pose24_np[9]).as_rotvec()
    Euler_Joint10 = R.from_quat(pose24_np[10]).as_rotvec()
    Euler_Joint11 = R.from_quat(pose24_np[11]).as_rotvec()
    Euler_Joint12 = R.from_quat(pose24_np[12]).as_rotvec()
    Euler_Joint13 = R.from_quat(pose24_np[13]).as_rotvec()
    Euler_Joint14 = R.from_quat(pose24_np[14]).as_rotvec()
    Euler_Joint15 = R.from_quat(pose24_np[15]).as_rotvec()
    Euler_Joint16 = R.from_quat(pose24_np[16]).as_rotvec()
    Euler_Joint17 = R.from_quat(pose24_np[17]).as_rotvec()
    Euler_Joint18 = R.from_quat(pose24_np[18]).as_rotvec()
    Euler_Joint19 = R.from_quat(pose24_np[19]).as_rotvec()
    Euler_Joint20 = R.from_quat(pose24_np[20]).as_rotvec()
    Euler_Joint21 = R.from_quat(pose24_np[21]).as_rotvec()
    Euler_Joint22 = R.from_quat(pose24_np[22]).as_rotvec()
    Euler_Joint23 = R.from_quat(pose24_np[23]).as_rotvec()

    Euler_Joint_combo.append(Euler_Joint0)
    Euler_Joint_combo.append(Euler_Joint1)
    Euler_Joint_combo.append(Euler_Joint2)
    Euler_Joint_combo.append(Euler_Joint3)
    Euler_Joint_combo.append(Euler_Joint4)
    Euler_Joint_combo.append(Euler_Joint5)
    Euler_Joint_combo.append(Euler_Joint6)
    Euler_Joint_combo.append(Euler_Joint7)
    Euler_Joint_combo.append(Euler_Joint8)
    Euler_Joint_combo.append(Euler_Joint9)
    Euler_Joint_combo.append(Euler_Joint10)
    Euler_Joint_combo.append(Euler_Joint11)
    Euler_Joint_combo.append(Euler_Joint12)
    Euler_Joint_combo.append(Euler_Joint13)
    Euler_Joint_combo.append(Euler_Joint14)
    Euler_Joint_combo.append(Euler_Joint15)
    Euler_Joint_combo.append(Euler_Joint16)
    Euler_Joint_combo.append(Euler_Joint17)
    Euler_Joint_combo.append(Euler_Joint18)
    Euler_Joint_combo.append(Euler_Joint19)
    Euler_Joint_combo.append(Euler_Joint20)
    Euler_Joint_combo.append(Euler_Joint21)
    Euler_Joint_combo.append(Euler_Joint22)
    Euler_Joint_combo.append(Euler_Joint23)

    return Euler_Joint_combo

def detect_zero_norm(pose24_np):
    for idx, quat in enumerate(pose24_np):
        norm = np.linalg.norm(quat)
        if norm == 0 or np.any(np.isnan(quat)) or np.any(np.isinf(quat)):
            quat = np.array([0, 0, 0, 1])
            pose24_np[idx] = quat
    return pose24_np


counter = 0

def main():
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

    # 姿态数据回调
    def pose_msg_callback(self: rebocap_ws_sdk.RebocapWsSdk, tran: list, pose24: list, static_index: int, ts: float):
        print(f'X_RecocapLink20: \n{X_RecocapLink20}\n')
        print_debug_msg(self, tran, pose24, static_index, ts)
        # message = socket.recv()
        # 将 X_RecocapLink20 通过 zmq 发送
        # socket.send(X_RecocapLink20.tobytes())
        pass


    # 异常断开，这里处理重连或报错
    def exception_close_callback(self: rebocap_ws_sdk.RebocapWsSdk):
        print("exception_close_callback")
    # server initialize
    context = zmq.Context()
    # # talk to client linux machine
    socket = context.socket(zmq.REP)
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

    sv = viser.ViserServer(port=8085)
    
    '''008_smpl_visualizer.py'''
    model_path = Path("D:\Projects\human-retargeting/assets\Model\smplh\male\model.npz")
    server = viser.ViserServer()
    server.scene.set_up_direction("+y")
    server.gui.configure_theme(control_layout="collapsible")

    # Main loop. We'll read pose/shape from the GUI elements, compute the mesh,
    # and then send the updated mesh in a loop.
    model = SmplHelper(model_path)
    gui_elements = make_gui_elements(
        server,
        num_betas=model.num_betas,
        num_joints=model.num_joints,
        parent_idx=model.parent_idx,
    )
    body_handle = server.scene.add_mesh_simple(
        "/human",
        model.v_template,
        model.faces,
        wireframe=gui_elements.gui_wireframe.value,
        color=gui_elements.gui_rgb.value,
    )
    '''008_smpl_visualizer.py'''

    while True:
        global X_WorldLink20

        # message_recv = socket.recv()
        tran, pose24, static_index, tp = sdk.get_last_msg()

        # X_WorldLink9, X_WorldLink16, X_WorldLink17, X_WorldLink18, X_WorldLink19, X_WorldLink20, X_WorldLink21, X_WorldLink22, X_WorldLink23 = get_link(pose24)
        
        # arm control
        # X_WorldLinks_combo = get_link(pose24)
        # add_frames(sv, X_WorldLinks_combo)
        # X_WorldLinks_combo = np.stack(X_WorldLinks_combo, axis=0)

        Euler_Joint_combo = compute_joints_angles(pose24)
        Euler_Joint_combo = np.stack(Euler_Joint_combo, axis=0)
        '''008_smpl_visualizer.py'''
        if not gui_elements.changed:
            continue

        gui_elements.changed = False

        # update joints
        for idx, Euler_Joint_i in enumerate(Euler_Joint_combo):
            gui_elements.gui_joints[idx].value = tuple(Euler_Joint_i)

        # If anything has changed, re-compute SMPL outputs.
        smpl_outputs = model.get_outputs(
            betas=np.array([x.value for x in gui_elements.gui_betas]),
            joint_rotmats=tf.SO3.exp(
                # (num_joints, 3)
                np.array([x.value for x in gui_elements.gui_joints])
            ).as_matrix(),
        )

        # Update the mesh properties based on the SMPL model output + GUI
        # elements.
        body_handle.vertices = smpl_outputs.vertices
        body_handle.wireframe = gui_elements.gui_wireframe.value
        body_handle.color = gui_elements.gui_rgb.value

        # Match transform control gizmos to joint positions.
        for i, control in enumerate(gui_elements.transform_controls):
            control.position = smpl_outputs.T_parent_joint[i, :3, 3]
        '''008_smpl_visualizer.py'''

        message_send = Euler_Joint_combo.astype(np.float64).tobytes()
        '''
        X_WorldLink22and23 = np.stack([X_WorldLink22, X_WorldLink23], axis=0)
        message_send = X_WorldLink22and23.tobytes()
        '''
        # socket.send(message_send)

if __name__ == "__main__":
    main()