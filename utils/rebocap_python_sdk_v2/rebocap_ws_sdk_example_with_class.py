import time

import rebocap_ws_sdk

counter = 0


class SDKManager:

    def __init__(self, port):
        self.coordinate_type = rebocap_ws_sdk.CoordinateType.UnityCoordinate
        self.sdk = rebocap_ws_sdk.RebocapWsSdk(coordinate_type=self.coordinate_type, use_global_rotation=False)
        self.connected = False
        self.recv_cnt = 0
        self.port = port
        self.sdk.set_pose_msg_callback(self.on_msg_recv)
        self.sdk.set_exception_close_callback(self.on_exception_close)

    def on_exception_close(self):
        self.connected = False
        print(f'on_exception_close')

    def on_msg_recv(self, sdk_instance: rebocap_ws_sdk.RebocapWsSdk, tran: list, pose24: list, static_index: int, ts: float):
        is_left_on_floor = 0 <= static_index <= 5
        is_right_on_floor = 6 <= static_index <= 11
        no_foot_on_ground = static_index < 0
        if self.recv_cnt % 60 == 0:
            print(f'timestamp:{ts}\n'
                  f'current coordinate_type: {self.coordinate_type.name}'
                  f'root position:{tran} left_on_floor:{is_left_on_floor}  right_on_floor:{is_right_on_floor}')
            for i in range(24):
                print(f'bone:{rebocap_ws_sdk.REBOCAP_JOINT_NAMES[i]} quaternion w,x,y,z is:{pose24[i]}')
            print('\n\n\n\n', flush=True)
        self.recv_cnt += 1

    def open_connect(self):
        if self.connected:
            return -10
        open_ret = self.sdk.open(self.port)
        # 检查连接状态
        if open_ret == 0:
            print("连接成功")
            return 0
        print("连接失败", open_ret)
        if open_ret == 1:
            print("连接状态错误")
        elif open_ret == 2:
            print("连接失败")
        elif open_ret == 3:
            print("认证失败")
        else:
            print("未知错误", open_ret)
        return open_ret

    def stop_connect(self):
        if self.connected:
            self.sdk.close()


def main():
    sdk_manager = SDKManager(7690)
    if sdk_manager.open_connect() == 0:
        for i in range(10):
            time.sleep(1.0)


if __name__ == "__main__":
    main()
