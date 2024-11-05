import pygame
import time

# 初始化 pygame 和控制器
pygame.init()
pygame.joystick.init()

# 检查是否连接了控制器
if pygame.joystick.get_count() == 0:
    print("没有检测到控制器")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"控制器已初始化: {joystick.get_name()}")

try:
    while True:
        # 处理 pygame 事件
        pygame.event.pump()

        # 获取控制器的轴数量
        num_axes = joystick.get_numaxes()

        print(f'Axis 0: {joystick.get_axis(0):.2f},\
                Axis 1: {joystick.get_axis(1):.2f},\
                Axis 2: {joystick.get_axis(2):.2f}')

        # 每隔0.1秒刷新一次，避免输出太快
        time.sleep(0.1)

except KeyboardInterrupt:
    print("退出程序...")
finally:
    pygame.quit()