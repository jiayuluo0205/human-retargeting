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
        pygame.event.get()
        left = joystick.get_axis(1)
        right = joystick.get_axis(2)
        print(f'Left: {left:.2f}, Right: {right:.2f}')

except KeyboardInterrupt:
    print("Quiting...")
finally:
    pygame.quit()