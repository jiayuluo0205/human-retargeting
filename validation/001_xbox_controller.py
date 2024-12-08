import pygame

pygame.init()

if pygame.joystick.get_count() == 0:
    print("No joystick connected!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick: {joystick.get_name()}")
button_map = {0: 'A', 1: 'B', 2: 'X', 3: 'Y', 4: 'LB', 5: 'RB', 6: 'Window', 7: 'Menu', 11: 'Screen'}

while True:
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {button_map[event.button]} pressed")
        elif event.type == pygame.JOYBUTTONUP:
            print(f"Button {button_map[event.button]} released")
        elif event.type == pygame.JOYAXISMOTION:
            left_x = joystick.get_axis(0)
            left_y = -joystick.get_axis(1)
            right_x = joystick.get_axis(2)
            right_y = -joystick.get_axis(3)
            lt = joystick.get_axis(4)
            rt = joystick.get_axis(5)
            print(f"Left X: {left_x:.2f}, Left Y: {left_y:.2f}, Right X: {right_x:.2f}, Right Y: {right_y:.2f}, LT: {lt:.2f}, RT: {rt:.2f}")
        elif event.type == pygame.JOYHATMOTION:
            print(f"Hat: {joystick.get_hat(0)}")

pygame.quit()
