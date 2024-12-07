import numpy as np

X_CameraTag25 = np.load("data/transform/camera_tag25.npy").reshape((4, 4))
X_CameraTag36 = np.load("data/transform/camera_tag36.npy").reshape((4, 4))
X_ArmTag36 = np.load("data/transform/rightarm_tag36.npy").reshape((4, 4))

# print(X_CameraTag25)

# X_Tag36Camera = np.linalg.inv(X_CameraTag36)

# X_ArmTag25 = X_ArmTag36 @ X_Tag36Camera @ X_CameraTag25
# np.save("data/transform/rightarm_tag25.npy", X_ArmTag25)
# print(X_ArmTag25)

X_Tag25Camera = np.linalg.inv(X_CameraTag25)
X_Tag25Tag36 = X_Tag25Camera @ X_CameraTag36

print(X_Tag25Camera)

