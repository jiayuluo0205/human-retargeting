import pyrealsense2 as rs
import numpy as np
import cv2
from xarm6_interface.utils import MultiRealsense

cameras = MultiRealsense(['233622079809', '147122075879'])



while True:
    obs = cameras.get_obs()
    agentview_img = obs[0]
    eye_in_hand_img = obs[1]
    # agentview_img_resized = cv2.resize(agentview_img)
    # eye_in_hand_img_resized = cv2.resize(eye_in_hand_img)

    combined_img = np.vstack((agentview_img, eye_in_hand_img))
    camera_intr = cameras.get_intrinsics()
    print(camera_intr[0].fx, camera_intr[0].ppx)
    exit()

    cv2.imshow('agentview_img', agentview_img)
    cv2.imshow('eye_in_hand_img', eye_in_hand_img)

    cv2.pollKey()
