import sys, os, cv2
import numpy as np
from glob import glob
import matplotlib.pyplot as plt
from loguru import logger as lgr
from scipy.spatial.transform import Rotation as R
from xarm6_interface.utils.sam_prompt_drawer_old import SAMPromptDrawer
from xarm6_interface import SAM_TYPE, SAM_PATH
from pathlib import Path


if __name__ == "__main__":
    save_dir = Path('xarm6_interface/utils/box_data')
    saved_images_paths = list(save_dir.glob('color_*.png'))
    prompt_drawer = SAMPromptDrawer(window_name="Prompt Drawer", screen_scale=2.0, sam_checkpoint=SAM_PATH, device="cuda", model_type=SAM_TYPE)

    # now filter the images with the validated indices 
    valid_frame_idx_path = os.path.join(save_dir, 'valid_frame_idx.npy')
    valid_frame_idx = np.load(valid_frame_idx_path)
    valid_frame_object_X_path = os.path.join(save_dir, 'valid_frame_object_X.npy')
    valid_frame_object_X = np.load(valid_frame_object_X_path)

    sample_img_paths = [p for i, p in enumerate(saved_images_paths) if i in valid_frame_idx]

    # Iterate through each image and obtain the mask
    for img_path in sample_img_paths:
        print(f"Processing image: {img_path}")
        if img_path.exists():
            # Load the RGB image
            rgb_image = cv2.imread(str(img_path))

            # Convert the image from BGR (OpenCV default) to RGB for SAMPromptDrawer
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

            # Run the prompt drawer to obtain the mask
            prompt_drawer.reset()
            mask = prompt_drawer.run(rgb_image)

            # Save the mask as a numpy array
            if mask is not None:
                img_name = img_path.name
                mask_name = img_name.replace('color_', 'mask_').replace('.png', '.npy')
                mask_path = img_path.parent / mask_name
                np.save(str(mask_path), mask)
                print(f"Mask saved to: {mask_path}")
            else:
                print(f"No mask generated for: {img_path}")
        else:
            print(f"Image not found: {img_path}")


