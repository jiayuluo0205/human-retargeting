import cv2 
import numpy as np
from pathlib import Path
from xarm6_interface import SAM_TYPE, SAM_PATH
from xarm6_interface.utils.sam_prompt_drawer import SAMPromptDrawer

import os
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

if __name__ == "__main__":
    serial_number= "233622079809" # "241122074374", "233622079809"
    exp_name = "1214_excalib_capture01"
    save_data_rel_dir_path = (Path(os.path.join(ROOT_DIR, f"3rdparty/xarm6/data/camera/{serial_number}")) / exp_name).resolve()
    sample_id_paths = list(save_data_rel_dir_path.glob("*"))
    sample_id_paths = sorted(sample_id_paths)
    sample_id_paths = [p for p in sample_id_paths if p.is_dir()]

    sample_img_paths = [p / "rgb_image.jpg" for p in sample_id_paths]

    # setup the prompt drawer
    prompt_drawer = SAMPromptDrawer(window_name="Prompt Drawer", screen_scale=2.0, sam_checkpoint=SAM_PATH, device="cuda", model_type=SAM_TYPE)
    
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
                mask_path = img_path.parent / "mask.npy"
                np.save(str(mask_path), mask)
                print(f"Mask saved to: {mask_path}")
            else:
                print(f"No mask generated for: {img_path}")
        else:
            print(f"Image not found: {img_path}")

