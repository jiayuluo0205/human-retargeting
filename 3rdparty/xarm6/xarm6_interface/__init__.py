from pathlib import Path
import torch
import os

PACKAGE_PATH = Path(__file__).parent
ASSETS_PATH = PACKAGE_PATH.parent / "assets"
XARM6_ASSETS_DIR_PATH = ASSETS_PATH / "xarm6"


XARM6_WO_EE_URDF_PATH = XARM6_ASSETS_DIR_PATH / "xarm6_wo_ee.urdf"
XARM6_WO_EE_SRDF_PATH = XARM6_ASSETS_DIR_PATH / "xarm6_wo_ee.srdf"

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

os.path.join(ROOT_DIR, "third_party/xarm6/data/camera/mounted_white")
SAM_PATH = Path(os.path.join(ROOT_DIR, "third_party/segment-anything/ckpt/sam_vit_h_4b8939.pth"))
# SAM_TYPE = "vit_l"
SAM_TYPE = "vit_h"

''' sam 2'''
SAM2_CHECKPOINT = "3rdparty/Grounded-SAM-2/checkpoints/sam2_hiera_small.pt"
SAM2_MODEL_CONFIG = "3rdparty/segment-anything-2/sam2/sam2_hiera_s.yaml"
GROUNDING_DINO_CONFIG = "3rdparty/Grounded-SAM-2/grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py"
GROUNDING_DINO_CHECKPOINT = "3rdparty/Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth"
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
DUMP_JSON_RESULTS = False

XARM6_IP = "192.168.1.208"
XARM6LEFT_IP = "192.168.1.232"