'''
Adapted from https://github.com/ootts/EasyHeC/blob/main/docs/custom.md
Press "z" to undo, "p" to switch to the point prompt mode, and "b" to switch to the bounding box prompt mode. Although the point prompt is supported, we recommend using the bounding box prompt since it's more stable than the point prompt.
'''
import cv2
import torch
import subprocess
import numpy as np
from enum import Enum
# from xarm6_interface.utils.misc import to_array, findContours
from segment_anything import sam_model_registry, SamPredictor
from sam2.build_sam import build_sam2_camera_predictor
from PIL import Image
from torchvision.transforms import transforms as T
from xarm6_interface import GROUNDING_DINO_CONFIG, GROUNDING_DINO_CHECKPOINT, BOX_THRESHOLD, TEXT_THRESHOLD
# from groundingdino.util.inference import load_model, load_image, predict
from torchvision.ops import box_convert


class DrawingMode(Enum):
    Box = 0
    Point = 1


def vis_mask(img,
             mask,
             color=[255, 255, 255],
             alpha=0.4,
             show_border=True,
             border_alpha=0.5,
             border_thick=1,
             border_color=None):
    """Visualizes a single binary mask."""
    if isinstance(mask, torch.Tensor):
        mask = to_array(mask > 0).astype(np.uint8)
    img = img.astype(np.float32)
    idx = np.nonzero(mask)

    img[idx[0], idx[1], :] *= 1.0 - alpha
    img[idx[0], idx[1], :] += [alpha * x for x in color]

    if show_border:
        contours, _ = findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # contours = [c for c in contours if c.shape[0] > 10]
        if border_color is None:
            border_color = color
        if not isinstance(border_color, list):
            border_color = border_color.tolist()
        if border_alpha < 1:
            with_border = img.copy()
            cv2.drawContours(with_border, contours, -1, border_color,
                             border_thick, cv2.LINE_AA)
            img = (1 - border_alpha) * img + border_alpha * with_border
        else:
            cv2.drawContours(img, contours, -1, border_color, border_thick,
                             cv2.LINE_AA)
    return img.astype(np.uint8)


def grounding_dino_get_bbox(rgb_np, text):
    
    grounding_model = load_model(
        model_config_path=GROUNDING_DINO_CONFIG, 
        model_checkpoint_path=GROUNDING_DINO_CHECKPOINT,
    )
    
    transform = T.Compose(
        [
            T.Resize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ]
    )
    rgb_image = Image.fromarray(rgb_np)
    
    image = transform(rgb_image)
    boxes, confidences, labels = predict(
        model=grounding_model,
        image=image,
        caption=text,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD,
    )
    # process the box prompt for SAM 2
    h, w, _ = rgb_np.shape
    boxes = boxes * torch.Tensor([w, h, w, h])
    input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

    return input_boxes

class SAMPromptDrawer(object):
    def __init__(self, window_name="Prompt Drawer", screen_scale=1.0, sam_checkpoint="",
                 device="cuda", model_type="default"):
        self.window_name = window_name
        self.reset()
        self.screen_scale = screen_scale * 1.2
        self.screen_scale = screen_scale

        # Initialize the SAM predictor
        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        sam.to(device=device)

        self.predictor = SamPredictor(sam)

    def reset(self):
        self.done = False
        self.drawing = False
        self.current = (0, 0)
        self.box = np.zeros([4], dtype=np.float32)
        self.points = np.empty((0, 2))
        self.labels = np.empty([0], dtype=int)
        self.mask = None
        self.mode = DrawingMode.Box
        self.boxes = np.zeros([0, 4], dtype=np.float32)
        self.box_labels = np.empty([0], dtype=int)

    def on_mouse(self, event, x, y, flags, user_param):
        # Mouse callback for every mouse event
        if self.done:
            return
        if self.mode == DrawingMode.Box:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drawing = True
                if flags & cv2.EVENT_FLAG_CTRLKEY:
                    self.box_labels = np.hstack([self.box_labels, 0])
                else:
                    self.box_labels = np.hstack([self.box_labels, 1])
                self.boxes = np.vstack([self.boxes, [x, y, x, y]])
            elif event == cv2.EVENT_LBUTTONUP:
                self.drawing = False
                self.boxes[-1, 2] = x
                self.boxes[-1, 3] = y
                self.detect()  # Recalculate mask
            elif event == cv2.EVENT_MOUSEMOVE:
                if self.drawing:
                    self.boxes[-1, 2] = x
                    self.boxes[-1, 3] = y
        elif self.mode == DrawingMode.Point:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.points = np.vstack([self.points, [x, y]])
                if flags & cv2.EVENT_FLAG_CTRLKEY:
                    label = 0
                else:
                    label = 1
                self.labels = np.hstack([self.labels, label])
                self.detect()  # Recalculate mask
            elif event == cv2.EVENT_RBUTTONDOWN:
                self.points = np.vstack([self.points, [x, y]])
                self.labels = np.hstack([self.labels, 1])
                self.detect()  # Recalculate mask

    def detect(self):
        # Prepare inputs for the predictor
        if len(self.points) != 0:
            input_point = self.points / self.ratio
            input_label = self.labels.astype(int)
        else:
            input_point = None
            input_label = None

        # Initialize the final mask
        final_mask = None

        # Process each box
        if len(self.boxes) == 0:
            # If no boxes are present, use only points for detection
            masks, scores, logits = self.predictor.predict(
                point_coords=input_point,
                point_labels=input_label,
                box=None,
                multimask_output=True,
            )
            maxidx = np.argmax(scores)
            final_mask = masks[maxidx].copy()
        else:
            # Iterate through all boxes and calculate masks
            for i in range(len(self.boxes)):
                box = self.boxes[i]
                box_label = self.box_labels[i]

                if np.all(box == 0):
                    box = None
                else:
                    box = box / self.ratio

                # Generate masks for each box
                masks, scores, logits = self.predictor.predict(
                    point_coords=input_point,
                    point_labels=input_label,
                    box=box,
                    multimask_output=True,
                )
                maxidx = np.argmax(scores)
                mask = masks[maxidx]

                # Combine masks logically based on the labels
                if final_mask is None:
                    final_mask = mask.copy()
                else:
                    if box_label == 0:
                        final_mask = np.logical_and(final_mask, ~mask)
                    else:
                        final_mask = np.logical_or(final_mask, mask)

        # Update the mask attribute
        if final_mask is not None:
            self.mask = final_mask.copy()
        elif self.mask is not None:
            self.mask = np.zeros_like(self.mask)

    def run(self, rgb):
        self.rgb = rgb
        self.predictor.set_image(rgb)
        image_to_show = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        image_h, image_w = image_to_show.shape[:2]

        if not hasattr(self, "ratio"):
            output = subprocess.check_output(["xrandr"]).decode("utf-8")
            current_mode = [line for line in output.splitlines() if "*" in line][0]
            screen_width, screen_height = [int(x) for x in current_mode.split()[0].split("x")]
            scale = self.screen_scale
            screen_w = int(screen_width / scale)
            screen_h = int(screen_height / scale)

            ratio = min(screen_w / image_w, screen_h / image_h)
            self.ratio = ratio
        target_size = (int(image_w * self.ratio), int(image_h * self.ratio))
        image_to_show = cv2.resize(image_to_show, target_size)

        cv2.namedWindow(self.window_name)
        cv2.imshow(self.window_name, image_to_show)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        while not self.done:
            tmp = image_to_show.copy()
            tmp = cv2.circle(tmp, self.current, radius=2, color=(0, 0, 255), thickness=-1)
            for box, box_label in zip(self.boxes, self.box_labels):
                color = (0, 255, 0) if box_label == 1 else (0, 0, 255)
                cv2.rectangle(tmp, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, 2)
            if self.points.shape[0] > 0:
                for ptidx, pt in enumerate(self.points):
                    color = (0, 255, 0) if self.labels[ptidx] == 1 else (0, 0, 255)
                    tmp = cv2.circle(tmp, (int(pt[0]), int(pt[1])), radius=5, color=color, thickness=-1)
            if self.mask is not None:
                mask_to_show = cv2.resize(self.mask.astype(np.uint8), target_size).astype(bool)
                tmp = vis_mask(tmp, mask_to_show.astype(np.uint8), color=[0, 255, 0], alpha=0.5).astype(np.uint8)
            cv2.imshow(self.window_name, tmp)
            waittime = 50
            key = cv2.waitKey(waittime)
            if key == 27 or key == 13:  # ESC hit
                self.done = True
            elif key == ord('r'):
                print("Reset")
                self.reset()
            elif key == ord('p'):
                print("Switch to point mode")
                self.mode = DrawingMode.Point
            elif key == ord('b'):
                print("Switch to box mode")
                self.mode = DrawingMode.Box
            elif key == ord('z'):
                print("Undo")
                if self.mode == DrawingMode.Point and len(self.points) > 0:
                    self.points = self.points[:-1]
                    self.labels = self.labels[:-1]
                    self.detect()
                elif self.mode == DrawingMode.Box and len(self.boxes) > 0:
                    self.boxes = self.boxes[:-1]
                    self.box_labels = self.box_labels[:-1]
                    self.detect()
        cv2.destroyWindow(self.window_name)
        return self.mask

    def close(self):
        del self.predictor
        torch.cuda.empty_cache()
        

class SAM2PromptDrawer(object):
    def __init__(self, window_name="Prompt Drawer", screen_scale=1.0, sam_checkpoint="",
                 device="cuda", model_type="default"):
        self.window_name = window_name
        self.reset()
        self.screen_scale = screen_scale * 1.2
        self.screen_scale = screen_scale

        sam2_root_dir = "3rdparty/segment-anything-2"
        sam2_checkpoint = f"{sam2_root_dir}/checkpoints/sam2_hiera_small.pt"
        # model_cfg = "sam2/sam2_hiera_s.yaml"

        # sam2_checkpoint = f"{sam2_root_dir}/checkpoints/sam2.1_hiera_small.pt"
        model_cfg = f"sam2_hiera_s.yaml"
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        # model_cfg = f"~/gjx/Cloth-Flod-in-isaac-sim/3rdparty/segment-anything-2/sam2/sam2_hiera_s.yaml"
=======
<<<<<<< HEAD
=======
        # model_cfg = f"~/gjx/Cloth-Flod-in-isaac-sim/3rdparty/segment-anything-2/sam2/sam2_hiera_s.yaml"
>>>>>>> b38cb60ae4ce008ac68c65a43b77bda796c41b3d
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        
        # import os
        # file_path = model_cfg
        # if os.path.exists(file_path):
        #     print(f"File exists: {file_path}")
        #     if os.path.isfile(file_path):
        #         print("It is a regular file.")
        #         if os.access(file_path, os.R_OK):
        #             print("File is readable.")
        #         else:
        #             print("File is not readable.")
        #     else:
        #         print("It is not a regular file (might be a directory).")
        # else:
        #     print(f"File does not exist: {file_path}")

<<<<<<< Updated upstream
<<<<<<< Updated upstream
        # self.predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint, hydra_overrides_extra=["+config_path=~/gjx/Cloth-Flod-in-isaac-sim/3rdparty/segment-anything-2/sam2"])
=======
<<<<<<< HEAD
=======
        # self.predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint, hydra_overrides_extra=["+config_path=~/gjx/Cloth-Flod-in-isaac-sim/3rdparty/segment-anything-2/sam2"])
>>>>>>> b38cb60ae4ce008ac68c65a43b77bda796c41b3d
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        self.predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)
        # self.predictor = build_sam2_camera_predictor(ckpt_path=sam2_checkpoint)
        

    def reset(self):
        self.done = False
        self.drawing = False
        self.current = (0, 0)
        self.box = np.zeros([4], dtype=np.float32)
        self.points = np.empty((0, 2))
        self.labels = np.empty([0], dtype=int)
        self.mask = None
        self.mode = DrawingMode.Box
        self.boxes = np.zeros([0, 4], dtype=np.float32)
        self.box_labels = np.empty([0], dtype=int)

    def on_mouse(self, event, x, y, flags, user_param):
        # Mouse callback for every mouse event
        if self.done:
            return
        if self.mode == DrawingMode.Box:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drawing = True
                if flags & cv2.EVENT_FLAG_CTRLKEY:
                    self.box_labels = np.hstack([self.box_labels, 0])
                else:
                    self.box_labels = np.hstack([self.box_labels, 1])
                self.boxes = np.vstack([self.boxes, [x, y, x, y]])
            elif event == cv2.EVENT_LBUTTONUP:
                self.drawing = False
                self.boxes[-1, 2] = x
                self.boxes[-1, 3] = y
                self.detect()  # Recalculate mask
            elif event == cv2.EVENT_MOUSEMOVE:
                if self.drawing:
                    self.boxes[-1, 2] = x
                    self.boxes[-1, 3] = y
        elif self.mode == DrawingMode.Point:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.points = np.vstack([self.points, [x, y]])
                if flags & cv2.EVENT_FLAG_CTRLKEY:
                    label = 0
                else:
                    label = 1
                self.labels = np.hstack([self.labels, label])
                self.detect()  # Recalculate mask
            elif event == cv2.EVENT_RBUTTONDOWN:
                self.points = np.vstack([self.points, [x, y]])
                self.labels = np.hstack([self.labels, 1])
                self.detect()  # Recalculate mask

    def detect(self):
        # Prepare inputs for the predictor
        if len(self.points) != 0:
            input_point = self.points / self.ratio
            input_label = self.labels.astype(int)
        else:
            input_point = None
            input_label = None

        # Initialize the final mask
        final_mask = None

        # Process each box
        if len(self.boxes) == 0:
            # If no boxes are present, use only points for detection
            masks, scores, logits = self.predictor.predict(
                point_coords=input_point,
                point_labels=input_label,
                box=None,
                multimask_output=True,
            )
            maxidx = np.argmax(scores)
            final_mask = masks[maxidx].copy()
        else:
            # Iterate through all boxes and calculate masks
            for i in range(len(self.boxes)):
                box = self.boxes[i]
                box_label = self.box_labels[i]

                if np.all(box == 0):
                    box = None
                else:
                    box = box / self.ratio

                # Generate masks for each box
                masks, scores, logits = self.predictor.predict(
                    point_coords=input_point,
                    point_labels=input_label,
                    box=box,
                    multimask_output=True,
                )
                maxidx = np.argmax(scores)
                mask = masks[maxidx]

                # Combine masks logically based on the labels
                if final_mask is None:
                    final_mask = mask.copy()
                else:
                    if box_label == 0:
                        final_mask = np.logical_and(final_mask, ~mask)
                    else:
                        final_mask = np.logical_or(final_mask, mask)

        # Update the mask attribute
        if final_mask is not None:
            self.mask = final_mask.copy()
        elif self.mask is not None:
            self.mask = np.zeros_like(self.mask)

    def run(self, rgb):
        self.rgb = rgb
        # self.predictor.set_image(rgb)
        self.predictor.load_first_frame(rgb)
        image_to_show = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        image_h, image_w = image_to_show.shape[:2]

        if not hasattr(self, "ratio"):
            output = subprocess.check_output(["xrandr"]).decode("utf-8")
            current_mode = [line for line in output.splitlines() if "*" in line][0]
            screen_width, screen_height = [int(x) for x in current_mode.split()[0].split("x")]
            scale = self.screen_scale
            screen_w = int(screen_width / scale)
            screen_h = int(screen_height / scale)

            ratio = min(screen_w / image_w, screen_h / image_h)
            self.ratio = ratio
        target_size = (int(image_w * self.ratio), int(image_h * self.ratio))
        image_to_show = cv2.resize(image_to_show, target_size)

        cv2.namedWindow(self.window_name)
        cv2.imshow(self.window_name, image_to_show)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        while not self.done:
            tmp = image_to_show.copy()
            tmp = cv2.circle(tmp, self.current, radius=2, color=(0, 0, 255), thickness=-1)
            for box, box_label in zip(self.boxes, self.box_labels):
                color = (0, 255, 0) if box_label == 1 else (0, 0, 255)
                cv2.rectangle(tmp, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, 2)
            if self.points.shape[0] > 0:
                for ptidx, pt in enumerate(self.points):
                    color = (0, 255, 0) if self.labels[ptidx] == 1 else (0, 0, 255)
                    tmp = cv2.circle(tmp, (int(pt[0]), int(pt[1])), radius=5, color=color, thickness=-1)
            if self.mask is not None:
                mask_to_show = cv2.resize(self.mask.astype(np.uint8), target_size).astype(bool)
                tmp = vis_mask(tmp, mask_to_show.astype(np.uint8), color=[0, 255, 0], alpha=0.5).astype(np.uint8)
            cv2.imshow(self.window_name, tmp)
            waittime = 50
            key = cv2.waitKey(waittime)
            if key == 27 or key == 13:  # ESC hit
                self.done = True
            elif key == ord('r'):
                print("Reset")
                self.reset()
            elif key == ord('p'):
                print("Switch to point mode")
                self.mode = DrawingMode.Point
            elif key == ord('b'):
                print("Switch to box mode")
                self.mode = DrawingMode.Box
            elif key == ord('z'):
                print("Undo")
                if self.mode == DrawingMode.Point and len(self.points) > 0:
                    self.points = self.points[:-1]
                    self.labels = self.labels[:-1]
                    self.detect()
                elif self.mode == DrawingMode.Box and len(self.boxes) > 0:
                    self.boxes = self.boxes[:-1]
                    self.box_labels = self.box_labels[:-1]
                    self.detect()
        cv2.destroyWindow(self.window_name)
        return self.mask

    def track(self, rgb):
        self.rgb = rgb
        _, self.mask = self.predictor.track(self.rgb)
        return self.mask
        
    def close(self):
        del self.predictor
        torch.cuda.empty_cache()
        
        
# class SAMAutoDrawer(object):
#     def __init__(self, window_name="Prompt Drawer", screen_scale=1.0, sam_checkpoint="",
#                  device="cuda", model_type="default"):
#         self.window_name = window_name
#         self.reset()
#         self.screen_scale = screen_scale * 1.2
#         self.screen_scale = screen_scale

#         # Initialize the SAM predictor
#         sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
#         sam.to(device=device)

#         self.predictor = SamPredictor(sam)

#     def reset(self):
#         self.done = False
#         self.drawing = False
#         self.current = (0, 0)
#         self.box = np.zeros([4], dtype=np.float32)
#         self.points = np.empty((0, 2))
#         self.labels = np.empty([0], dtype=int)
#         self.mask = None
#         self.mode = DrawingMode.Box
#         self.boxes = np.zeros([0, 4], dtype=np.float32)
#         self.box_labels = np.empty([0], dtype=int)

#     def detect(self):
#         # Prepare inputs for the predictor
#         if len(self.points) != 0:
#             input_point = self.points / self.ratio
#             input_label = self.labels.astype(int)
#         else:
#             input_point = None
#             input_label = None

#         # Initialize the final mask
#         final_mask = None

#         # Process each box
#         if len(self.boxes) == 0:
#             # If no boxes are present, use only points for detection
#             masks, scores, logits = self.predictor.predict(
#                 point_coords=input_point,
#                 point_labels=input_label,
#                 box=None,
#                 multimask_output=True,
#             )
#             maxidx = np.argmax(scores)
#             final_mask = masks[maxidx].copy()
#         else:
#             # Iterate through all boxes and calculate masks
#             for i in range(len(self.boxes)):
#                 box = self.boxes[i]
#                 box_label = self.box_labels[i]

#                 if np.all(box == 0):
#                     box = None
#                 else:
#                     box = box / self.ratio

#                 # Generate masks for each box
#                 masks, scores, logits = self.predictor.predict(
#                     point_coords=input_point,
#                     point_labels=input_label,
#                     box=box,
#                     multimask_output=True,
#                 )
#                 maxidx = np.argmax(scores)
#                 mask = masks[maxidx]

#                 # Combine masks logically based on the labels
#                 if final_mask is None:
#                     final_mask = mask.copy()
#                 else:
#                     if box_label == 0:
#                         final_mask = np.logical_and(final_mask, ~mask)
#                     else:
#                         final_mask = np.logical_or(final_mask, mask)

#         # Update the mask attribute
#         if final_mask is not None:
#             self.mask = final_mask.copy()
#         elif self.mask is not None:
#             self.mask = np.zeros_like(self.mask)

#     def run(self, rgb):
#         self.rgb = rgb
#         self.predictor.set_image(rgb)
#         image_to_show = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
#         image_h, image_w = image_to_show.shape[:2]

#         if not hasattr(self, "ratio"):
#             output = subprocess.check_output(["xrandr"]).decode("utf-8")
#             current_mode = [line for line in output.splitlines() if "*" in line][0]
#             screen_width, screen_height = [int(x) for x in current_mode.split()[0].split("x")]
#             scale = self.screen_scale
#             screen_w = int(screen_width / scale)
#             screen_h = int(screen_height / scale)

#             ratio = min(screen_w / image_w, screen_h / image_h)
#             self.ratio = ratio
#         target_size = (int(image_w * self.ratio), int(image_h * self.ratio))
#         image_to_show = cv2.resize(image_to_show, target_size)

#         # auto detect bounding box
#         gray_image = cv2.cvtColor(image_to_show, cv2.COLOR_BGR2GRAY)
#         blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
#         edges = cv2.Canny(blurred_image, 50, 200)
        
#         cv2.imshow('Edges Detected', edges)
#         cv2.waitKey(0)
#         # exit()
        
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         contour_image = image_to_show.copy()
#         cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
#         import time
#         import matplotlib.pyplot as plt
#         plt.imshow(cv2.cvtColor(contour_image, cv2.COLOR_BGR2RGB))
#         plt.axis('off')
#         plt.show()
#         time.sleep(3)
#         exit()
        
#         # 图像尺寸
#         image_height, image_width = gray_image.shape

#         # 中心点坐标
#         center_x, center_y = image_width // 2, image_height // 2

#         # 设置一个最小面积过滤
#         min_area = 100

#         # 初始化存储中央衣服轮廓的变量
#         selected_contour = None

#         for contour in contours:
#             # 计算轮廓的边界框
#             x, y, w, h = cv2.boundingRect(contour)
            
#             # 计算面积
#             area = cv2.contourArea(contour)
            
#             # 过滤小的轮廓，确保边界框在图像的中央区域
#             if area > min_area and (center_x - w//2 < x < center_x + w//2) and (center_y - h//2 < y < center_y + h//2):
#                 selected_contour = contour
#                 break

#         # 画出选中的轮廓及其边界框
#         if selected_contour is not None:
#             # 画出边界框
#             x, y, w, h = cv2.boundingRect(selected_contour)
#             cv2.rectangle(image_to_show, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
#         cv2.imshow('Image with Bounding Box', image_to_show)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
#         exit()
            
        

        
#         return self.mask

#     def close(self):
#         del self.predictor
#         torch.cuda.empty_cache()


def shrink_mask(mask_np, shrink_coefficient=0.95):
    """
    Shrink the edges of a binary mask by a specified coefficient.
    
    Args:
    - mask_np (numpy.ndarray): Input binary mask (1 for mask, 0 for background).
    - shrink_coefficient (float): The coefficient by which to shrink the mask (0 < coefficient < 1).
    
    Returns:
    - shrunk_mask (numpy.ndarray): The shrunk binary mask.
    """
    # Convert mask to uint8 type (required for OpenCV operations)
    mask_uint8 = mask_np.astype(np.uint8) * 255

    # Find contours in the binary mask
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Calculate the moments to find the centroid
    M = cv2.moments(mask_uint8)
    if M["m00"] == 0:
        # Handle cases where mask area is zero
        return mask_np
    
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    centroid = np.array([cx, cy])

    # Create an empty mask for the shrunk contour
    shrunk_mask = np.zeros_like(mask_uint8)

    # Iterate through contours and shrink them
    for contour in contours:
        # Calculate new contour points by moving towards the centroid
        shrunk_contour = np.array([
            shrink_coefficient * (point[0] - centroid) + centroid for point in contour
        ], dtype=np.int32)
        
        # Draw the shrunk contours on the empty mask
        cv2.drawContours(shrunk_mask, [shrunk_contour], -1, 255, thickness=cv2.FILLED)
    
    # Convert back to binary format (0 and 1)
    shrunk_mask = (shrunk_mask > 0).astype(np.uint8)
    
    return shrunk_mask
