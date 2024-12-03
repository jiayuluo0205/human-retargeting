import cv2
import numpy as np
from robotpy_apriltag import AprilTag, AprilTagPoseEstimate, AprilTagDetector
import viser

image = cv2.imread(filename="img/test_image.test_image.png")

server = viser.ViserServer(port=8080)
wxyz = R.from_euler("YZX", [180.0, 45.0, 54.45], degrees=True).as_quat()[[3, 0, 1, 2]]
position = (2.6, 2.6, 2.6)

detector = AprilTagDetector()
detector.addFamily(fam="tag36h11")
X_ImageCamera = np.eye(4)
X_ImageCamera[:2, 3] = np.array((-0.5, -0.5))

server.scene.add_image(image=image, render_width=1, render_height=1, name="color", position=position, wxyz=wxyz)


# cap = cv2.VideoCapture(0)
# detector = AprilTagDetector()
# detector.addFamily(fam="tag36h11")
# detector.addFamily(fam="tag25h9")

# i=0
# while(1):
#     # 获得图像
#     ret, frame = cap.read()
#     # 检测按键
#     k=cv2.waitKey(1)
#     if k==27:
#         break
#     elif k==ord('s'):
#         cv2.imwrite('E:/OpenCV_pic/'+str(i)+'.jpg', frame)
#         i+=1
#     # 检测apriltag
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     tags = detector.detect(gray)
#     for tag in tags:
#         # cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
#         # cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
#         # cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
#         # cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom

#         cv2.circle(frame, (int(tag.getCorner(0).x), int(tag.getCorner(0).y)), 4,(255,0,0), 2)
#         cv2.circle(frame, (int(tag.getCorner(1).x), int(tag.getCorner(1).y)), 4,(255,0,0), 2)
#         cv2.circle(frame, (int(tag.getCorner(2).x), int(tag.getCorner(2).y)), 4,(255,0,0), 2)
#         cv2.circle(frame, (int(tag.getCorner(3).x), int(tag.getCorner(3).y)), 4,(255,0,0), 2)
#     # 显示检测结果
#     cv2.imshow('capture', frame)

# cap.release()
# cv2.destroyAllWindows()

