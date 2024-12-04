import cv2
import numpy as np
from robotpy_apriltag import AprilTag, AprilTagPoseEstimate, AprilTagDetector

image = cv2.imread(filename="img/test.jpg")
print(type(image))
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像

detector = AprilTagDetector()
detector.addFamily(fam="tag16h5")
tags = detector.detect(gray_image)

for tag in tags:
    cv2.circle(image, (int(tag.getCorner(0).x), int(tag.getCorner(0).y)), 4,(255,0,0), 2) # left-top
    print(tag.getHomographyMatrix())
    print(tag.getFamily())

cv2.imshow("apriltag_test",image)
cv2.waitKey()