import cv2
import numpy as np

# read image
img_left = cv2.imread("./left_image")
img_right = cv2.imread("./right_image")
img_size = img_left.shape
#(480, 752)

# K1 K2 D1 D2 R T
fs = cv2.FileStorage("./stereo_calibrate_matlab.yaml", cv2.FILE_STORAGE_READ)
K1 = fs.getNode("K1")
K2 = fs.getNode("K2")
D1 = fs.getNode("D1")
D2 = fs.getNode("D2")
R = fs.getNode("R")
T = fs.getNode("T")

# calculate  R and P
R1, R2, P1, P2, Q, validPixROI1,validPixROI2 = cv2.stereoRectify(K1, D1, K2, D2, img_size, R, T)

# using R and P undistor origin image

# cal map
lmapx, lmapy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_32FC1)
rmapx, rmapy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_32FC1)

# remap, recitify two image

left_rectify = cv2.remap(img_left, lmapx, lmapy, cv::INTER_LINEAR)
right_rectify = cv2.remap(img_right, rmapx, rmapy, cv::INTER_LINEAR)

# imshow rectified image
rectified_image = np.array((img_size[0], img_size[1]*2))
rectified_image[:, 0:img_size[1]-1] = left_rectify
rectified_image[:, img_size[1]:img_size[1]*2-1] = right_rectify
cv2.imshow("rectified image", rectified_image)