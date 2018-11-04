
import cv2
import numpy as np

cameraMatrix1 = np.array([[350.042, 0, 323.704],[0, 350.042, 178.72],[0, 0, 1]])
cameraMatrix2 = np.array([[349.99, 0, 341.897],[0, 349.99, 198.233],[0, 0, 1]])
distCoeffs1 = np.array([-0.170118, 0.0230741, 0, 0, 0])
distCoeffs2 = np.array([-0.173119, 0.0261502, 0, 0, 0])
## for zed #### 
Rz, _ = cv2.Rodrigues(np.array([0, 0, 0.000596446]))
Ry, _ = cv2.Rodrigues(np.array([0, 0.015432, 0]))
Rx, _ = cv2.Rodrigues(np.array([0.00414705, 0, 0]))
R = np.dot(Rz, np.dot(Ry, Rx)) # Rz*Ry*Rx
print(R)
###############
#R = np.array([[1.0000,-0.0040,-0.0015],[0.0040, 1.0000,-0.0023],[0.0015, 0.0023, 1.0000]])
T = np.array([ -0.12, 0.0, 0.0])
ImageSize = np.array([480, 752])
R1 = np.zeros((3,3))
R2 = np.zeros((3,3))
P1 = np.zeros((4,3))
P2 = np.zeros((4,3))
Q = np.zeros((4,4))
R1, R2, P1, P2, Q, validPixROI1,validPixROI2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (480, 752), R, T)
f = cv2.FileStorage()
f.open('parameter.yaml', 1)
f.write('LEFT-D', distCoeffs1)
f.write('LEFT-K', cameraMatrix1)
f.write('LEFT-R', R1)
f.write('LEFT-P', P1)
f.write('Right-D', distCoeffs2)
f.write('Right-K', cameraMatrix2)
f.write('Right-R', R2)
f.write('Right-P', P2)

