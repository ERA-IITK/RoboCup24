import numpy as np
import cv2
from matplotlib import pyplot as plt

# Define camera matrix K
K = np.array( [[124.47175225,0.,224.71804041],
 [0.,123.73205159,201.10938475],
 [0.,0.,1.]])

# Define distortion coefficients d
d = np.array([-0.2562813,   0.06879545 ,-0.00079856 ,-0.00151498 ,-0.00918441])
# Read an example image and acquire its size
img = cv2.imread("test.jpg")
h, w = img.shape[:2]

# Generate new camera matrix from parameters
newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0)

# Generate look-up tables for remapping the camera image
mapx, mapy = cv2.initUndistortRectifyMap(K, d, None, newcameramatrix, (w, h), 5)

# Remap the original image to a new image
newimg = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

# Display old and new image
fig, (oldimg_ax, newimg_ax) = plt.subplots(1, 2)
oldimg_ax.imshow(img)
oldimg_ax.set_title('Original image')
newimg_ax.imshow(newimg)
newimg_ax.set_title('Unwarped image')
plt.show()