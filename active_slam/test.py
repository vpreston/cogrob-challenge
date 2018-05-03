import numpy as np
import matplotlib.pyplot as plt
import cv2
# from scipy import signal

# data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)


data = np.mat([[-1,-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1,-1],[0,1,1,1,1,0],[0,1,1,1,1,0],[0,1,1,1,1,0],[-1,-1,-1,-1,-1,-1]])
points = np.zeros(data.shape)
# fpoints = np.zeros(data.shape)

# Approach 1: Slow!
def near_unkown(data,i,j):
    x,y = data.shape
    return data[min(i+1,x-1),j] == -1 or data[max(i-1,0),j] == -1 or data[i,min(j+1,y-1)] == -1 or data[i,max(j-1,0)] == -1

for i in range(data.shape[0]):
    for j in range(data.shape[1]):
        if data[i,j] == 1 and near_unkown(data,i,j):
            points[i,j] = 1

point_coords = np.where(points==1)
print(point_coords)

# # Approach 2: Faster?
# arr1 = np.array([[1],[-1]])
# points1 = signal.fftconvolve(data,arr1,'same')
# arr2 = np.array([[-1],[1]])
# points2 = signal.fftconvolve(data,arr2,'same')
# arr3 = np.array([[1,-1]])
# points3 = signal.fftconvolve(data,arr3,'same')
# arr4 = np.array([[-1,1]])
# points4 = signal.fftconvolve(data,arr4,'same')
# fpoints = np.where(points1+.01>=2,1,0) + np.where(points2==2,1,0) + np.where(points3==2,1,0) + np.where(points4==2,1,0) 


fig = plt.figure()
ax1 = fig.add_subplot(2,2,1)
ax1.imshow(data)
ax2 = fig.add_subplot(2,2,2)
ax2.imshow(points)
# ax3 = fig.add_subplot(2,1,2)
# ax3.imshow(im_with_keypoints)
plt.show()
