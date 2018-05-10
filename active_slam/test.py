import numpy as np
import matplotlib.pyplot as plt
import cv2
from scipy.ndimage import gaussian_filter, convolve
# from scipy import signal

# data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)


data = np.mat([[-1,-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1,-1],[100,0,0,0,0,100],[100,0,0,0,0,100],[100,0,0,0,0,100],[-1,-1,-1,-1,-1,-1]])

wall_filter = np.ones((3,3))
print(data)
print('')
print(np.where(convolve(data,wall_filter,mode='constant')==0,1,0))



# points = np.zeros(data.shape)
# # fpoints = np.zeros(data.shape)

# # Approach 1: Slow!
# def near_unkown(data,i,j):
#     x,y = data.shape
#     return data[min(i+1,x-1),j] == -1 or data[max(i-1,0),j] == -1 or data[i,min(j+1,y-1)] == -1 or data[i,max(j-1,0)] == -1

# for i in range(data.shape[0]):
#     for j in range(data.shape[1]):
#         if data[i,j] == 1 and near_unkown(data,i,j):
#             points[i,j] = 1

# # # Approach 2: Faster?
# arr1 = np.array([[-1],[1]])#np.array([[0,-1,0],[0,1,0],[0,0,0]])
# points1 = convolve(data,arr1,mode='constant')
# arr2 = np.array([[1],[-1]])
# points2 = convolve(data,arr1,mode='constant')
# arr3 = np.array([[1,-1]])
# points3 = convolve(data,arr1,mode='constant')
# arr4 = np.array([[-1,1]])
# points4 = convolve(data,arr1,mode='constant')
# fpoints = np.where(points1+.01>=2,1,0) + np.where(points2+.01>=2,1,0) + np.where(points3+.01>=2,1,0) + np.where(points4+.01>=2,1,0) 

# gauss = gaussian_filter(points,1,mode='constant')
# gauss2 = gaussian_filter(points,3,mode='constant')


# n = 4

# flat = gauss2.flatten()
# indices = np.argpartition(flat, -n)[-n:]
# indices = indices[np.argsort(-flat[indices])]
# xs, ys = np.unravel_index(indices, gauss2.shape)
# print(xs,ys,gauss2[xs,ys])

# print(data)
# print(points)
# print(gauss2)

# fig = plt.figure()
# ax1 = fig.add_subplot(2,2,1)
# ax1.imshow(data)
# ax2 = fig.add_subplot(2,2,2)
# ax2.imshow(points)
# ax3 = fig.add_subplot(2,2,3)
# ax3.imshow(points1)
# ax3 = fig.add_subplot(2,2,4)
# ax3.imshow(gauss2)
# plt.show()

