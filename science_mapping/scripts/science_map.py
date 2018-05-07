#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point
import numpy as np
import imp
import matplotlib.pyplot as plt

# From Gaussian Team
#GaussianProcess = imp.load_source("GaussianProcess", '/home/vpreston/catkin_ws/src/cogrob-challenge/science_mapping/scripts/Gaussian_Processes_Sampling/GaussianProcess.py')
# import util -  not sure if we need this
import GaussianProcess

class ScienceMapping():
    def __init__(self):
        self.sub = rospy.Subscriber("/update_points", Bool, self.getmap_cb, queue_size=10)
        self.pub = rospy.Publisher("/possible_points", PointCloud, queue_size=10)
        rospy.init_node('science_mapping')
        # rate = rospy.Rate(5)
        # while not rospy.is_shutdown():
        #     self.getmap_cb(True)
        #     rate.sleep()

    
    def getmap_cb(self, val):
        if val.data == True:
            science = [(0, 1), (6, 4)]
            m = GaussianProcess.get_image_map()
            entropy = np.sum(- m * np.log2(m), axis=2)
            # plt.imshow(entropy, cmap='viridis', interpolation='nearest')
            # while True:
            #     plt.show()
            # print entropy.shape
            # Find the max entropy value
            entropyMax = np.amax(entropy)
            # print entropyMax

            # Calculate mean value
            means = np.sum(m * np.array(science)[:,0], axis=2)
            # print means
            # plt.imshow(means, cmap='viridis', interpolation='nearest')
            # while True:
            #     plt.show()
            
            # Find max means value
            meansMax = np.amax(means)
            
            # Trade-off function
            beta = 0.5
            rewards = np.zeros((m.shape[0],m.shape[1]))
            rewards.flatten()
            rewards = []
            
            for i in range(0,m.shape[0]):
                for j in range(0,m.shape[1]):
                    reward = (1 - beta)*(means[i,j]/meansMax) + beta*(entropy[i,j]/entropyMax)

                    rewardIdx = [i, j, reward]
                    rewards.append(rewardIdx)

            # # Rank rewards from most to least
            sortedRewards = sorted(rewards, key= lambda x: x[2], reverse=True)#rewards.sort(key=lambda x: x[2])
            print sortedRewards[0:10]
            rospy.loginfo("Logging information now!")
        else:
            pass

        #TODO make into pointcloud object
        #TODO make the 10 a parameter
        #TODO run evaluations on the GP Team stuff (confirm science points look good)


if __name__ == '__main__':
    # rospy.init_node('science_map')
    try:
        active = ScienceMapping()
    except rospy.ROSInterruptException:
        pass
    # rospy.spin()
