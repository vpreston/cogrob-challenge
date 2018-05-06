#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point
import numpy as np

# From Gaussian Team
import GaussianProcess
# import util -  not sure if we need this

class ScienceMapping():
    def __init__(self):
        #self.sub = rospy.Subscriber("/map", OccupancyGrid, self.newmap, queue_size=10)
        #self.pub = rospy.Publisher("/possible_points", PointCloud, queue_size=10)

    # This function runs the GP processing
    def gpmap(self, ??):
        
        # while True (do we need this?)
        global MAP
        MAP = GaussianProcess.get_image_map()
        
        # NOTE: this MAP has the form (M x N x K) where M x N is the dimensions of the grid and K is the number of habitats. Calling a specific map reference MAP(X, Y, C) = P (c = C|(x,y) = (X,Y))
        
        # Calculate Shannon entropy at each point in the map
        # H(Xi) = - SUM (P(yi = ci| Xi) log2 P(yi = ci| Xi))
        entropy = np.sum(- MAP * np.log2(MAP), axis=2)
    
        # Compute mean function
        # calculated mean by mean = np.sum(likelihoods*np.array(feature_stats)[:,0], axis=1) but this requires a mean science value for each feature, hich we don't have. Can we simulate this in some way??
        
        # Trade-off
        # x_k+1 = argmax[(1-B) * exploit/maxexploit + B * explore/maxexplore]
        #         rewards = (1 - self.beta) * means / means.max() + self.beta * entropies / entropies.max()




    # this callback function analyzes map data and publishes a point cloud
    '''def newmap(self, msg):
        # turn the map into a numpy array
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        points = np.zeros(data.shape)
        # look through places we know are empty and are next to unknown regions
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                if data[i,j] == 100 and self.near_unkown(data,i,j):
                    points[i,j] = 1

        xs, ys = np.where(points==1)
        self.pub.publish(self.create_point_cloud(xs, ys))

    # given map data and a coordinate, this helper function checks whether there are unknown points adjacent to the coordinate
    def near_unkown(self, data,i,j):
        x,y = data.shape
        return data[min(i+1,x-1),j] == -1 or data[max(i-1,0),j] == -1 or data[i,min(j+1,y-1)] == -1 or data[i,max(j-1,0)] == -1

    # this function creates a point cloud
    def create_point_cloud(self, xs, ys):
        c = PointCloud()
        c.header.seq = 1
        c.header.stamp = rospy.Time.now()
        c.header.frame_id = '/map'

        c.points = []

        channel = ChannelFloat32()
        channel.name = "Values"
        channel.values = []

        c.channels = [channel]

        for i in range(len(xs)):
            p = Point()
            p.x = xs[i]
            p.y = ys[i]
            c.points.append(p)
            channel.values.append(1)

        return c
'''


if __name__ == '__main__':
    rospy.init_node('active_slam')
    active = ActiveSlam()
    rospy.spin()
