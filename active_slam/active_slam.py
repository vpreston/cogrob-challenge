#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point
import numpy as np
from scipy.ndimage import gaussian_filter

class ActiveSlam():
    def __init__(self):
        self.sub = rospy.Subscriber("/map", OccupancyGrid, self.newmap, queue_size=10)
        self.sub_flag = rospy.Subscriber("/semaphore", Bool, self.callback, queue_size=10)
        self.pub = rospy.Publisher("/possible_points", PointCloud, queue_size=10)

        self.map_msg = None

        # weights of exploring new territory vs. remapping known territory
        self.unkown_const = 1.
        self.wall_const = .5

        # a larger sigma will correlate kep points farther apart
        self.sigma = 3

        # the number of points we are publishing
        self.num_points = 10


    # this callback function analyzes map data and publishes a point cloud
    def callback(self, msg):
        if msg.data:
            # turn the map into a numpy array
            data = np.asarray(self.map_msg.data, dtype=np.int8).reshape(self.map_msg.info.height, self.map_msg.info.width)
            points = np.zeros(data.shape)
            # look through places we know are empty and are next to unknown regions
            for i in range(data.shape[0]):
                for j in range(data.shape[1]):
                    if data[i,j] == 100
                        if self.near_unkown(data,i,j):
                            points[i,j] = self.unkown_const
                        elif self.near_wall(data,i,j):
                            points[i,j] = self.wall_const

            points = gaussian_filter(points,3,mode='constant')

            xs, ys, vals = self.find_largest(points, self.num_points)
            self.pub.publish(self.create_point_cloud(xs, ys, vals))

    # given map data and a coordinate, this helper function checks whether there are unknown points adjacent to the coordinate
    def near_unkown(self, data,i,j):
        x,y = data.shape
        return data[min(i+1,x-1),j] == -1 or data[max(i-1,0),j] == -1 or data[i,min(j+1,y-1)] == -1 or data[i,max(j-1,0)] == -1

    # given map data and a coordinate, this helper function checks whether there are unknown points adjacent to the coordinate
    def near_unkown(self, data,i,j):
        x,y = data.shape
        return data[min(i+1,x-1),j] == 0 or data[max(i-1,0),j] == 0 or data[i,min(j+1,y-1)] == 0 or data[i,max(j-1,0)] == 0

    # finds the indices and values of the n largest elements of the array
    def find_largest(self, arr, n):
        flat = arr.flatten()
        indices = np.argpartition(flat, -n)[-n:]
        indices = indices[np.argsort(-flat[indices])]
        xs, ys = np.unravel_index(indices, arr.shape)
        return xs,ys,arr[xs,ys] 

    # this function creates a point cloud
    def create_point_cloud(self, xs, ys, vals):
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
            channel.values.append(vals[i])

        return c



if __name__ == '__main__':
    rospy.init_node('active_slam')
    active = ActiveSlam()
    rospy.spin()