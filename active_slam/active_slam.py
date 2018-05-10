#!/usr/bin/env python
# license removed for brevity
import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point
import numpy as np
from scipy.ndimage import gaussian_filter, convolve

class ActiveSlam():
    def __init__(self):
        self.sub = rospy.Subscriber("/map", OccupancyGrid, self.new_map, queue_size=10)
        self.sub_flag = rospy.Subscriber("/semaphore", Bool, self.callback, queue_size=10)
        self.pub = rospy.Publisher("/possible_points", PointCloud, queue_size=10)

        self.map_msg = None

        # weights of exploring new territory vs. remapping known territory
        self.unkown_const = rospy.get_param('active_slam/unknown_const')# 1.
        self.wall_const = rospy.get_param('active_slam/wall_const')#0.

        # a larger sigma will correlate kep points farther apart
        self.sigma = rospy.get_param('active_slam/sigma')#20

        # the number of points we are publishing
        self.num_points = rospy.get_param('active_slam/num_points')#30

        # number of pixels from the change
        self.pixel_dist = rospy.get_param('active_slam/pixel_dist')#10

        # create a matrix for convolution with map to find points the robot can go
        self.occupancy_filter = np.ones((self.pixel_dist,self.pixel_dist))


    # this callback function analyzes map data and publishes a point cloud
    def callback(self, msg):
        if msg.data:
            # turn the map into a numpy array
            data = np.asarray(self.map_msg.data, dtype=np.int8).reshape(self.map_msg.info.height, self.map_msg.info.width)
            points = np.zeros(data.shape)
            valid_map_points = np.where(convolve(data,self.occupancy_filter,mode='constant')==0,1,0) * np.where(data==0,1,0)
            # look through places we know are empty and are next to unknown regions
            for i in range(data.shape[0]):
                for j in range(data.shape[1]):
                    if data[i,j] == 0:
                        if self.near_unknown(data,i,j):
                            points[i,j] = self.unkown_const
                        elif self.near_wall(data,i,j):
                            points[i,j] = self.wall_const

            points = gaussian_filter(points,3,mode='constant') * valid_map_points

            xs, ys, vals = self.find_random_largest(points, self.num_points)
            self.pub.publish(self.create_point_cloud(xs, ys, vals))

    # save map data
    def new_map(self, data):
        self.map_msg = data

    # given map data and a coordinate, this helper function checks whether there are unknown points adjacent to the coordinate
    def near_unknown(self, data,i,j):
        x,y = data.shape
        d = 1
        return data[min(i+d,x-1),j] == -1 or data[max(i-d,0),j] == -1 or data[i,min(j+d,y-1)] == -1 or data[i,max(j-d,0)] == -1

    # given map data and a coordinate, this helper function checks whether there are wall points adjacent to the coordinate
    def near_wall(self, data,i,j):
        x,y = data.shape
        d = 1
        return data[min(i+d,x-1),j] == 100 or data[max(i-d,0),j] == 100 or data[i,min(j+d,y-1)] == 100 or data[i,max(j-d,0)] == 100

    # finds the indices and values of the n largest elements of the array
    def find_largest(self, arr, n):
        flat = arr.flatten()
        indices = np.argpartition(flat, -n)[-n:]
        indices = indices[np.argsort(-flat[indices])]
        xs, ys = np.unravel_index(indices, arr.shape)
        return xs,ys,arr[xs,ys]

    # finds the indices and values of n large elements randomly
    def find_random_largest(self,arr,n):
        flat = arr.flatten()
        indices = np.random.choice(len(flat),n,replace=False,p=flat/sum(flat))
        xs,ys = np.unravel_index(indices,arr.shape)
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
            x = self.map_msg.info.origin.orientation.x
            y = self.map_msg.info.origin.orientation.y
            z = self.map_msg.info.origin.orientation.z
            w = self.map_msg.info.origin.orientation.w
            roll, pitch, yaw = euler_from_quaternion((x,y,z,w))
            offset_x = self.map_msg.info.origin.position.x
            offset_y = self.map_msg.info.origin.position.y
            p.y = (xs[i]*np.cos(yaw) + ys[i]*np.sin(yaw))*self.map_msg.info.resolution + offset_y
            p.x = (ys[i]*np.cos(yaw) - xs[i]*np.sin(yaw))*self.map_msg.info.resolution + offset_x
            c.points.append(p)
            channel.values.append(vals[i])

        return c

if __name__ == '__main__':
    rospy.init_node('active_slam')
    active = ActiveSlam()
    rospy.spin()