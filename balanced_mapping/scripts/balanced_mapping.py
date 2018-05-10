#!/usr/bin/env python
# license removed for brevity
import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point
import numpy as np
from scipy.ndimage import gaussian_filter
#import imp
#from copy import deepcopy

# From Gaussian Team
#import GaussianProcess

class BalancedSampling():
	def __init__(self):
		self.sub = rospy.Subscriber("/map", OccupancyGrid, self.new_map, queue_size=10)

		# flag is if the subscribers want new points or not
		self.sub_flag = rospy.Subscriber("/semaphore", Bool, self.callback, queue_size=10)

		# ActiveSlam() and ScienceMapping() nodes both publish to /possible_points topic
		self.pub = rospy.Publisher("/possible_points", PointCloud, queue_size=10)

		self.map_msg = None
        	# balanced sampling parameter: 0-1 ( 0 is Active SLAM only, 1 is Science Mapping only )
		self.meta_beta = rospy.get_param('balanced_mapping/meta_beta') # start with 0
		assert(self.meta_beta >= 0.0 and self.meta_beta <= 1.0),"meta_beta must be between 0 and 1"

		# number of points output 
		self.num_points = rospy.get_param('balanced_mapping/num_points') # 10
		assert(self.num_points > 0),"num_points must be greater than 0!"
		
		### ACTIVE-SLAM ONLY PARAMETERS ###
		# weights of exploring new territory vs. remapping known territory
		self.unkown_const = rospy.get_param('balanced_mapping/unknown_const')# 1.
		self.wall_const = rospy.get_param('balanced_mapping/wall_const')#0.

		# a larger sigma will correlate kep points farther apart
		self.sigma = rospy.get_param('balanced_mapping/sigma')#20

		# number of pixels from the change
		self.pixel_dist = rospy.get_param('balanced_mapping/pixel_dist')#10

	def callback(self,msg):	
		if msg.data:
            # turn the map into a numpy array
			data = np.asarray(self.map_msg.data, dtype=np.int8).reshape(self.map_msg.info.height, self.map_msg.info.width)
			allpoints = np.zeros(data.shape)
			
		if self.meta_beta < 0.001:
			# only do Active-SLAM points
			points = self.getActiveSLAM(data,allpoints)
			xs, ys, vals = self.find_largest(points, self.num_points)
			self.pub.publish(self.create_point_cloud_SLAM(xs, ys, vals))
		elif self.meta_beta > 0.999:
			# TODO: only do Science-Mapping points
				# pending final merge of science mapping
			pass
		else:
			# grab Top SLAM points (These are in the OCCUPANCY GRID MAP)
			points = self.getActiveSLAM(data,allpoints)
			xs, ys, vals = self.find_largest(points, self.num_points)
			slamMax = np.amax(vals)
			
			# TODO: grab Top Science points
				# pending final merge of science mapping (points don't match up now)
				
			# TEMP -- simulated science points 
			xgp = xs
			ygp  = ys
			valgp = np.random.random_sample((self.num_points,))
			scienceMax = np.amax(valgp)
			
			# TODO: grab SLAM values at Top Science points (talk to Lotta about fastest way to do this)
			# TEMP -- only works because we are using same points
			vals_at_gp = vals
			
			# TODO: grab Science values at Top SLAM points (talk to Jesse  about fastest way to do this)
			# TEMP -- only works because we are using same points
			valgp_at_s = valgp
			
			
			# check max length of each vals set 
			min_len = self.num_points
			if len(vals) < min_len:
				min_len = len(vals)
			
			# make combined point set 	
			
			x_all = np.hstack((xs,xgp))
			y_all= np.hstack((ys,ygp))
			val_SLAM = np.hstack((vals,vals_at_gp))
			val_GP = np.hstack((valgp_at_s,valgp))
			print(x_all)
			
			# do convex combination of points
			rewards = []
			# worst case: need to go through 2*num_points because we are evaluating at top points from both sets
			for k in range(len(val_SLAM)):
				reward = (1 - self.meta_beta)*(val_SLAM[k]/slamMax) + self.meta_beta*(val_GP[k]/scienceMax)
				
				rewardIdx = [k, reward]
				rewards.append(rewardIdx)
				
			# # Rank rewards from most to least
			sortedRewards = sorted(rewards, key= lambda x: x[1], reverse=True)
			rospy.loginfo("Meta-Rewards generated")
			rospy.loginfo(sortedRewards)
			self.pub.publish(self.create_point_cloud(x_all, y_all, sortedRewards, min_len))
			
	# save map data
	def new_map(self, data):
		self.map_msg = data
		
	def create_point_cloud(self, x_all, y_all,sortedRewards, min_len):
		c = PointCloud()
		c.header.seq = 1
		c.header.stamp = rospy.Time.now()
		c.header.frame_id = '/map'

		c.points = []

		channel = ChannelFloat32()
		channel.name = "Values"
		channel.values = []

		c.channels = [channel]

		for element in sortedRewards[0:min_len]:
			p = Point()
			x = self.map_msg.info.origin.orientation.x
			y = self.map_msg.info.origin.orientation.y
			z = self.map_msg.info.origin.orientation.z
			w = self.map_msg.info.origin.orientation.w
			roll, pitch, yaw = euler_from_quaternion((x,y,z,w))
			offset_x = self.map_msg.info.origin.position.x
			offset_y = self.map_msg.info.origin.position.y
			p.y = (x_all[element[0]]*np.cos(yaw) + y_all[element[0]]*np.sin(yaw))*self.map_msg.info.resolution + offset_y
			p.x = (y_all[element[0]]*np.cos(yaw) - x_all[element[0]]*np.sin(yaw))*self.map_msg.info.resolution + offset_x
			c.points.append(p)
			channel.values.append(element[1])
			
		return c

	######### ACTIVE SLAM CODE ######## (from 10 May 18) (not sure if we can import the class in ROSpy or not)
	def getActiveSLAM(self,data,allpoints):
		# look through places we know are empty and are next to unknown regions
		for i in range(data.shape[0]):
			for j in range(data.shape[1]):
				if data[i,j] == 0:
					if self.near_unknown(data,i,j):
					    allpoints[i,j] = self.unkown_const
					elif self.near_wall(data,i,j):
					    allpoints[i,j] = self.wall_const

		return gaussian_filter(allpoints,3,mode='constant')

		
	# given map data and a coordinate, this helper function checks whether there are unknown points adjacent to the coordinate
	def near_unknown(self, data,i,j):
		x,y = data.shape
		d = self.pixel_dist
		return data[min(i+d,x-1),j] == -1 or data[max(i-d,0),j] == -1 or data[i,min(j+d,y-1)] == -1 or data[i,max(j-d,0)] == -1
	# given map data and a coordinate, this helper function checks whether there are wall points adjacent to the coordinate
	def near_wall(self, data,i,j):
		x,y = data.shape
		d = self.pixel_dist
		return data[min(i+d,x-1),j] == 100 or data[max(i-d,0),j] == 100 or data[i,min(j+d,y-1)] == 100 or data[i,max(j-d,0)] == 100
	# finds the indices and values of the n largest elements of the array
	def find_largest(self, arr, n):
		flat = arr.flatten()
		indices = np.argpartition(flat, -n)[-n:]
		indices = indices[np.argsort(-flat[indices])]
		xs, ys = np.unravel_index(indices, arr.shape)
		return xs,ys,arr[xs,ys] 
	# this function creates a point cloud
	def create_point_cloud_SLAM(self, xs, ys, vals):
		c = PointCloud()
		c.header.seq = 1
		c.header.stamp = rospy.Time.now()
		c.header.frame_id = '/map'	# this is the /map FRAME, which is different than the /map topic (which gives us the occupancy grid)

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
	rospy.init_node('balanced_mapping')
	active = BalancedSampling()
	rospy.spin()


# def map_to_world(poses,map_info):
#     scale = map_info.resolution
#     angle = quaternion_to_angle(map_info.origin.orientation)
#     # rotation
#     c, s = np.cos(angle), np.sin(angle)
#     # we need to store the x coordinates since they will be overwritten
#     temp = np.copy(poses[:,0])
#     poses[:,0] = c*poses[:,0] - s*poses[:,1]
#     poses[:,1] = s*temp + c*poses[:,1]
#     # scale
#     poses[:,:2] *= float(scale)
#     # translate
#     poses[:,0] += map_info.origin.position.x
#     poses[:,1] += map_info.origin.position.y
#     poses[:,2] += angle

