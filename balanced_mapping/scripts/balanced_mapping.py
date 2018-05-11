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

# Imports required for science-mapping functionality
#import imp
from copy import deepcopy
# From Gaussian Team
#import GaussianProcess

class BalancedSampling():
	def __init__(self):
		self.sub = rospy.Subscriber("/map", OccupancyGrid, self.new_map, queue_size=10)

		# flag is if the subscribers want new points or not
		self.sub_flag = rospy.Subscriber("/semaphore", Bool, self.callback, queue_size=10)
		# use rostopic pub -1 /semaphore std_msgs/Bool -- 'True'
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
		self.wall_const = rospy.get_param('balanced_mapping/wall_const')#0.001

		# a larger sigma will correlate kep points farther apart
		self.sigma = rospy.get_param('balanced_mapping/sigma')#20

		# number of pixels from the change
		self.pixel_dist = rospy.get_param('balanced_mapping/pixel_dist')#10

		# whether point selection is done deterministically (choosing top n) or randomly (choosing n in a weighted way)
		self.pick_randomly = rospy.get_param('balanced_mapping/random_selection')#True

		# create a matrix for convolution with map to find points the robot can go
		self.occupancy_filter = np.ones((self.pixel_dist,self.pixel_dist))


		### SCIENCE-MAPPING ONLY PARAMETERS ###
		# science_beta balances between explore/exploit for the science map
		self.science_beta = rospy.get_param('balanced_mapping/science_beta') #0.5

		# this sets the value of exploiting the two features
		self.science_exploit = [(6, 1), (3, 1)]
	
		# these set the offset distance from obstacles (in units of 

	def callback(self,msg):	
		if msg.data:
            	# turn the map into a numpy array
			data = np.asarray(self.map_msg.data, dtype=np.int8).reshape(self.map_msg.info.height, self.map_msg.info.width)
			allpoints = np.zeros(data.shape)


		if self.meta_beta < 0.001:
			
			# get ActiveSLAM points
			points = self.getActiveSLAM(data,allpoints)
			if self.pick_randomly:
				xs, ys, vals = self.find_random_largest(points, self.num_points)
			else:
				xs, ys, vals = self.find_largest(points, self.num_points)

			# publish Active-SLAM points
			rospy.loginfo("Active-SLAM-Only-Rewards generated")
			self.pub.publish(self.create_point_cloud_SLAM(xs, ys, vals))

		elif self.meta_beta > 0.999:
			# TODO: RECONCILE WITH GP TEAM ONCE THEY ARE DONE, RIGHT NOW ONLY RANDOM DATA
			xgp, ygp, rewards_gp = self.getScienceMapping(data)
			
			# rewards is an unsorted list of lists, where each list is [k_ind, science_mapping_reward], where k_ind matches that reward to xgp[k_kind], ygp[k_ind]

			# # Rank rewards from most to least
			sortedRewards = sorted(rewards_gp, key= lambda x: x[1], reverse=True)

			# check that points don't hit any obstacles (TODO: talk to victoria about feasibility check)
			#sortedRewards = self.check_feasibility(xgp, ygp, sortedRewards)
			# after this function call, sorted rewards is now a list of [x_i,y_i,val] lists
			
			# this is TEMP, normally this functionality will be done in sorted rewards
			finalRewards = []
			for el in sortedRewards:
				finalRewards.append([xgp[el[0]],ygp[el[0]],el[1]])
			
			rospy.loginfo("Science-Mapping-Only-Rewards generated")
			rospy.loginfo(finalRewards)
			# publi
			self.pub.publish(self.create_point_cloud(finalRewards, self.num_points))
		else:
			# grab Top SLAM points (These are in the OCCUPANCY GRID MAP)
			# get ActiveSLAM points
			points = self.getActiveSLAM(data,allpoints)
			if self.pick_randomly:
				xs, ys, vals = self.find_random_largest(points, self.num_points)
			else:
				xs, ys, vals = self.find_largest(points, self.num_points)

			slamMax = np.amax(vals)
			
			# Grab top science points
			xgp, ygp, rewards_gp = self.getScienceMapping(data)
			listgp = []
			for el in rewards_gp:
				listgp.append(el[1])

			valgp = np.asarray(listgp)
			scienceMax = np.amax(valgp)

			# Grab Active SLAM reward value at science points
			vals_at_gp = []
			for k in range(len(valgp)):
				vals_at_gp.append(points[xgp[k],ygp[k]])

			vals_at_gp = np.asarray(vals_at_gp )
			# TODO: grab Science values at Top SLAM points (Need actual GP map for this, we can just use randomized data again here
			# TEMP -- only works because we don't have actual GP map to query in ROS
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
			#print(x_all)
			
			# do convex combination of points
			rewards = []
			# worst case: need to go through 2*num_points because we are evaluating at top points from both sets
			for k in range(len(val_SLAM)):
				reward = (1 - self.meta_beta)*(val_SLAM[k]/slamMax) + self.meta_beta*(val_GP[k]/scienceMax)
				
				rewardIdx = [k, reward]
				rewards.append(rewardIdx)
				
			# # Rank rewards from most to least
			sortedRewards = sorted(rewards, key= lambda x: x[1], reverse=True)

			# check that points don't hit any obstacles (TODO: talk to victoria about feasibility check)
			#sortedRewards = self.check_feasibility(x_all, y_all, sortedRewards)
			# after this function call, sorted rewards is now a list of [x,y,val] lists
			# this is TEMP, normally this functionality will be done in check feasibility
			finalRewards = []
			for el in sortedRewards:
				finalRewards.append([x_all[el[0]],y_all[el[0]],el[1]])
			

			rospy.loginfo("Meta-Rewards generated")
			rospy.loginfo(finalRewards)
			self.pub.publish(self.create_point_cloud(finalRewards, min_len))
			
	# save map data
	def new_map(self, data):
		self.map_msg = data
		
	def create_point_cloud(self, sortedRewards, min_len):
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
			p.y = (element[0]*np.cos(yaw) + element[1]*np.sin(yaw))*self.map_msg.info.resolution + offset_y
			p.x = (element[1]*np.cos(yaw) - element[0]*np.sin(yaw))*self.map_msg.info.resolution + offset_x
			c.points.append(p)
			channel.values.append(element[2])
			
		return c

	# feasibility check required when GP points are used
	# TODO: talk to victoria about this - it seems to have an issue with the for-loop now, note that in science_map offset_x and offset_y where not defined, so I may be using the wrong "offsets here"
	def check_feasibility(self,x_all,y_all, rewards):
	    	r = deepcopy(rewards)
		g = []
	    	for element in r:
			px = x_all[element[0]]/self.map_msg.info.resolution - self.map_msg.info.origin.position.x
			py = y_all[element[0]]/self.map_msg.info.resolution - self.map_msg.info.origin.position.y		

			x_query = [px+(i-2) for i in range(0,4)]
			y_query = [py+(i-2) for i in range(0,4)]

			adjusted_x = []
			adjusted_y = []

			for i,x in enumerate(x_query):
				for j,y in enumerate(y_query):
				    if self.map_msg.data[x,y] >= 40. or self.map_msg.data[x,y]==-1.:
					adjusted_x.append(-(x-px))
					adjusted_y.append(-(y-px))
				    else:
					pass
				if len(adjusted_x) != 0 and len(adjusted_y) != 0:
				    if x_all[element[0]]+np.mean(adjusted_x) == x_all[element[0]]and y_all[element[0]]+np.mean(adjusted_y) == y_all[element[0]]:
					pass
				    else:
					g.append([x_all[element[0]]+np.mean(adjusted_x), y_all[element[0]]+np.mean(adjusted_y)])
				else:
				    g.append([x_all[element[0]], y_all[element[0]], element[1]])
        	return g

	######### SCIENCE-MAPPING ONLY CODE ####### 
	def getScienceMapping(self, data):
		# (11 May note this only generated random data now since GP interface not defined clearly to test in ROS)
		# random point, entropy, and science reward gen
		# TODO: should generate from valid points only,not outside SLAM map
		xgp = np.random.randint(2.2*self.map_msg.info.width/5,3*self.map_msg.info.width/5,(self.num_points,))
		ygp = np.random.randint(2.2*self.map_msg.info.height/5,3*self.map_msg.info.height/5,(self.num_points,))

		# in reality, the whole GP will be searched for entropy and mean science, then the best num_points will be taken per below
		entropy = np.random.random((self.num_points,))
		means = np.random.random((self.num_points,))
	
		entropyMax = np.amax(entropy)
		meansMax = np.amax(means)
		rewards = []

		for k in range(self.num_points):
			reward = (1 - self.science_beta)*(means[k]/meansMax) + self.science_beta*(entropy[k]/entropyMax)
		
			rewardIdx = [k, reward]
			rewards.append(rewardIdx)

		return xgp, ygp, rewards
		
		
	######### ACTIVE SLAM CODE ######## (from 11 May 18) 
	def getActiveSLAM(self,data,allpoints):
		valid_map_points = np.where(convolve(data,self.occupancy_filter,mode='constant')==0,1,0) * np.where(data==0,1,0)
		# look through places we know are empty and are next to unknown regions
		for i in range(data.shape[0]):
			for j in range(data.shape[1]):
				if data[i,j] == 0:
					if self.near_unknown(data,i,j):
					    allpoints[i,j] = self.unkown_const
					elif self.near_wall(data,i,j):
					    allpoints[i,j] = self.wall_const

		return gaussian_filter(allpoints,3,mode='constant') * valid_map_points

	# finds the indices and values of n large elements randomly
	def find_random_largest(self,arr,n):
		flat = arr.flatten()
		indices = np.random.choice(len(flat),n,replace=False,p=flat/sum(flat))
		xs,ys = np.unravel_index(indices,arr.shape)
		return xs,ys,arr[xs,ys]
	
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

