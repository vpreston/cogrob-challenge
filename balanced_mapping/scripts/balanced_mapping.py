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
from copy import deepcopy
# From Gaussian Team
import GaussianProcess

class BalancedSampling():
	def __init__(self):
		self.sub = rospy.Subscriber("/map", OccupancyGrid, self.new_map, queue_size=10)

		# flag is if the subscribers want new points or not
		self.sub_flag = rospy.Subscriber("/semaphore", Bool, self.callback, queue_size=10)
		# use rostopic pub -1 /semaphore std_msgs/Bool -- 'True'
		# ActiveSlam() and ScienceMapping() nodes both publish to /possible_points topic
		self.pub = rospy.Publisher("/possible_points", PointCloud, queue_size=10)
		self.debug = rospy.Publisher("/debug", Bool, queue_size=10)

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

		self.viz_num = 0
	
		# these set the offset distance from obstacles (in units of 

	def callback(self,msg):	
		if msg.data:
            	# turn the map into a numpy array
			data = np.asarray(self.map_msg.data, dtype=np.int8).reshape(self.map_msg.info.height, self.map_msg.info.width)
			allpoints = np.zeros(data.shape)


		if self.meta_beta < 0.001:
			# Uncomment these lines if you'd like to watch the Gaussian Process map be generated
			# science = [(6,1),(3,1)]
			# m = GaussianProcess.GPRegressor()
			# rospy.loginfo("GP Regressor initialized")
			# for i,element in enumerate(science):
			# 	m.visualize(c=i, file_path='/home/vpreston/Documents/misc/random/{}_{}.png'.format(i, self.viz_num))
			# self.viz_num += 1

			# get ActiveSLAM points
			points = self.getActiveSLAM(data,allpoints)
			if self.pick_randomly:
				xs, ys, vals = self.find_random_largest(points, self.num_points)
			else:
				xs, ys, vals = self.find_largest(points, self.num_points)

			# publish Active-SLAM points
			rospy.loginfo("Active-SLAM-Only-Rewards generated")
			self.pub.publish(self.create_point_cloud_SLAM(xs, ys, vals))

		elif self.meta_beta > 0.9:
			# get Science points
			sortedRewards = self.getScienceMapping(data)
			rospy.loginfo("Science-Mapping-Only-Rewards generated")
			self.pub.publish(self.create_point_cloud(sortedRewards, self.num_points))
		
		else:
			rewards = []
			# grab Top SLAM points (These are in the OCCUPANCY GRID MAP)
			# get ActiveSLAM points
			points = self.getActiveSLAM(data,allpoints)
			if self.pick_randomly:
				xs, ys, slam_slam_vals = self.find_random_largest(points, self.num_points)
			else:
				xs, ys, slam_slam_vals = self.find_largest(points, self.num_points)
			slamMax = np.amax(slam_slam_vals)
			
			# get active slam points from GP map
			xs_gp_map, ys_gp_map, slam_gp_vals = self.getScienceVals(xs, ys)

			
			# now let's do the reverse and get science maps, and find their slam value
			# Grab top science points
			rewards_gp = self.getScienceMapping(data)
			gp_x = [m[0] for m in rewards_gp]
			gp_y = [m[1] for m in rewards_gp]
			gp_gp_vals = [m[2] for m in rewards_gp]
			gpMax = np.amax(gp_gp_vals)

			# Grab Active SLAM reward value at science points
			gp_slam_vals = []

			for k in range(len(rewards_gp)):
				slam_x = int((rewards_gp[k][0]/self.map_msg.info.resolution) - self.map_msg.info.origin.position.x+self.map_msg.info.width/2)
				slam_y = int((rewards_gp[k][1]/self.map_msg.info.resolution) - self.map_msg.info.origin.position.y+self.map_msg.info.height/2)
				
				if slam_x < 383 and slam_y < 383:
					gp_slam_vals.append(points[slam_x,slam_y])

			# get the single rewards for these slam points
			for x,y,slam_val,gp_val in zip(xs_gp_map, ys_gp_map, slam_slam_vals, slam_gp_vals):
				reward = (1 - self.meta_beta)*(slam_val/slamMax) + self.meta_beta*(gp_val/gpMax)
				rewards.append([x,y,reward])

			# now get the single rewards for these gp points
			for x,y,slam_val,gp_val in zip(gp_x, gp_y, gp_slam_vals, gp_gp_vals):
				reward = (1 - self.meta_beta)*(slam_val/slamMax) + self.meta_beta*(gp_val/gpMax)
				rewards.append([x,y,reward])

			# now sort
			sortedRewards = sorted(rewards, key= lambda x: x[2], reverse=True)
			finalRewards = sortedRewards
			
			# make sure returning is valid
			min_len = self.num_points
			if len(finalRewards) < min_len:
				min_len = len(finalRewards)

			rospy.loginfo("Meta-Rewards generated")
			# rospy.loginfo(finalRewards[0:min_len])
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

		i = 0
		data = np.asarray(self.map_msg.data, dtype=np.int8).reshape(self.map_msg.info.height, self.map_msg.info.width)
		valid = np.where(convolve(data,self.occupancy_filter,mode='constant')==0,1,0) * np.where(data < 5,1,0)
		for element in sortedRewards:
			if self.check_feasibility(element[1],element[0], valid) == True and i < min_len:
				i += 1
				p = Point()
				p.y = element[1]
				p.x = element[0]
				c.points.append(p)
				channel.values.append(element[2])
		# rospy.loginfo(c)
		rospy.loginfo('Generating points for publishing')
			
		return c

	# feasibility check required when GP points are used
	# TODO: talk to victoria about this - it seems to have an issue with the for-loop now, note that in science_map offset_x and offset_y where not defined, so I may be using the wrong "offsets here"
	def check_feasibility(self, x, y, valid_map_points):
		px = int((x/self.map_msg.info.resolution) - self.map_msg.info.origin.position.x+self.map_msg.info.width/2)
		py = int((y/self.map_msg.info.resolution) - self.map_msg.info.origin.position.y+self.map_msg.info.height/2)

		x_query = [px+(i-2) for i in range(0,4)]
		y_query = [py+(i-2) for i in range(0,4)]

		for i,x in enumerate(x_query):
			for j,y in enumerate(y_query):
				if x > 383 or y > 383:
					return False
				else:
					if int(valid_map_points[x,y]) == 0:
						return False
		return True


	######### SCIENCE-MAPPING ONLY CODE ####### 
	def getScienceMapping(self, data):
		science = [(6,1),(3,1)]

		# creates an object which allows us to query data points
		# Note! data points to query need to be in the map frame!
		m = GaussianProcess.GPRegressor()
		rospy.loginfo("GP Regressor initialized")

		for i,element in enumerate(science):
			m.visualize(c=i, file_path='/home/vpreston/Documents/misc/random/{}_{}.png'.format(i, self.viz_num))

		self.viz_num += 1

		# conversion constants to world frame
		x = self.map_msg.info.origin.orientation.x
		y = self.map_msg.info.origin.orientation.y
		z = self.map_msg.info.origin.orientation.z
		w = self.map_msg.info.origin.orientation.w
		roll, pitch, yaw = euler_from_quaternion((x,y,z,w))
		offset_x = self.map_msg.info.origin.position.x
		offset_y = self.map_msg.info.origin.position.y
		coords = []
		entropy = []
		mean = []
		rewards = []

		rospy.loginfo(data.shape[0])
		rospy.loginfo(data.shape[1])

		for i in range(data.shape[0]):
			for j in range(data.shape[1]):
				py = (i*np.cos(yaw) + j*np.sin(yaw))*self.map_msg.info.resolution + offset_y
				px = (j*np.cos(yaw) - i*np.sin(yaw))*self.map_msg.info.resolution + offset_x
				if True:
					ent = np.sum(- m(px,py) * np.log2(m(px,py)))
					me = np.sum(m(px,py)*np.array(science)[:,0])
					entropy.append(ent)
					mean.append(me)
					coords.append([px,py,ent,me])
				else:
					pass

		maxEntropy = max(entropy)
		maxMean = max(mean)

		for point in coords:
			reward = (1-self.science_beta)*(point[3]/maxMean) + self.science_beta*(point[2]/maxEntropy)
			rewardIdx = [point[0],point[1],reward]
			rewards.append(rewardIdx)

		sortedRewards = sorted(rewards, key=lambda x: x[2], reverse=True)
		return sortedRewards


	def getScienceVals(self, xs, ys):
		# gets a list of occupancy points and gets their info
		science = [(6,1),(3,1)]

		# creates an object which allows us to query data points
		# Note! data points to query need to be in the map frame!
		m = GaussianProcess.GPRegressor()

		# conversion constants to world frame
		x = self.map_msg.info.origin.orientation.x
		y = self.map_msg.info.origin.orientation.y
		z = self.map_msg.info.origin.orientation.z
		w = self.map_msg.info.origin.orientation.w
		roll, pitch, yaw = euler_from_quaternion((x,y,z,w))
		offset_x = self.map_msg.info.origin.position.x
		offset_y = self.map_msg.info.origin.position.y
		coords = []
		entropy = []
		mean = []
		rewards = []
		fx = []
		fy = []

		for i,j in zip(xs, ys):
			py = (i*np.cos(yaw) + j*np.sin(yaw))*self.map_msg.info.resolution + offset_y
			px = (j*np.cos(yaw) - i*np.sin(yaw))*self.map_msg.info.resolution + offset_x
			ent = np.sum(- m(px,py) * np.log2(m(px,py)))
			me = np.sum(m(px,py)*np.array(science)[:,0])
			entropy.append(ent)
			mean.append(me)
			coords.append([px,py,ent,me])

		maxEntropy = max(entropy)
		maxMean = max(mean)

		for point in coords:
			reward = (1-self.science_beta)*(point[3]/maxMean) + self.science_beta*(point[2]/maxEntropy)
			fx.append(point[0])
			fy.append(point[1])
			rewards.append(reward)

		return fx, fy, rewards

		
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
