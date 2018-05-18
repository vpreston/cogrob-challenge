#!/usr/bin/env python
# license removed for brevity
import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point
import numpy as np
from scipy.ndimage import gaussian_filter, convolve

# From Gaussian Team
import GaussianProcess

class ImageSpoofing():
	def __init__(self):
		self.sub = rospy.Subscriber("/odom", Odometry, self.new_pose, queue_size=10)
		self.cp = None
		GaussianProcess.setup()

		rospy.init_node('image_spoof')
		rate = rospy.Rate(0.5)
		while not rospy.is_shutdown():
			self.update_gp()
			rate.sleep()


	def new_pose(self, msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		self.cp = [x,y]

	def update_gp(self):
		if self.cp != None:
			# Uncommment this line if you'd like to generate a random environment
			# fake_probs = float(np.random.random(1)[0])
			#Comment out this if-else structure if you'd like to generate a random environment
			if self.cp[0] > 0.0:
				fake_probs = 0.9
			else:
				fake_probs = 0.1
			GaussianProcess.new_image(np.array([fake_probs, 1-fake_probs]),self.cp[0],self.cp[1])


if __name__ == '__main__':
	# everytime odom is published, update the GP with a totally random classification value
	# rospy.init_node('image_spoof')
	active = ImageSpoofing()
	rospy.spin()
