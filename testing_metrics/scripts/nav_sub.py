#!/usr/bin/env python

#credit given to Fiorella Sibona (https://github.com/FiorellaSibona) for tutorials leading to this code example

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveBaseSeq():
	def __init__(self):
		rospy.init_node('nav_sub')
		self.ready_for_next = True

		rospy.sleep(20.)
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		rospy.loginfo("Waiting for move_base server...")
		wait = self.client.wait_for_server(rospy.Duration(5.0))
		if not wait:
			rospy.logerr("Action server not initialized")
			rospy.signal_shutdown("Action server not initialized. Aborting mission.")
			return
		rospy.loginfo("Connected to server")
		rospy.loginfo("Starting goal navigation")

		self.sub = rospy.Subscriber("/possible_points", PointCloud, self.cloud_cb, queue_size=10)
		
		while not rospy.is_shutdown():
			rospy.spin()

	def cloud_cb(self, cloud):
		if self.ready_for_next:
			frame_id = cloud.header.frame_id
			points = cloud.points
			vals = cloud.channels

			current_max = -1000
			target = None
			for p,v in zip(points,vals):
				if v > current_max:
					current_max = v
					target = p
			t = [target.x, target.y, 0]
			p_select = Pose(Point(*(t)), Quaternion(*(quaternion_from_euler(0,0,90*3.14/180, axes='sxyz'))))
			goal = MoveBaseGoal()
			goal.target_pose.header.frame_id = frame_id
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.pose = p_select
			rospy.loginfo("Sending new pose to server")
			self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
			self.ready_for_next = False
			rospy.spin()
		else:
			pass
			rospy.spin()

	def active_cb(self):
		rospy.loginfo("Goal pose is now bring processed")

	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for goal pose received")

	def done_cb(self,status,result):
		if status == 2:
			rospy.loginfo("Goal pose canceled")
			self.ready_for_next = True

		if status == 3:
			rospy.loginfo("Goal pose reached")
			self.ready_for_next = True

		if status == 4:
			rospy.loginfo("Goal pose aborted")
			# rospy.signal_shutdown("Pose aborted, Mission aborted")
			return

		if status == 5:
			rospy.loginfo("Goal pose rejected")
			# rospy.signal_shutdown("Pose rejected, Mission aborted")
			return

		if status == 8:
			rospy.loginfo("Goal pose canceled")
			self.ready_for_next = True



if __name__ == '__main__':
	try:
		MoveBaseSeq()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")