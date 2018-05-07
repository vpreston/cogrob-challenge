#!/usr/bin/env python

#credit given to Fiorella Sibona (https://github.com/FiorellaSibona) for tutorials leading to this code example

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

class MoveBaseSeq():
	def __init__(self):
		rospy.init_node('nav_world')
		sequence = rospy.get_param('nav_world/seq')
		angles = rospy.get_param('nav_world/yseq')

		quat_seq = []
		self.pose_seq = []
		self.goals = 0

		for a in angles:
			quat_seq.append(Quaternion(*(quaternion_from_euler(0,0,a*3.14/180, axes='sxyz'))))

		n = 3
		points = [sequence[i:i+n] for i in range(0, len(sequence), n)]

		for p in points:
			self.pose_seq.append(Pose(Point(*p), quat_seq[n-3]))
			n += 1

		rospy.sleep(30.)
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		rospy.loginfo("Waiting for move_base server...")
		wait = self.client.wait_for_server(rospy.Duration(10.0))
		if not wait:
			rospy.logerr("Action server not initialized")
			rospy.signal_shutdown("Action server not initialized. Aborting mission.")
			return
		rospy.loginfo("Connected to server")
		rospy.loginfo("Starting goal navigation")
		self.movebase_client()

	def active_cb(self):
		rospy.loginfo("Goal pose "+str(self.goals+1)+" is now bring processed")

	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for goal pose "+str(self.goals+1)+" received")

	def done_cb(self,status,result):
		self.goals += 1
		if status == 2:
			rospy.loginfo("Goal pose " +str(self.goals)+ " canceled")

		if status == 3:
			rospy.loginfo("Goal pose " + str(self.goals) + " reached")
			if self.goals < len(self.pose_seq):
				next_goal = MoveBaseGoal()
				next_goal.target_pose.header.frame_id = "map"
				next_goal.target_pose.header.stamp = rospy.Time.now()
				next_goal.target_pose.pose = self.pose_seq[self.goals]
				rospy.loginfo("Sending goal pose "+str(self.goals+1))
				self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
			else:
				rospy.loginfo("Final goal reached; shutting down")
				rospy.signal_shutdown("Final goal reached")
				return

		if status == 4:
			rospy.loginfo("Goal pose "+str(self.goals)+ " aborted")
			rospy.signal_shutdown("Pose aborted, Mission aborted")
			return

		if status == 5:
			rospy.loginfo("Goal pose " + str(self.goals)+ "rejected")
			rospy.signal_shutdown("Pose rejected, Mission aborted")
			return

		if status == 8:
			rospy.loginfo("Goal pose " + str(self.goals) + " canceled")

	def movebase_client(self):
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = self.pose_seq[self.goals]
		rospy.loginfo("Sending pose " +str(self.goals+1) + " to server")
		self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
		rospy.spin()


if __name__ == '__main__':
	try:
		MoveBaseSeq()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation test finished.")