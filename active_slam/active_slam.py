#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupanyGrid
from sensor_msgs.msg import PointCloud

class ActiveSlam():
    def __init__(self):
        self.sub = rospy.Subscriber("/map", OccupanyGrid, self.newmap, queue_size=10)
        self.pub = rospy.Publisher("/possible_points", PointCloud, queue_size=10)

    def newmap(self, data):
        pass


if __name__ == '__main__':
    rospy.init_node('active_slam')
    active = ActiveSlam()
    rospy.spin()