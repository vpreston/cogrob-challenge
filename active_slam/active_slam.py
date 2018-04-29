#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

class ActiveSlam():
    def __init__(self):
        self.sub = rospy.Subscriber("/MAP_NAME", MAPTYPE, self.newmap, queue_size=10)
        self.pub = rospy.Publisher("LOCATIONNAME", LOCATIONTYPE, queue_size=10)

    def newmap(self, data):
        pass


if __name__ == '__main__':
    rospy.init_node('active_slam')
    active = ActiveSlam()
    rospy.spin()