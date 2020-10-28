#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard\n %s", data.position)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/object_pose", Pose, callback)
    rospy.spin()

if __name__=='__main__':
    listener()