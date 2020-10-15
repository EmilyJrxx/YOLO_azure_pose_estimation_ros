#!/usr/bin/env python

import rospy, sys, numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import message_filters
from std_msgs.msg import Header

class k4a_vision:
    def __init__(self):
        rospy.init_node("k4a_vision", anonymous = False)
        self.bridge = cv_bridge.CvBridge()
        self.rgb_topic = '/rgb/image_raw'
        self.depth_topic = '/depth_to_rgb/image_raw'
        self.cloud_topic = '/points2'
        self.image_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        # self.cloud_sub = message_filters.Subscriber(self.cloud_topic, PointCloud2)
        self.ts        = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        self.ts.registerCallback(image_cloud_callback)

    def image_cloud_callback(self, rgb, depth):
        # BEGIN BRIDGE
        img_rgb   = self.bridge.imgmsg_to_cv2(rgb, desired_encoding = 'bgr8')
        img_depth = self.bridge.imgmsg_to_cv2(depth, desired_encoding = 'bgr8')
        cv2.imshow("Azure RGB", img_rgb)
        cv2.waitKey(1)
        cv2.imshow("Azure Depth", img_depth)
        cv2.waitKey(1)
        
vision = k4a_vision()
rospy.spin()