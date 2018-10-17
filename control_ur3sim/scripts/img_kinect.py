#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class GetImage:
    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)

        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # ウインドウのサイズを変更
        cv_half_image = cv2.resize(cv_image,   (0,0),fx=0.5, fy=0.5)

        # ウインドウ表示
        cv2.imshow("Origin Image", cv_half_image)

        cv2.waitKey(1)

#def main():
#    gi = GetImage()

#    try:
#        rospy.spin()
#    except KeyboardInterrupt:
#        print("Shutting down")
#        cv2.destroyAllWindows()
