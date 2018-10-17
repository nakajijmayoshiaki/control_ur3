#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import message_filters #Necessary for synchronizing the two data
from control_msgs.msg import JointTrajectoryControllerState  #for Subscribe(Sim)
from sensor_msgs.msg import Image #img msg

#This code is for testing if the message_filter works well or not.
class UR3ROS():
    def __init__(self):
        rospy.init_node('py_listener')
        self.sub = message_filters.Subscriber('/two_arms_controller/state', JointTrajectoryControllerState, queue_size=1, buff_size=2**24) #for Simulator
        self.img = message_filters.Subscriber('/camera/rgb/image_raw',Image, queue_size=1, buff_size=2**24)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub, self.img], 10, 1)
        self.ts.registerCallback(self.urStateCallback)

        #########Here you can comment out and test the if individual subdcriber work or not.#######
        #self.img_output= rospy.Subscriber("/camera/rgb/image_raw",Image, self.show_img)
        #self.subtest= rospy.Subscriber("/two_arms_controller/state", JointTrajectoryControllerState, self.sub_test)


    def urStateCallback(self, sub, ros_img):
        print "Synchronized!"
        for i in range(len(sub.actual.positions)):
            rospy.loginfo("Joint[%s] : %s", i, sub.actual.positions[i])

        cvbridge = CvBridge()
        try:
            cv_image = cvbridge.imgmsg_to_cv2(ros_img, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_output = cv2.resize(cv_image, (128,128))
        cv2.imshow("Origin Image", cv_output)
        cv2.waitKey(1)

    def sub_test(sel, data):
        for i in range(len(data.actual.positions)):
            rospy.loginfo("Joint[%s] : %s", i, data.actual.positions[i])

    def show_img(self, data):
        cvbridge = CvBridge()
        try:
            cv_image = cvbridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_output = cv2.resize(cv_image, (128,128))
        cv2.imshow("Origin Image", cv_output)
        cv2.waitKey(1)


Listener = UR3ROS()

rospy.spin()
