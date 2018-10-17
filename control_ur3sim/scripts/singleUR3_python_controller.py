#!/usr/bin/env python

import sys, time
import os, re, glob
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters #Necessary for synchronizing the two data

#Import Message Type
from sensor_msgs.msg import Image #for Kinect
from control_msgs.msg import JointTrajectoryControllerState  #for Subscribe(Sim)
from sensor_msgs.msg import JointState #for Subscribe
from control_msgs.msg import FollowJointTrajectoryActionGoal #for Publish
from trajectory_msgs.msg import JointTrajectoryPoint #for Publish

"""
This code had written by Yoshiaki Nakajima at September 2018.
This code can control simulated UR3 and Real UR3.
At first you should change self.simflg according to the environment in #Variables that you can Adjust# section
You can see the ROS wiki written in the bitbucket, that explains how to build the environment and use itself.
"""

class UR3ROS():
    def __init__(self):
        rospy.init_node('UR3_control_py') #Initialize the node

        #Define the Variables
        self.currentJoint = [0,0,0,0,0,0] #for inserting the current joint data
        self.step = 1
        self.jointflg = False
        self.npcurrentJoint = np.zeros((1,6))
        self.cvbridge = CvBridge()
        self.saveimglog = ""
        self.i = 1
        self.firsttime = True
        ############################################################
        ###############Variables that you can Adjust################
        ############################################################
        self.logfolder = "./log_singlerobot/" #place of log file
        self.imglogfolder = self.logfolder+"IMAGE/" #IMAGE LOG FOLDER
        self.jntlogfolder = self.logfolder+"JOINT/" #MOTION LOG FOLDER
        self.simflg = False #True is for SIMULATOR
        self.max_step = 150 # how many times you can take the data
        self.rate = 5 #Hz
        self.logger = True
        ############################################################

        #Publish
        if self.simflg:
            self.pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)

            self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

            self.ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            self.point = JointTrajectoryPoint() #Point should be an object

        else:
            self.pub = rospy.Publisher('/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
            self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            self.ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            self.point = JointTrajectoryPoint() #Point should be an object

    def SaveJoint(self):
        np.savetxt(self.jntlogfolder+"Joint"+str(self.i)+'.txt',self.npcurrentJoint, fmt='%.8f')
        print self.jntlogfolder+"Joint"+str(self.i)+'.txt'+": Saved!"


    def SaveImg(self, data, step):
        cv2.imwrite(self.saveimglog+"/"+"IMG"+str(step)+".png", data)
        print self.saveimglog+"/"+"IMG"+str(step)+".png"+": Saved!"


    def MakeIMGFolder(self):
        try:
            os.makedirs(self.imglogfolder)
            print self.imglogfolder + "IMG save file were made"
        except:
            print self.imglogfolder + " is already exist!"

        try:
            os.makedirs(self.jntlogfolder)
            print self.jntlogfolder + "JOINT save file were made"
        except:
            print self.jntlogfolder + " is already exist!"

        files = glob.glob(self.imglogfolder+"*")
        #sort the files
        convert = lambda text: int(text) if text.isdigit() else text
        alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
        files.sort(key=alphanum_key)


        for file in files:
            self.saveimglog = self.imglogfolder+"Image"+str(self.i)
            if self.saveimglog == file:
                self.i += 1
        self.saveimglog = self.imglogfolder+"Image"+str(self.i)
        os.makedirs(self.saveimglog+"/")


    def urStateCallback(self, sub, ros_img): #For SIM
        #output the data
        if self.simflg == True:
            for i in range(len(sub.actual.positions)):
                #rospy.loginfo("Joint[%s] : %s", i, sub.actual.positions[i])
                self.currentJoint[i] = (sub.actual.positions[i]) #insert right joint to the list

        else:
            for i in range(len(sub.position)): #left joint comes fisrt
                #rospy.loginfo("Joint[%s] : %s", i, sub.position[i])
                self.currentJoint[+i] = (sub.position[i]) #insert left joint to the list

        print "Joint: ", self.currentJoint

        #insert and stack the data
        if not np.all(self.npcurrentJoint==0):
            tmp = np.reshape(np.array(self.currentJoint),(1,6))
            self.npcurrentJoint = np.concatenate([self.npcurrentJoint, tmp], axis=0)
        else:
            self.npcurrentJoint = np.reshape(np.array(self.currentJoint),(1,6))
            self.MakeIMGFolder() # Save Folders are made in the beginning
            print "First Input"

        #process for img (convert, crop, resize, save)
        img = self.cvbridge.imgmsg_to_cv2(ros_img, "bgr8")
        height, width, channel = img.shape
        cropped_img = img[height/2-480:width/2+480, height/2-480:width/2+480] #cropped in center frame
        resized_img = cv2.resize(img, (128,128))
        #cv2.imshow("log_data",resized_img)
        self.SaveImg(resized_img, self.step)     #save img for each step

        print "STEP: ", self.step, self.npcurrentJoint.shape
        self.step += 1

        try:
            r = rospy.Rate(self.rate)
            r.sleep()
        except:
            pass


    def subscribe(self):
        #Subscribe the Data
        if self.simflg == True:
            self.sub = message_filters.Subscriber('/arm_controller/state', JointTrajectoryControllerState, queue_size=1, buff_size=2**24) #for Simulator
        else:
            self.sub = message_filters.Subscriber('/joint_states', JointState, queue_size=1, buff_size=2**24) #for Robots
        self.img = message_filters.Subscriber('/camera/rgb/image_raw',Image, queue_size=1, buff_size=2**24)


        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub, self.img], 10, 1) #Synchronize two data
        self.ts.registerCallback(self.urStateCallback)
        #########Here you can comment out and test the if individual subdcriber work or not. #######
        #self.img_output= rospy.Subscriber("/camera/rgb/image_raw",Image, self.show_img)
        #self.subtest= rospy.Subscriber("/two_arms_controller/state", JointTrajectoryControllerState, self.sub_test) #SIMULATOR
        #self.subtest= rospy.Subscriber("/right_arm/joint_states", JointState, self.sub_test)
        #self.subtest= rospy.Subscriber("/left_arm/joint_states", JointState, self.sub_test)

    def doWait(self, goalpoint):
        try:
            while True: #Wait for the motion reach the goal
                gap = np.abs(np.array(self.currentJoint) - np.array(goalpoint))
                time.sleep(1)
                print goalpoint, gap
                if np.all(gap<0.01): #You can set the shreshold
                    print "Finish Moving"
                    print gap
                    break
        except KeyboardInterrupt:
            sys.exit()


    def publish(self, goal_point, duration):
        if self.firsttime:
            self.point.positions = goal_point #add Joint Point
            self.point.time_from_start = rospy.Duration(duration)
            self.ctrl_jnt.goal.trajectory.joint_names = self.joint_names
            self.ctrl_jnt.goal.trajectory.points.append(self.point) #insert the object
            self.ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0]
            self.firsttime = False
        else:
            self.point.positions = goal_point
            self.point.time_from_start = rospy.Duration(duration)

        self.pub.publish(self.ctrl_jnt)
        print "published!!! : ", self.ctrl_jnt.goal.trajectory.points[0].positions

        self.doWait(goal_point)
        print "#####################FINISH MOVEMENT########################"
        time.sleep(1)#We have to wait for the publisher which will have been established


    def Loop(self):

        time.sleep(1) #We have to wait for the publisher which will have been established
        while not rospy.is_shutdown():

            ########################################################################
            ###################Put joints that you want to move#####################
            ########################################################################
            #self.publish([0, 0.3, 0, 0, 0, 0], 1.0) # Go Initial States and Set Duration(1.0 is the duration)
            #self.publish([-1, -2, -2, 0, 0, 0], 1.0)
            #self.publish([0,0,0,0,0,0], 1.0)
            self.publish([5.2,-1.8,-1.8,0.0,1.3,-2.4], 10.0)
            self.publish([5.0,-1.6,-1.4,0.2,1.5,-2.6], 10.0)
            self.publish([5.2,-1.8,-1.8,0.0,1.3,-2.4], 10.0)

            #UWAGA !!! You should be careful about the value of joints
            #Because the values for SIM and ROBOT is different
            ########################################################################

            rospy.signal_shutdown('End of Log Steps')
            print "Saving the Joint Data......"
            self.SaveJoint()
            print "Done"
            break
            """
            #for generate the output data
            r = rospy.Rate(self.rate)
            r.sleep()

            if(self.step > self.max_step): #Finish the process if steps are over
                rospy.signal_shutdown('End of Log Steps')
                print "Saving the Joint Data......"
                self.SaveJoint()
                print "Done"
                break
            """


    ##################These two moethods are for testing to subscribe each data################
    def sub_test(sel, data):
        for i in range(len(data.actual.positions)):
            rospy.loginfo("Joint[%s] : %s", i, data.actual.positions[i])

    def show_img(self, data):
        try:
            cv_image = self.cvbridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_output = cv2.resize(cv_image, (128,128))
        cv2.imshow("Origin Image", cv_output)
        cv2.waitKey(1)
    ###########################################################################################

print "Initializing ....."
UR3ROS = UR3ROS()
UR3ROS.subscribe()
UR3ROS.Loop()
