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
And if you wanna use ROBOT control, you have to also change line 109
You can see the ROS wiki written in the bitbucket, that explains how to build the environment and use itself.
"""

class UR3ROS():
    def __init__(self):
        rospy.init_node('UR3_control_py') #Initialize the node

        #Define the Variables
        self.currentJoint = [0,0,0,0,0,0,0,0,0,0,0,0] #for inserting the current joint data
        self.step = 1
        self.jointflg = False
        self.npcurrentJoint = np.zeros((1,12))
        self.cvbridge = CvBridge()
        self.saveimglog = ""
        self.i = 1
        self.firsttime = True
        ############################################################
        ###############Variables that you can Adjust################
        ############################################################
        self.logfolder = "./log_robot/"
        self.imglogfolder = self.logfolder+"IMAGE/" #IMAGE LOG FOLDER
        self.jntlogfolder = self.logfolder+"JOINT/" #MOTION LOG FOLDER
        self.simflg = False #True is for SIMULATOR
        self.max_step = 150 # how many times you can take the data
        self.rate = 5 #Hz
        self.logger = True
        ############################################################

        #Publish
        if self.simflg:
            self.pub = rospy.Publisher('/two_arms_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)

            self.joint_names = ["left_arm_shoulder_pan_joint", "left_arm_shoulder_lift_joint", "left_arm_elbow_joint", "left_arm_wrist_1_joint", "left_arm_wrist_2_joint", "left_arm_wrist_3_joint","right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint"]

            self.ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            self.point = JointTrajectoryPoint() #Point should be an object

        else:
            self.l_pub = rospy.Publisher('/left_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
            self.r_pub = rospy.Publisher('/right_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)

            self.l_joint_names = ["left_arm_shoulder_pan_joint", "left_arm_shoulder_lift_joint", "left_arm_elbow_joint", "left_arm_wrist_1_joint", "left_arm_wrist_2_joint", "left_arm_wrist_3_joint"]
            self.r_joint_names = ["right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint"]

            self.l_ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            self.r_ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            self.l_point = JointTrajectoryPoint() #Point should be an object
            self.r_point = JointTrajectoryPoint() #Point should be an objec

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


    #def urStateCallback(self, sub, ros_img): #For SIM
    def urStateCallback(self, r_sub, l_sub, ros_img): #You shold change the args when you change ROBOT or SIM
        #output the data
        if self.simflg == True:
            for i in range(len(sub.actual.positions)):
                #rospy.loginfo("Joint[%s] : %s", i, sub.actual.positions[i])
                self.currentJoint[i] = (sub.actual.positions[i]) #insert right joint to the list

        else:
            for i in range(len(l_sub.position)): #left joint comes fisrt
                #rospy.loginfo("L_Joint[%s] : %s", i, l_sub.position[i])
                self.currentJoint[+i] = (l_sub.position[i]) #insert left joint to the list

            for i in range(len(r_sub.position)): #right data stacks later
                #rospy.loginfo("R_Joint[%s] : %s", i, r_sub.position[i])
                self.currentJoint[6+i] = (r_sub.position[i]) #insert right joint to the list

        print "Joint: ", self.currentJoint

        #insert and stack the data
        if not np.all(self.npcurrentJoint==0):
            tmp = np.reshape(np.array(self.currentJoint),(1,12))
            self.npcurrentJoint = np.concatenate([self.npcurrentJoint, tmp], axis=0)
        else:
            self.npcurrentJoint = np.reshape(np.array(self.currentJoint),(1,12))
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
            self.sub = message_filters.Subscriber('/two_arms_controller/state', JointTrajectoryControllerState, queue_size=1, buff_size=2**24) #for Simulator
        else:
            self.r_sub = message_filters.Subscriber('/right_arm/joint_states', JointState, queue_size=1, buff_size=2**24) #for Robots
            self.l_sub = message_filters.Subscriber('/left_arm/joint_states', JointState, queue_size=1, buff_size=2**24) #for Robots
        self.img = message_filters.Subscriber('/camera/rgb/image_raw',Image, queue_size=1, buff_size=2**24)

        if self.simflg:
            self.ts = message_filters.ApproximateTimeSynchronizer([self.sub, self.img], 10, 1) #Synchronize two data
        else:
            self.ts = message_filters.ApproximateTimeSynchronizer([self.r_sub, self.l_sub, self.img], 10, 0.5, allow_headerless=True)

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
                time.sleep(0.5)
                if np.all(gap<0.01): #You can set the shreshold
                    print "Finish Moving"
                    break
        except KeyboardInterrupt:
            sys.exit()


    def publish(self, goal_point, duration):

        if self.simflg:
            if self.firsttime:
                self.point.positions = goal_point #add Joint Point for SIM
                self.point.time_from_start = rospy.Duration(duration)
                self.ctrl_jnt.goal.trajectory.joint_names = self.joint_names
                self.ctrl_jnt.goal.trajectory.points.append(self.point) #insert the object
                self.ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0,0,0,0,0,0,0]
                self.firsttime = False
            else:
                self.point.positions = goal_point #add Joint Point for SIM
                self.point.time_from_start = rospy.Duration(duration)

            self.pub.publish(self.ctrl_jnt)
            print "sim published!!! : ", self.ctrl_jnt.goal.trajectory.points[0].positions

        else:

            if self.firsttime:
                self.l_point.positions = goal_point[:6] #add Joint Point
                self.l_point.time_from_start = rospy.Duration(duration)
                self.l_ctrl_jnt.goal.trajectory.joint_names = self.l_joint_names
                self.l_ctrl_jnt.goal.trajectory.points.append(self.l_point) #insert the object
                self.l_ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0]

                self.r_point.positions = goal_point[6:] #add Joint Point
                self.r_point.time_from_start = rospy.Duration(duration)
                self.r_ctrl_jnt.goal.trajectory.joint_names = self.r_joint_names
                self.r_ctrl_jnt.goal.trajectory.points.append(self.r_point) #insert the object
                self.r_ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0]
                self.firsttime = False

            else:
                self.l_point.positions = goal_point[:6] #add Joint Point
                self.l_point.time_from_start = rospy.Duration(duration)
                self.r_point.positions = goal_point[6:] #add Joint Point
                self.r_point.time_from_start = rospy.Duration(duration)

            self.l_pub.publish(self.l_ctrl_jnt)
            print "left published!! : ", self.l_ctrl_jnt.goal.trajectory.points[0].positions
            self.r_pub.publish(self.r_ctrl_jnt)
            print "right published!! : ", self.r_ctrl_jnt.goal.trajectory.points[0].positions

        self.doWait(goal_point)
        print "#####################FINISH MOVEMENT########################"
        time.sleep(1)

    def Loop(self):

        time.sleep(1) #We have to wait for the publisher which will have been established
        while not rospy.is_shutdown():

            ########################################################################
            ###################Put joints that you want to move#####################
            ########################################################################
            #self.publish([1.506451964378357, 0.29819822311401367, 0.08147716522216797, 4.60950231552124, 5.394764423370361, -0.1822145620929163, 5.200009822845459, -1.7999499479876917, -1.799917999898092, -3.0819569722950746e-05, 1.299915075302124, -2.39998442331423], 1.0)# Go Initial States and Set Duration(1.0 is the duration)
            #self.publish([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1.0)
            #self.publish([-0.5741803407669067, 0.943168878555298, 0.07171653747558594, 1.602299690246582, 0.402291774749756, -0.18226272264589483, -0.786047458648682, 0.842814270650045, -0.6517179648028772, -1.6196101347552698, 1.3155720233917236, -0.957030502949849904], 1.0) # Go Initial States and Set Duration

            self.publish([1.50, 0.29, 0.08, 4.60, 5.39, -0.18, 5.20, -1.79, -1.79, 0.01, 1.29, -2.39], 10.0)
            self.publish([1.30, 0.39, 0.18, 4.60, 5.39, -0.18, 4.80, -1.49, -1.49, 0.31, 1.29, -2.39], 10.0)
            self.publish([1.50, 0.29, 0.08, 4.60, 5.39, -0.18, 5.20, -1.79, -1.79, 0.01, 1.29, -2.39], 10.0)
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
