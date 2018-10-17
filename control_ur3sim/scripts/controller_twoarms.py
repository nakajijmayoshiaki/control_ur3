#!/usr/bin/env python

import sys
import os, re, glob
import copy
import time
import rospy
import cv2
import numpy as np
import moveit_msgs.msg
from sensor_msgs.msg import Image #img msg
from cv_bridge import CvBridge, CvBridgeError
import message_filters #Necessary for synchronizing the two data
#import moveit_commander #This does not work :(
from control_msgs.msg import JointTrajectoryControllerState  #for Subscribe(Sim)
from sensor_msgs.msg import JointState #for Subscribe
from control_msgs.msg import FollowJointTrajectoryActionGoal #for Publish
from trajectory_msgs.msg import JointTrajectoryPoint #for Publish

# define variables
currentJoint = [0,0,0,0,0,0,0,0,0,0,0,0] #for inserting the current joint data
step = 1
jointflg = False
npcurrentJoint = np.zeros((1,6))
cvbridge = CvBridge()

############################################################
###############Variables that you can Adjust################
############################################################
imglogfolder = "./log/img/" #IMAGE LOG FOLDER
saveimglog = ""
simflg = True
max_step = 50 # how many times you can take the data
rate = 10 #Hz
l_goal_point = [-81,-151,-51,-88,109,351]
r_goal_point = [83,-19,57,3.2,104,3]
two_goal_point = [0,0.9,0.8,0.2,0.5,0.5,0.5,0.5,0.5,0.5,-0.3,0] #for SIM
logger = True

def SaveLog(savedata):
    try:
        os.makedirs("./log/")
        print "file were made"
    except:
        print "./log/ is already exist!"

    np.savetxt('./log/ur3_joint_data'+'.txt',savedata, fmt='%.8f')
    print "Saved!"

def MakeIMGFolder():
    global imglogfolder, saveimglog
    try:
        os.makedirs(imglogfolder)
        print "Log file were made"
    except:
        print imglogfolder + " is already exist!"

    files = glob.glob(imglogfolder+"*")
    #sort the files
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    files.sort(key=alphanum_key)

    i = 1
    print files
    for file in files:
        saveimglog = imglogfolder+"IMG"+str(i)
        if saveimglog == file:
            i += 1
    saveimglog = imglogfolder+"IMG"+str(i)
    os.makedirs(saveimglog+"/")


def SaveImg(data, step):
    global saveimglog
    cv2.imwrite(saveimglog+"/"+str(step)+".png", data)
    print "Image data saved!"


def urStateCallback(sub, ros_img):
    global npcurrentJoint, step, max_step, rate, robot

    #output the data
    if simflg == True:
        for i in range(len(sub.actual.positions)):
            rospy.loginfo("Joint[%s] : %s", i, sub.actual.positions[i])
            currentJoint[i] = (sub.actual.positions[i]) #insert right joint to the list

    else:
        for i in range(len(r_sub.position)):
            rospy.loginfo("R_Joint[%s] : %s", i, r_sub.position[i])
            currentJoint[i] = (r_sub.position[i]) #insert right joint to the list

        for i in range(len(l_sub.position)):
            rospy.loginfo("L_Joint[%s] : %s", i, l_sub.position[i])
            currentJoint[5+i] = (l_sub.position[i]) #insert left joint to the list

    #insert and stack the data
    if not np.all(npcurrentJoint==0):
        tmp = np.reshape(np.array(currentJoint),(1,12))
        npcurrentJoint = np.concatenate([npcurrentJoint, tmp], axis=0)
    else:
        npcurrentJoint = np.reshape(np.array(currentJoint),(1,12))
        MakeIMGFolder()
        print "First Input"

    #process for img (convert, crop, resize, save)
    img = cvbridge.imgmsg_to_cv2(ros_img, "rgb8")
    height, width, channel = img.shape
    cropped_img = img[height/2-480:width/2+480, height/2-480:width/2+480] #cropped in center frame
    resized_img = cv2.resize(img, (128,128))
    #cv2.imshow("log_data",resized_img)
    SaveImg(resized_img, step)     #save img for each step

    print "STEP: ", step, npcurrentJoint.shape
    step += 1

    r = rospy.Rate(rate)
    r.sleep()

    if(step >= max_step):
        rospy.signal_shutdown('End of Log')
        print "Saving the Joint Data......"
        SaveLog(npcurrentJoint, step)
        print "Done"
        sys.exit()

"""
def sub_test(data):
    for i in range(len(data.actual.positions)):
        rospy.loginfo("Joint[%s] : %s", i, data.actual.positions[i])
        currentJoint[i] = (data.actual.positions[i]) #insert right joint to the list

def show_img(data):
    try:
        cv_image = cvbridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv_output = cv2.resize(cv_image, (128,128))
    cv2.imshow("Origin Image", cv_output)
    cv2.waitKey(1)
"""



def main():
    rospy.init_node('py_listener')

    #Subscribe
    global simflg
    if simflg == True:
        sub = message_filters.Subscriber('/two_arms_controller/state', JointTrajectoryControllerState, queue_size=1, buff_size=2**24) #for Simulator
    else:
        l_sub = message_filters.Subscriber('/left_arm/joint_states', JointState, queue_size=1, buff_size=2**24) #for Robots
        r_sub = message_filters.Subscriber('/right_arm/joint_states', JointState, queue_size=1, buff_size=2**24) #for Robots
    img = message_filters.Subscriber('/camera/rgb/image_raw',Image, queue_size=1, buff_size=2**24)

    if simflg:
        ts = message_filters.ApproximateTimeSynchronizer([sub, img], 10, 1)
    else:
        ts = message_filters.ApproximateTimeSynchronizer([l_sub, r_sub, img], 10, 0.5, allow_headerless=True)

    ts.registerCallback(urStateCallback)

    #img_output= rospy.Subscriber("/camera/rgb/image_raw",Image, show_img)
    #subtest= rospy.Subscriber("/two_arms_controller/state", JointTrajectoryControllerState, sub_test)


    #Publish
    if simflg:
        pub = rospy.Publisher('/two_arms_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
        joint_names = ["left_arm_shoulder_pan_joint", "left_arm_shoulder_lift_joint", "left_arm_elbow_joint", "left_arm_wrist_1_joint", "left_arm_wrist_2_joint", "left_arm_wrist_3_joint","right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint"]
    else:
        l_pub = rospy.Publisher('/left_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
        r_pub = rospy.Publisher('/right_arm/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)

        l_joint_names = ["left_arm_shoulder_pan_joint", "left_arm_shoulder_lift_joint", "left_arm_elbow_joint", "left_arm_wrist_1_joint", "left_arm_wrist_2_joint", "left_arm_wrist_3_joint"]
        r_joint_names = ["right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint"]


    while not rospy.is_shutdown():

        if simflg:
            ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            point = JointTrajectoryPoint() #Point should be an object

            point.positions = two_goal_point #add Joint Point for SIM
            point.time_from_start = rospy.Duration(10.0)
            ctrl_jnt.goal.trajectory.joint_names = joint_names
            ctrl_jnt.goal.trajectory.points.append(point) #insert the object
            ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0,0,0,0,0,0,0]

            pub.publish(ctrl_jnt)
            print "sim published!! : ", ctrl_jnt.goal.trajectory.points[0].positions

        else:

            l_ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            r_ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
            l_point = JointTrajectoryPoint() #Point should be an object
            r_point = JointTrajectoryPoint() #Point should be an object

            l_point.positions = l_goal_point #add Joint Point
            l_point.time_from_start = rospy.Duration(10.0)
            l_ctrl_jnt.goal.trajectory.joint_names = l_joint_names
            l_ctrl_jnt.goal.trajectory.points.append(l_point) #insert the object
            l_ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0]

            r_point.positions = r_goal_point #add Joint Point
            r_point.time_from_start = rospy.Duration(10.0)
            r_ctrl_jnt.goal.trajectory.joint_names = r_joint_names
            r_ctrl_jnt.goal.trajectory.points.append(r_point) #insert the object
            r_ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0]

            l_pub.publish(l_ctrl_jnt)
            print "left published!! : ", l_ctrl_jnt.goal.trajectory.points[0].positions
            r_pub.publish(r_ctrl_jnt)
            print "right published!! : ", r_ctrl_jnt.goal.trajectory.points[0].positions

        global rate
        r = rospy.Rate(rate)
        r.sleep()


if __name__ == '__main__':
    #try:
    main()
    #except rospy.ROSInterruptException: pass
