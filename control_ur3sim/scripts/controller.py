#!/usr/bin/env python

import sys
import os
import copy
import time
import rospy
import moveit_msgs.msg
#import moveit_commander #This does not work :(
from control_msgs.msg import JointTrajectoryControllerState  #for Subscribe(Sim)
from sensor_msgs.msg import JointState #for Subscribe(Robot)
from control_msgs.msg import FollowJointTrajectoryActionGoal #for Publish
from trajectory_msgs.msg import JointTrajectoryPoint #for Publish
import numpy as np

currentJoint = [0,0,0,0,0,0]
step = 0
max_step = 50
rate = 0.5 #Hz
goal_point = [5.2,-1.8,-1.8,0.0,1.3,-2.4]
jointflg = False

logger = True
npcurrentJoint = np.zeros((1,6))

def savelog(savedata):
    try:
        os.makedirs("./log/")
        print "file were made"
    except:
        print "./log/ is already exist!"

    np.savetxt('./log/ur3_joint_data'+'.txt',savedata, fmt='%.8f')
    print "Saved!!!"


def urStateCallbackSIM(jointdata): #for simulator
    for i in range(len(jointdata.actual.positions)):
        rospy.loginfo("Joint[%s] : %s", i, jointdata.actual.positions[i])
        currentJoint[i] = jointdata.actual.positions[i]

    global npcurrentJoint, step, max_step

    if not np.all(npcurrentJoint==0):
        tmp = np.reshape(np.array(currentJoint),(1,6))
        npcurrentJoint = np.concatenate([npcurrentJoint, tmp], axis=0)
    else:
        npcurrentJoint = np.reshape(np.array(currentJoint),(1,6))
        print "first input"

    print "STEP: ", step
    step += 1

    if(step >= max_step):
        rospy.signal_shutdown('End of Log')
        savelog(npcurrentJoint)
        exit()

    global rate
    r = rospy.Rate(rate)
    r.sleep()

def urStateCallbackROBOT(jointdata): #for robots
    for i in range(len(jointdata.position)):
        rospy.loginfo("Joint[%s] : %s", i, jointdata.position[i])
        currentJoint[i] = jointdata.position[i]

    global npcurrentJoint, step, max_step

    if not np.all(npcurrentJoint==0):
        tmp = np.reshape(np.array(currentJoint),(1,6))
        npcurrentJoint = np.concatenate([npcurrentJoint, tmp], axis=0)
    else:
        npcurrentJoint = np.reshape(np.array(currentJoint),(1,6))
        print "first input"

    print "STEP: ", step
    step += 1

    #if(step >= max_step):
    #    rospy.signal_shutdown('End of Log')
    #    savelog(npcurrentJoint)
    #    exit()

    global rate
    r = rospy.Rate(5)
    r.sleep()

def main():
    rospy.init_node('py_listener')

    #sub = rospy.Subscriber('arm_controller/state', JointTrajectoryControllerState ,urStateCallbackSIM) #for Simulator
    sub = rospy.Subscriber('/joint_states', JointState ,urStateCallbackROBOT, queue_size=1, buff_size=2**24) #for Robots

    #pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal,queue_size = 20)
    pub = rospy.Publisher('/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)

    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    while not rospy.is_shutdown():
        ctrl_jnt = FollowJointTrajectoryActionGoal() #make object for publisher
        point = JointTrajectoryPoint() #Point should be an object

        point.positions = goal_point #add Joint Point
        point.time_from_start = rospy.Duration(10.0)
        ctrl_jnt.goal.trajectory.joint_names = joint_names
        ctrl_jnt.goal.trajectory.points.append(point)
        ctrl_jnt.goal.trajectory.points[0].velocities = [0,0,0,0,0,0]
        pub.publish(ctrl_jnt)
        print "published!! : ", ctrl_jnt.goal.trajectory.points[0].positions

        global rate
        r = rospy.Rate(rate)
        time.sleep(5)
        point.positions = [0,0,0,0,0,0] #add Joint Point
        pub.publish(ctrl_jnt)
        print "published!! : ", ctrl_jnt.goal.trajectory.points[0].positions
        time.sleep(5)


if __name__ == '__main__':
    #try:
    main()
    #except rospy.ROSInterruptException: pass
