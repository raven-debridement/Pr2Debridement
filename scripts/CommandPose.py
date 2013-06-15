#!/usr/bin/env python

# Python Arm Wrapper for the PR2
# Robert Bosch 

# Written by Adam Stambler with tons of help from the ros docs
# Modifed by Sarah Osentoski to use IK w/ collision checking, to take in rotation information

"""
http://brown-ros-pkg.googlecode.com/svn/trunk/experimental/pr2_remotelab/ik_control/bin/arm.py
"""

import roslib; roslib.load_manifest('Pr2Debridement')
import actionlib
import rospy
import pr2_controllers_msgs.msg
#import geometry_msgs.msg
import kinematics_msgs.msg
import kinematics_msgs.srv
#import motion_planning_msgs.msg
#import motion_planning_msgs.srv
import trajectory_msgs.msg 
import std_msgs.msg
from math import *
import numpy as np
import tf
from geometry_msgs.msg import *
from Constants import *
from PR2CMClient import *
from Util import *

class CommandPoseClass:
    def __init__(self, armName):
        self.armName = armName
        self.running = False
        pubTopic = '/' + armName + '_' + ConstantsClass.ControllerName.JTCartesian + '/command_pose'
        self.pub = rospy.Publisher(pubTopic, PoseStamped)

    def startup(self):
        """
        Switches to CartesianPose controller.
        Store success in running.
        """
        self.running = PR2CMClient.change_arm_controller(self.armName, ConstantsClass.ControllerName.JTCartesian)
        
    def isRunning(self):
        return self.running
    
    def goToPose(self, desPose):
        """
        Commands the gripper to go to the desiredPose.
        
        desPose -- PoseStamped desired pose of gripper

        Returns True if successfully published
        """
        if not self.isRunning():
            return False

        cmd = PoseStamped()
        cmd.header.frame_id = desPose.header.frame_id

        cmd.pose = desPose.pose

        self.pub.publish(cmd)
        
        return True










def test():
    
    def stereoCallback(msg):
        test.desiredPose = PoseStamped()
        test.desiredPose.pose.orientation.w = .5**.5
        test.desiredPose.pose.orientation.x = 0
        test.desiredPose.pose.orientation.y = .5**.5
        test.desiredPose.pose.orientation.z = 0
        
        test.desiredPose.header = msg.header
        test.desiredPose.pose.position = msg.point
        test.desiredPose.pose.position.z += .15

    rospy.init_node('test_command_pose')
    listener = tf.TransformListener()

    test.desiredPose = None

    gripperPose = PoseStamped()
    gripperPose.pose.orientation.w = 1
    gripperPose.header.frame_id = ConstantsClass.ToolFrame.Left

    gripperFrame = ConstantsClass.ToolFrame.Left

    rospy.Subscriber(ConstantsClass.StereoName, PointStamped, stereoCallback)

    commandPose = CommandPoseClass(ConstantsClass.ArmName.Left)
    commandPose.startup()

    timeout = TimeoutClass(rospy.Duration(10))

    while commandPose.isRunning():
        rospy.loginfo('outer loop')
        if test.desiredPose != None:
            rospy.loginfo('inner loop')
            
            
            gripperPose.header.stamp = rospy.Time.now()

            commandPose.goToPose(test.desiredPose)
            rospy.loginfo('Going to pose')

            success = True
            timeout.start()
            while euclideanDistance(poseStampedToPointStamped(gripperPose), poseStampedToPointStamped(test.desiredPose),listener) > .01:
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.05)
            rospy.loginfo('At pose!')
            
        rospy.sleep(.1)

 



if __name__ == '__main__':
    test()
