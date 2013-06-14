#! /usr/bin/env python/

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
import geometry_msgs.msg
import kinematics_msgs.msg
import kinematics_msgs.srv
#import motion_planning_msgs.msg
#import motion_planning_msgs.srv
import trajectory_msgs.msg 
import std_msgs.msg
from math import *
import numpy as np
import tf


class CommandPoseClass:
    def __init__(self, armName):
        self.armName = armName
        self.running = False
        self.listener = tf.TransformListener()
        pubTopic = '/' + armName + '_' + ConstantsClass.ControllerName.CartesianPose + '/command_pose'
        self.pub = rospy.Publisher(pubTopic, PoseStamped)

        self.startup()

    def startup(self):
        """
        Switches to CartesianPose controller.
        Store success in running.
        """
        self.running = PR2CMClient.change_arm_controller(self.armName, ConstantsClass.ControllerName.CartesianPose)
        
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

        cmd = PoseCommand()
        cmd.header.frame_id = desPoint.header.frame_id

        cmd.pose = desPose.pose

        self.pub.publish(cmd)
        
        return True

    def stop(self):
        """
        Stop controller immediately -- warning, this may cause arm to drop
        """
        
        if not self.isRunning():
            return

        self.running = PR2CMClient.stop_arm_controller(self.armName, ConstantsClass.ControllerName.CartesianPose)
