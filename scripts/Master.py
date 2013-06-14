#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped
import tf

from CommandGripper import *
from CommandPose import *
from CommandTwist import *
from Constants import *
from ImageDetection import *

class MasterClass():
    def __init__(self, armName, imageDetector):
        """
        armName must be from ConstantsClass.ArmName
        imageDetector is instance of ImageDetectionClass
            - pass in so two MasterClasses (i.e. two arms) can share imageDetector
        """
        self.armName = armName
        
        if (armName == Constants.ArmName.Left):
            self.gripperName = ConstantsClass.GripperName.Left
        else:
            self.gripperName = ConstantsClass.GripperName.Right

        self.imageDetector = imageDetector
        self.commandGripper = CommandGripperClass(self.gripperName)
        #uncomment when implemented
        #self.commandPose = CommandPoseClass(self.armName)
        self.commandTwist = CommandTwistClass(self.armName)

    def run(self):
        while True:
            # can change rate
            rospy.sleep(.1)

            # find cancer point
            if self.imageDetector.hasFoundCancer():
                cancerPointStamped = self.imageDetector.getCancerPoint()
            else:
                continue

            # find gripper point
            if self.imageDetector.hasFoundGripper(self.gripperName):
                gripperPointStamped = self.imageDetector.getGripperPoint(self.gripperName)
            else:
                continue

            # self.commandPose if/else block needs to be added
            # to get near cancerPointStamped
                                
            # open gripper
            if not self.commandGripper.openGripper():
                continue

            # visual servo to get to cancer point

            # threshold is distance between gripper and cancer before declare success
            threshold = .05
            self.commandTwist.startup()
            while Util.euclideanDistance(gripperPointStamped, cancerPointStamped) > threshold:
                gripperPointStamped = self.imageDetector.getGripperPoint(self.gripperName)
                if not self.commandTwist.driveTowardPoint(gripperPointStamped, cancerPointStamped):
                    continue

            # close gripper (but not all the way)
            if not self.commandGripper.setGripper(.5):
                continue

            # self.commandPose if/else block needs to be added
            # to get back to receptacle
            
            # open gripper to drop off
            if not self.commandGripper.openGripper():
                continue


    
            #use self.commandPose(...) to get near cancerPointStamped
                

