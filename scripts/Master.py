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
        self.commandPose = CommandPoseClass(self.armName)
        self.commandTwist = CommandTwistClass(self.armName)

    def run(self):
        while True:
            # can change rate
            rospy.sleep(.1)

            # timeout class with 15 second timeout
            timeout = TimeoutClass(rospy.Duration(15))

            rospy.loginfo('Searching for cancer point')
            # find cancer point
            if self.imageDetector.hasFoundCancer():
                cancerPoint = self.imageDetector.getCancerPoint()
            else:
                continue


            rospy.loginfo('Searching for ' + self.gripperName)
            # find gripper point
            if self.imageDetector.hasFoundGripper(self.gripperName):
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
            else:
                continue


            rospy.loginfo('Moving to a point near the cancer point')
            # go near cancer point
            threshold = .05
            self.commandPose.startup()
            #### STUB: REPLACE WITH CORRECT ###
            nearCancerPose = PoseStamped()
            ###################################
            self.commandPose.goToPose(nearCancerPose)

            success = True
            timeout.start()
            while euclideanDistance(gripperPoint, nearCancerPoint) > threshold:
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)
                
            if not success:
                continue


            rospy.loginfo('Opening the gripper')
            # open gripper
            if not self.commandGripperC.openGripper():
                continue


            rospy.loginfo('Visual servoing to the cancer point')
            # visual servo to get to cancer point
            # threshold is distance between gripper and cancer before declare success
            threshold = .05
            self.commandTwist.startup()

            success = True
            timeout.start()
            while euclideanDistance(gripperPoint, cancerPoint) > threshold:
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                if (not self.commandTwist.driveTowardPoint(gripperPoint, cancerPoint)) or (timeout.hasTimedOut()):
                    success = False
                    break

            if not success:
                continue


            rospy.loginfo('Closing the gripper')
            # close gripper (but not all the way)
            if not self.commandGripper.setGripper(.5):
                continue


            rospy.loginfo('Moving to the receptacle')
            # self.commandPose if/else block needs to be added
            # to get back to receptacle
            if self.imageDetector.hasFoundReceptacle():
                receptaclePoint = self.imageDetector.getReceptaclePoint()
                
                threshold = .05
                self.commandPose.startup()
                self.commandPose.goToPose(receptaclePose)

                success = True
                timeout.start()
                while euclideanDistance(gripperPoint, receptaclePoint) > threshold:
                    gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                    if timeout.hasTimedOut():
                        success = False
                        break
                    rospy.sleep(.1)
            else:
                continue

            if not success:
                continue
                                             

            rospy.loginfo('Opening the gripper to drop in the receptacle')
            # open gripper to drop off
            if not self.commandGripper.openGripper():
                continue


