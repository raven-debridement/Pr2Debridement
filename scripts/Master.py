#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped
import tf
import random

from CommandGripper import *
from ArmControl import *
from Constants import *
from ImageDetection import *
from Util import *

class MasterClass():
    def __init__(self, armName, imageDetector):
        """
        armName must be from ConstantsClass.ArmName
        imageDetector is instance of ImageDetectionClass
            - pass in so two MasterClasses (i.e. two arms) can share imageDetector
        """
        self.armName = armName
        
        if (armName == ConstantsClass.ArmName.Left):
            self.gripperName = ConstantsClass.GripperName.Left
            self.toolframe = ConstantsClass.ToolFrame.Left
        else:
            self.gripperName = ConstantsClass.GripperName.Right
            self.toolframe = ConstantsClass.ToolFrame.Right

        self.listener = tf.TransformListener()

        self.imageDetector = imageDetector
        self.commandGripper = CommandGripperClass(self.gripperName)
        self.armControl = ArmControlClass(self.armName)

    def run(self):
        while True:
            # can change rate
            rospy.sleep(.5)

            # delay between parts
            delay = 1
            # timeout class with 15 second timeout
            timeout = TimeoutClass(15)
            # translation bound
            transBound = .06
            # rotation bound
            rotBound = float("inf")
            
            rospy.loginfo('Searching for the receptacle')
            if not self.imageDetector.hasFoundReceptacle():
                continue

            rospy.loginfo('Searching for object point')
            # find object point and pose
            if self.imageDetector.hasFoundObject():
                objectPose = self.imageDetector.getObjectPose()
                objectPoint = poseStampedToPointStamped(objectPose)
            else:
                continue


            rospy.loginfo('Searching for ' + self.gripperName)
            # find gripper point
            if self.imageDetector.hasFoundGripper(self.gripperName):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                gripperPoint = poseStampedToPointStamped(gripperPose)
            else:
                continue

            
            rospy.loginfo('Opening the gripper')
            # open gripper
            if not self.commandGripper.openGripper():
                continue
            
            
            rospy.loginfo('Moving close to the object point')
            # Add noise below ######
            nearObjectPose = PoseStamped(objectPose.header, objectPose.pose)
            nearObjectPose.pose.position.y += .1 # 10 cm to left
            nearObjectPose.pose.position.z += .1 # and a little above, for safety
            ########################
            self.armControl.planAndGoToArmPose(nearObjectPose, ConstantsClass.Request.goNear, self.listener)

            success = True
            timeout.start()
            while not withinBounds(gripperPose, nearObjectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                """
                objectPose = self.imageDetector.getObjectPose()
                self.armControl.planAndGoToArmPose(objectPose, self.listener)
                """
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.5)

            if not success:
                continue

            
            rospy.sleep(delay)
            rospy.loginfo('Visual servoing to the object point')
            # visual servo to get to the object point
            transBound = .01
            rotBound = .01

            success = True
            timeout.start()
            while not withinBounds(gripperPose, objectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                objectPose = self.imageDetector.getObjectPose()
            
                # need to eventually take gripperPose into account
                self.armControl.goToArmPose(objectPose, self.listener)

                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)

            if not success:
                continue



            rospy.sleep(delay)
            rospy.loginfo('Closing the gripper')
            # close gripper (consider not all the way)
            if not self.commandGripper.closeGripper():
                continue

            

            rospy.sleep(delay)
            rospy.loginfo('Moving vertical with the object')
            # visual servo to get to the object point
            transBound = .01
            rotBound = .01
            vertObjectPose = PoseStamped(objectPose.header, objectPose.pose)
            vertObjectPose.pose.position.z += .1

            success = True
            timeout.start()
            while not withinBounds(gripperPose, vertObjectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
            
                # need to eventually take gripperPose into account
                self.armControl.goToArmPose(vertObjectPose, self.listener)

                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)

            if not success:
                continue


            rospy.sleep(delay)
            rospy.loginfo('Moving to the receptacle')
            transBound = .07
            rotBound = float("inf")
            # move to receptacle
            receptaclePose = self.imageDetector.getReceptaclePose()
            self.armControl.planAndGoToArmPose(receptaclePose, ConstantsClass.Request.goReceptacle, self.listener)

            success = True
            timeout.start()
            while not withinBounds(gripperPose, receptaclePose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                receptaclePose = self.imageDetector.getReceptaclePose()
                # CURRENTLY NOT TAKING INTO ACCOUNT GRIPPER POSE
                #self.armControl.planAndGoToArmPose(receptaclePose)
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.5)

            if not success:
                continue

            rospy.loginfo('Opening the gripper to drop in the receptacle')
            # open gripper to drop off
            if not self.commandGripper.openGripper():
                continue

            # temporary
            # when image segmentation done, object will automatically be
            # removed b/c it will be in the receptacle
            self.imageDetector.removeFirstObjectPoint()



def test():
    rospy.init_node('master_node')
    imageDetector = ImageDetectionClass()
    master = MasterClass(ConstantsClass.ArmName.Left, imageDetector)
    master.run()

if __name__ == '__main__':
    test()
