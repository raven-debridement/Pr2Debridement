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
from CommandPose import *
from CommandTwist import *
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
        else:
            self.gripperName = ConstantsClass.GripperName.Right

        self.listener = tf.TransformListener()

        self.imageDetector = imageDetector
        self.commandGripper = CommandGripperClass(self.gripperName)
        self.commandPose = CommandPoseClass(self.armName)
        self.commandTwist = CommandTwistClass(self.armName)

    def run(self):
        while True:
            # can change rate
            rospy.sleep(.5)

            # timeout class with 15 second timeout
            timeout = TimeoutClass(100)

            rospy.loginfo('Searching for cancer point')
            # find cancer point and pose
            if self.imageDetector.hasFoundCancer():
                cancerPose = self.imageDetector.getCancerPose()
                cancerPoint = poseStampedToPointStamped(cancerPose)
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
            nearCancerPose = reversePoseStamped(cancerPose)
            nearCancerPose.pose.position.z += .2
            # Add noise! ################
            #nearCancerPose.pose.position.x += .05#random.uniform(-.05,.05)
            #nearCancerPose.pose.position.y += random.uniform(-.05,.05)
            #nearCancerPose.pose.position.z += random.uniform(-.05,.05)
            #############################
            nearCancerPoint = poseStampedToPointStamped(nearCancerPose)
            self.commandPose.goToPose(nearCancerPose)

            success = True
            timeout.start()
            while euclideanDistance(gripperPoint, nearCancerPoint,self.listener) > threshold:
                #rospy.loginfo(euclideanDistance(gripperPoint, nearCancerPoint,self.listener))
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)
                
            if not success:
                continue

            rospy.sleep(.5)
            rospy.loginfo('Opening the gripper')
            # open gripper
            if not self.commandGripper.openGripper():
                continue


            rospy.loginfo('Visual servoing to the cancer point')
            # visual servo to get to cancer point
            # threshold is distance between gripper and cancer before declare success
            threshold = .01
            self.commandTwist.startup()

            success = True
            isCentered = False
            timeout.start()
            while euclideanDistance(gripperPoint, cancerPoint, self.listener) > threshold:
                #rospy.loginfo(euclideanDistance(gripperPoint, cancerPoint, self.listener))
                if not isCentered and euclideanDistance(gripperPoint, cancerPoint, self.listener, zPlane=isCentered) < threshold:
                    isCentered = True
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                self.commandTwist.driveTowardPoint(gripperPoint, cancerPoint, zPlane=isCentered)
                if timeout.hasTimedOut():
                    success = False
                    break
                #determines the rate
                rospy.sleep(.05)

            if success:
                self.commandTwist.stop()
            else:
                continue

            rospy.sleep(.5)
            rospy.loginfo('Closing the gripper')
            # close gripper (consider not all the way)
            if not self.commandGripper.closeGripper():
                continue

            
            rospy.loginfo('Moving vertical with cancer')
            # move straight up from the table
            # move to nearCancerPoint
            self.commandPose.startup()
            self.commandPose.goToPose(nearCancerPose)

            success = True
            timeout.start()
            while euclideanDistance(gripperPoint, nearCancerPoint,self.listener) > threshold:
                rospy.loginfo(euclideanDistance(gripperPoint, nearCancerPoint,self.listener))
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)
                
            if not success:
                continue





            rospy.loginfo('Moving to the receptacle')
            # self.commandPose if/else block needs to be added
            # to get back to receptacle
            if self.imageDetector.hasFoundReceptacle():
                receptaclePoint = self.imageDetector.getReceptaclePoint()
                
                receptaclePose = reversePoseStamped(self.imageDetector.getReceptaclePose())
                receptaclePose.pose.position.z += .1
                
                threshold = .15
                self.commandPose.startup()
                self.commandPose.goToPose(receptaclePose)

                success = True
                timeout.start()
                while euclideanDistance(gripperPoint, receptaclePoint, self.listener) > threshold:
                    rospy.loginfo(euclideanDistance(gripperPoint, receptaclePoint, self.listener))
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



def test():
    rospy.init_node('master_node')
    imageDetector = ImageDetectionClass()
    master = MasterClass(ConstantsClass.ArmName.Left, imageDetector)
    master.run()

if __name__ == '__main__':
    test()
