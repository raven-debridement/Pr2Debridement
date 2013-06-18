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

            # timeout class with 60 second timeout
            timeout = TimeoutClass(60)
            # translation bound
            transBound = .02
            # rotation bound
            rotBound = .01
            
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

            """
            rospy.loginfo('Opening the gripper')
            # open gripper
            if not self.commandGripper.openGripper():
                continue
            """
            
            rospy.loginfo('Servoing to the object point')
            # Add noise below ######

            ########################
            """
            preObjectPose = PoseStamped(objectPose)
            print(objectPose)
            commonTime = self.listener.getLatestCommonTime(self.toolframe, objectPose.header.frame_id)
            objectPose.header.stamp = commonTime
            objectPose = self.listener.transformPose(self.toolframe, objectPose)
            print(objectPose)
            """
            # CURRENTLY NOT TAKING INTO ACCOUNT GRIPPER POSE
            #self.armControl.planAndGoToArmPose(objectPose)
            self.armControl.goToArmPose(objectPose)

            success = True
            timeout.start()
            while not withinBounds(gripperPose, objectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                objectPose = self.imageDetector.getObjectPose()

                """
                commonTime = self.listener.getLatestCommonTime(self.toolframe, objectPose.header.frame_id)
                objectPose.header.stamp = commonTime
                objectPose = self.listener.transformPose(self.toolframe, objectPose)
                print(objectPose)

                # CURRENTLY NOT TAKING INTO ACCOUNT GRIPPER POSE
                self.armControl.planAndGoToArmPose(objectPose)
                """
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.5)

            if not success:
                continue

            """
            rospy.loginfo('Moving to a point near the object point')
            # go near object point
            threshold = .05
            self.commandPose.startup()
            nearObjectPose = reversePoseStamped(objectPose)
            nearObjectPose.pose.position.z += .2
            # Add noise! ################
            #nearObjectPose.pose.position.x += .05#random.uniform(-.05,.05)
            #nearObjectPose.pose.position.y += random.uniform(-.05,.05)
            #nearObjectPose.pose.position.z += random.uniform(-.05,.05)
            #############################
            nearObjectPoint = poseStampedToPointStamped(nearObjectPose)
            self.commandPose.goToPose(nearObjectPose)

            success = True
            timeout.start()
            while euclideanDistance(gripperPoint, nearObjectPoint,self.listener) > threshold:
                #rospy.loginfo(euclideanDistance(gripperPoint, nearObjectPoint,self.listener))
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)
                
            if not success:
                continue
            """


            """
            rospy.loginfo('Visual servoing to the object point')
            # visual servo to get to object point
            # threshold is distance between gripper and object before declare success
            threshold = .01
            self.commandTwist.startup()

            success = True
            isCentered = False
            timeout.start()
            while euclideanDistance(gripperPoint, objectPoint, self.listener) > threshold:
                #rospy.loginfo(euclideanDistance(gripperPoint, objectPoint, self.listener))
                if not isCentered and euclideanDistance(gripperPoint, objectPoint, self.listener, zPlane=isCentered) < threshold:
                    isCentered = True
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                self.commandTwist.driveTowardPoint(gripperPoint, objectPoint, zPlane=isCentered)
                if timeout.hasTimedOut():
                    success = False
                    break
                #determines the rate
                rospy.sleep(.05)

            if success:
                self.commandTwist.stop()
            else:
                continue
            """

            rospy.sleep(.5)
            rospy.loginfo('Closing the gripper')
            # close gripper (consider not all the way)
            if not self.commandGripper.closeGripper():
                continue

            
            """
            rospy.loginfo('Moving vertical with object')
            # move straight up from the table
            # move to nearObjectPoint
            self.commandPose.startup()
            self.commandPose.goToPose(nearObjectPose)

            success = True
            timeout.start()
            while euclideanDistance(gripperPoint, nearObjectPoint,self.listener) > threshold:
                rospy.loginfo(euclideanDistance(gripperPoint, nearObjectPoint,self.listener))
                gripperPoint = self.imageDetector.getGripperPoint(self.gripperName)
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)
                
            if not success:
                continue
            """

            rospy.loginfo('Moving to the receptacle')
            # move to receptacle
            receptaclePose = self.imageDetector.getReceptaclePose()
            
            success = True
            timeout.start()
            while not withinBounds(gripperPose, receptaclePose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                receptaclePose = self.imageDetector.getReceptaclePose()
                # CURRENTLY NOT TAKING INTO ACCOUNT GRIPPER POSE
                self.armControl.planAndGoToArmPose(receptaclePose)
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.5)

            if not success:
                continue

            """
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
            """                              

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
