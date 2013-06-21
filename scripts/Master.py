#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import PointStamped, Pose
import tf

from CommandGripper import CommandGripperClass
from ArmControl import ArmControlClass
from Constants import ConstantsClass
from ImageDetection import ImageDetectionClass
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
        while not rospy.is_shutdown():
            # can change rate
            rospy.sleep(.5)

            # delay between parts
            delay = .5
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
            objectPose = self.imageDetector.getObjectPose()
            if objectPose == None:
                continue
            objectPoint = poseStampedToPointStamped(objectPose)
            

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
            nearObjectPose = PoseStamped(objectPose.header, objectPose.pose)
            nearObjectPose.pose.position.y += .1 # 10 cm to left
            nearObjectPose.pose.position.z += .1 # and a little above, for safety
            # Add noise below ######

            ########################
            self.armControl.goToArmPose(nearObjectPose, True, ConstantsClass.Request.goNear)

            success = True
            timeout.start()
            while not withinBounds(gripperPose, nearObjectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                """
                objectPose = self.imageDetector.getObjectPose()
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
            rotBound = .1

            success = True
            timeout.start()
            while not withinBounds(gripperPose, objectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                objectPose = self.imageDetector.getObjectPose()
            
                ############# need to eventually take gripperPose into account
                reportedGripperPose = self.commandGripper.gripperPose()
                gripperPoseDifference = subPoses(reportedGripperPose, gripperPose)
                desiredObjectPose = addPoses(objectPose, gripperPoseDifference)
                
                #print('objectPose')
                #print(objectPose)
                #print('desiredObjectPose')
                #print(desiredObjectPose)
                ########################################################
                self.armControl.goToArmPose(desiredObjectPose, False)

                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.1)

            if not success:
                continue



            rospy.sleep(delay)
            rospy.loginfo('Closing the gripper')
            # close gripper (consider not all the way)
            if not self.commandGripper.setGripper(.5):
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
            
                self.armControl.goToArmPose(vertObjectPose, False)

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
            self.armControl.goToArmPose(receptaclePose, True, ConstantsClass.Request.goReceptacle)

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
            #self.imageDetector.removeObjectPoint()



def test():
    rospy.init_node('master_node')
    imageDetector = ImageDetectionClass()
    master = MasterClass(ConstantsClass.ArmName.Left, imageDetector)
    master.run()

if __name__ == '__main__':
    test()
