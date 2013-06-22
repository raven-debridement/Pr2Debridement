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
    """
    Contains the master pipeline for Pr2Debridement in the run method

    The general pipeline is as follows:
    - identify the receptacle, object, and gripper
    - open the gripper
    - move to a point near the object
    - servo to the object
    - close the gripper
    - move up with the object
    - move to the receptacle
    - open the gripper
    ... repeat

    If any of the steps fails, the loop goes back to the beginning

    Each pipeline staged is logged using rospy.loginfo
    """
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

        # initialize the three main control mechanisms
        # image detection, gripper control, and arm control
        self.imageDetector = imageDetector
        self.commandGripper = CommandGripperClass(self.gripperName)
        self.armControl = ArmControlClass(self.armName)

    def run(self):
        """
        Loops through the pipeline
        """
        while not rospy.is_shutdown():
            # can change rate
            rospy.sleep(.5)

            # delay between parts of the pipeline
            delay = .5
            # timeout class with 15 second timeout, can change
            timeout = TimeoutClass(15)

            # bounds, can change for each particular
            # pipeline section. keep loose for testing

            # translation bound in meters
            transBound = .06
            # rotation bound in radians
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

            self.armControl.goToArmPose(nearObjectPose, True, ConstantsClass.Request.goNear)

            success = True
            timeout.start()
            while not withinBounds(gripperPose, nearObjectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                
                if timeout.hasTimedOut():
                    success = False
                    break
                rospy.sleep(.5)

            if not success:
                continue
            
            




            rospy.sleep(delay)
            rospy.loginfo('Visual servoing to the object point')
            # visual servo to get to the object point
            transBound = .05
            rotBound = float("inf")

            success = True
            timeout.start()
            while not withinBounds(gripperPose, objectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
                objectPose = self.imageDetector.getObjectPose()
                # for servoing to work, make "goal" point be to the right more
                # might not work, just trying this
                objectPose.pose.position.y -= .03

                # servo to pose by one step, wait for user to press enter
                self.armControl.servoToPose(gripperPose, objectPose)
                raw_input()
                
                """
                # originally tried servoing by computing "differences" in poses
                # didn't work.
                reportedGripperPose = self.commandGripper.gripperPose()
                if reportedGripperPose == None:
                    continue
                gripperPoseDifference = subPoses(reportedGripperPose, gripperPose)
                if gripperPoseDifference == None:
                    continue
                desiredObjectPose = addPoses(objectPose, gripperPoseDifference)
                if desiredObjectPose == None:
                    continue
                self.armControl.goToArmPose(desiredObjectPose, False)
                """


                if timeout.hasTimedOut():
                    success = False
                    # commented out the break
                    # while stepping through the servoing
                    #break
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
            # move vertical with the object
            transBound = .03
            rotBound = .1
            vertObjectPose = PoseStamped(objectPose.header, objectPose.pose)
            vertObjectPose.pose.position.z += .1

            # currently have set to no planning, can change
            self.armControl.goToArmPose(vertObjectPose, False)

            success = True
            timeout.start()
            while not withinBounds(gripperPose, vertObjectPose, transBound, rotBound, self.listener):
                gripperPose = self.imageDetector.getGripperPose(self.gripperName)
            
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




def mainloop():
    """
    Gets an instance of the MasterClass
    for the left arm and executes the
    run loop
    """
    rospy.init_node('master_node')
    imageDetector = ImageDetectionClass()
    master = MasterClass(ConstantsClass.ArmName.Left, imageDetector)
    master.run()

if __name__ == '__main__':
    mainloop()
