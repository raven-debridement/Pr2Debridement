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

            if self.imageDetector.hasFoundCancer():
                cancerPointStamped = self.imageDetector.getCancerPoint()
            
            
    
            #use self.commandPose(...) to get near cancerPointStamped
                

            # can change rate
            rospy.sleep(.1)
