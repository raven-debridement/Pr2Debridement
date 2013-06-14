#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
import tf
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

import ConstantsClass

class CommandGripperClass():
    def __init__(self, gripperName):
        """
        gripperName must be from ConstantsClass.GripperName
        """
        # max gripper range spec is 90mm
        self.maxRange = .09
        # no effort limit (may change)
        self.efforLimit = -1

        self.client = actionlib.SimpleActionClient(gripperName + '_controller/gripper_action', Pr2GripperCommandAction)

    def openGripper(self):
        return self.setGripper(0)

    def closeGripper(self):
        return self.setGripper(1)

    def setGripper(self, openPercentage):
        position = openPercentage * self.maxRange

        self.client.wait_for_server()
        self.client.send_goal(Pr2GripperCommand(position, self.effortLimit))
        self.client.wait_for_result()

        result = client.get_result()
        
        return (client.get_state() == GoalStatus.SUCCEEDED)
