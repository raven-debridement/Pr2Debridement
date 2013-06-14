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

from ConstantsClass import *

class CommandGripperClass():
    def __init__(self, gripperName):
        """
        gripperName must be from ConstantsClass.GripperName
        """
        # max gripper range spec is 90mm
        self.maxRange = .09
        # no effort limit (may change)
        self.effortLimit = -1

        self.client = actionlib.SimpleActionClient(gripperName + '_controller/gripper_action', Pr2GripperCommandAction)

    def openGripper(self):
        return self.setGripper(1)

    def closeGripper(self):
        return self.setGripper(0)

    def setGripper(self, openPercentage):
        position = openPercentage * self.maxRange

        self.client.wait_for_server()
        self.client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position, self.effortLimit)))
        self.client.wait_for_result()

        result = self.client.get_result()
        
        """
        if result.stalled:
            print('stalled!')
        elif result.reached_goal:
            print('reached goal!')
        else:
            print('failed!')
        """

        return (self.client.get_state() == GoalStatus.SUCCEEDED)



#for testing
if __name__ == '__main__':
    rospy.init_node('test_gripper_class')

    success = True
    timeDelay = 3

    cgl = CommandGripperClass(ConstantsClass.GripperName.Left)    
    success &= cgl.openGripper()
    rospy.sleep(timeDelay)
    success &= cgl.closeGripper()
    rospy.sleep(timeDelay)

    success &= cgl.setGripper(.5)
    rospy.sleep(timeDelay)
    success &= cgl.setGripper(.25)
    rospy.sleep(timeDelay)
    success &= cgl.setGripper(.75)
    rospy.sleep(timeDelay)

    success &= cgl.closeGripper()

    
    cgr = CommandGripperClass(ConstantsClass.GripperName.Right)    
    success &= cgr.openGripper()
    rospy.sleep(timeDelay)
    success &= cgr.closeGripper()
    rospy.sleep(timeDelay)

    success &= cgr.setGripper(.5)
    rospy.sleep(timeDelay)
    success &= cgr.setGripper(.25)
    rospy.sleep(timeDelay)
    success &= cgr.setGripper(.75)
    rospy.sleep(timeDelay)

    success &= cgr.closeGripper()


    if success:
        print('CommandGripperClass passed tests')
    else:
        print('CommandGripperClass failed!!!')
    
