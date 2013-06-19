#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal, Pr2GripperCommand

from Constants import ConstantsClass

class CommandGripperClass():
    def __init__(self, gripperName):
        """
        gripperName must be from ConstantsClass.GripperName
        """
        # max gripper range spec is 90mm
        self.maxRange = .09
        # no effort limit (may change)
        self.effortLimit = -1
        # distance from center of gripper to tip
        self.gripperLength = .05

        self.client = actionlib.SimpleActionClient(gripperName + '_controller/gripper_action', Pr2GripperCommandAction)


    def openGripper(self):
        return self.setGripper(1)

    def closeGripper(self):
        return self.setGripper(0)

    def setGripper(self, openPercentage):
        position = openPercentage * self.maxRange

        self.client.wait_for_server()
        self.client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position, self.effortLimit)))
        
        #temp fix
        #rospy.sleep(2)
        #return True

        self.client.wait_for_result()
        return True

        # below not guaranteed to work for grasping
        # result = self.client.get_result()        
        # return (result.reached_goal) or (result.stalled)

    def gripperLength(self):
        return self.gripperLength



def test():

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
    

#for testing
if __name__ == '__main__':
    rospy.init_node('gripper_node')
    #cgl = CommandGripperClass(ConstantsClass.GripperName.Left)
    #cgl.openGripper()
    test()
