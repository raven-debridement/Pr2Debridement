#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped
import tf

import PR2CMClient
import ConstantsClass

class CommandTwistClass():
    def __init__(self, armName, scale=.5):
        self.scale = scale
        self.armName = armName
        self.running = False
        self.listener = tf.Listener()
        pubTopic = '/' + armName + '_' + ConstantsClass.ControllerName.CartesianTwist + '/command'
        self.pub = rospy.Publisher(pubTopic, Twist)

        self.startup()

    def startup(self):
        """
        Switches to CartesianTwist controller.
        Store success in running.
        """
        self.running = PR2CMClient.change_arm_controller(self.armName,
                                                         ConstantsClass.ControllerName.CartesianTwist)
        
    def isRunning(self):
        return self.running

    
    def driveTowardPoint(self, currPoint, desPoint):
        """
        Given the current gripper point and the desired gripper point,
        this commands the gripper to go towards the desiredPoint.

        Note: only cares about translation of the gripper, not rotation

        currPoint -- PointStamped current position of gripper
        desPoint -- PointStamped desired position of gripper
        """
        if not self.isRunning():
            return

        currPoint, desPoint = self.convertToSameFrameAndTime(currPoint, desPoint)

        twistCommand = Twist()

        twistCommand.linear.x = (currPoint.x - desPoint.x) * self.scale
        twistCommand.linear.y = (currPoint.y - desPoint.y) * self.scale
        twistCommand.linear.z = (currPoint.z - desPoint.z) * self.scale

        twistCommand.angular.x = 0
        twistCommand.angular.y = 0
        twistCommand.angular.z = 0

        self.pub.publish(twistCommand)


    def convertToSameFrameAndTime(self, point0, point1):
        """
        Converts point 0 and point 1 to the same frame
        """
        
        p0frame, p1frame = point0.header.frame_id, point1.header.frame_id

        # need to be on same time so transformation will work
        commonTime = self.listener.getLastestCommonTime(p0frame, p1frame)
        point0.header.stamp = point1.header.stamp = commonTime

        return (point0, self.listener.transformPoint(p0frame, point1))

    
    def stop(self):
        """
        Stop velocity control immediately
        """
        
        if not self.isRunning():
            return

        self.pub.publish(Twist())
