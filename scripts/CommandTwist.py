#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import *
import tf

from PR2CMClient import *

from Constants import *
import Util

class CommandTwistClass():
    def __init__(self, armName, listener=None, scale=.5):
        """
        Must call startup before calling driveTowardPoint
        """
        self.scale = scale
        self.armName = armName
        self.running = False
        if listener == None:
            self.listener = tf.TransformListener()
        else:
            self.listener = listener
        pubTopic = '/' + armName + '_' + ConstantsClass.ControllerName.CartesianTwist + '/command'
        self.pub = rospy.Publisher(pubTopic, Twist)


    def startup(self):
        """
        Switches to CartesianTwist controller.
        Store success in running.
        """
        self.running = PR2CMClient.change_arm_controller(self.armName, ConstantsClass.ControllerName.CartesianTwist)
        
    def isRunning(self):
        return self.running

    
    def driveTowardPoint(self, currPoint, desPoint, xPlane=True,yPlane=True,zPlane=True):
        """
        Given the current gripper point and the desired gripper point,
        this commands the gripper to go towards the desiredPoint.

        Note: only cares about translation of the gripper, not rotation

        currPoint -- PointStamped current position of gripper
        desPoint -- PointStamped desired position of gripper
        """
        if not self.isRunning():
            return

        try:
            currPoint, desPoint = Util.convertToSameFrameAndTime(currPoint, desPoint, self.listener)
        except tf.Exception:
            return

        twistCommand = Twist()

        if xPlane:
            twistCommand.linear.x = (desPoint.point.x - currPoint.point.x) * self.scale
        if yPlane:
            twistCommand.linear.y = (desPoint.point.y - currPoint.point.y) * self.scale
        if zPlane:
            twistCommand.linear.z = (desPoint.point.z - currPoint.point.z) * self.scale

        """
        twistCommand.angular.x = 0
        twistCommand.angular.y = 0
        twistCommand.angular.z = 0
        """

        self.pub.publish(twistCommand)
    
    def stop(self):
        """
        Stop velocity control immediately
        """
        
        if not self.isRunning():
            return

        self.pub.publish(Twist())






def test():

    def stereoCallback(msg):
        test.desiredPoint = msg

        """
        trans_msg = self.listener.transformPoint('l_wrist_roll_link', msg)
        self.x = trans_msg.point.x
        self.y = trans_msg.point.y
        self.z = trans_msg.point.z
        """
        #rospy.loginfo('(' + self.x + ',' + self.y + ',' + self.z + ')')

    rospy.init_node('test_command_twist')
    test.desiredPoint = None
    gripperFrame = ConstantsClass.ToolFrame.Left

    rospy.Subscriber(ConstantsClass.StereoName, PointStamped, stereoCallback)

    ctwist = CommandTwistClass(ConstantsClass.ArmName.Left)
    ctwist.startup()

    pub = rospy.Publisher('/l_arm_twist_controller/command', Twist)

    dt = Twist()
    dt.linear.x = 0
    dt.linear.y = 0
    dt.linear.z = -.1
    # negative z is towards table

    dt.angular.x = 0
    dt.angular.y = 0
    dt.angular.z = 0

    while True:
        pub.publish(dt)
        rospy.sleep(.1)
    
    """
    pub = rospy.Publisher('/l_arm_controller/command', Twist)
    desired_twist = Twist()
    desired_twist.linear.x = 0
    desired_twist.linear.y = 0
    desired_twist.linear.z = 0

    desired_twist.angular.x = 0
    desired_twist.angular.y = 0
    desired_twist.angular.z = 0
    """

    while ctwist.isRunning():
        rospy.loginfo('outer loop')
        if test.desiredPoint != None:
            rospy.loginfo('inner loop')
            #desiredFrame = self.desiredPoint.header.frame_id
            #commonTime = self.listener.getLatestCommonTime(desiredFrame, self.gripperFrame)

            gripperPoint = PointStamped()
            gripperPoint.header.frame_id = ConstantsClass.ToolFrame.Left
            #gripperPoint.header.stamp = commonTime
            
            #trans_gripperPoint = self.listener.transformPoint(desiredFrame, gripperPoint)

            #self.desiredPoint.header.stamp = commonTime
            #trans_desiredPoint = self.listener.transformPoint(self.gripperFrame,self.desiredPoint)
            #scale = 0.5

            #desired_twist.linear.x = (self.desiredPoint.point.x - trans_gripperPoint.point.x) * scale
            #desired_twist.linear.y = (self.desiredPoint.point.y - trans_gripperPoint.point.y) * scale
            #desired_twist.linear.z = (self.desiredPoint.point.z - trans_gripperPoint.point.z) * scale

            
            #rospy.loginfo('(' + str(desired_twist.linear.x) + ',' + str(desired_twist.linear.y) + ',' + str(desired_twist.linear.z) + ')')
            #pub.publish(desired_twist)
            ctwist.driveTowardPoint(gripperPoint, test.desiredPoint)

        rospy.sleep(1.0)

 



if __name__ == '__main__':
    test()
