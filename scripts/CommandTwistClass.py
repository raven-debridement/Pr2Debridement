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



    def __init__(self):
        rospy.init_node('l_arm_controller_node')

        self.desiredPoint = None
        self.gripperFrame = 'l_gripper_tool_frame'
        self.listener = tf.TransformListener()
        rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

        pub = rospy.Publisher('/l_arm_controller/command', Twist)
        desired_twist = Twist()
        desired_twist.linear.x = 0
        desired_twist.linear.y = 0
        desired_twist.linear.z = 0

        desired_twist.angular.x = 0
        desired_twist.angular.y = 0
        desired_twist.angular.z = 0

        while not rospy.is_shutdown():
            if self.desiredPoint != None:
                #(trans,rot) = self.listener.lookupTransform(self.desiredPoint.header.frame_id, self.gripperFrame, rospy.Time(0))

                desiredFrame = self.desiredPoint.header.frame_id
                commonTime = self.listener.getLatestCommonTime(desiredFrame, self.gripperFrame)

                gripperPoint = PointStamped()
                gripperPoint.header.frame_id = self.gripperFrame
                gripperPoint.header.stamp = commonTime
                
                trans_gripperPoint = self.listener.transformPoint(desiredFrame, gripperPoint)

                self.desiredPoint.header.stamp = commonTime
                trans_desiredPoint = self.listener.transformPoint(self.gripperFrame,self.desiredPoint)
                scale = 0.5

                desired_twist.linear.x = (self.desiredPoint.point.x - trans_gripperPoint.point.x) * scale
                desired_twist.linear.y = (self.desiredPoint.point.y - trans_gripperPoint.point.y) * scale
                desired_twist.linear.z = (self.desiredPoint.point.z - trans_gripperPoint.point.z) * scale


                #desired_twist.linear.x = trans_desiredPoint.point.x * scale
                #desired_twist.linear.y = trans_desiredPoint.point.y * scale
                #desired_twist.linear.z = trans_desiredPoint.point.z * scale
                
                #desired_twist.linear.x = (trans_desiredPoint.point.x-trans[0]) * scale
                #desired_twist.linear.y = (trans_desiredPoint.point.y-trans[1]) * scale
                #desired_twist.linear.z = (trans_desiredPoint.point.z-trans[2]) * scale
                
                rospy.loginfo('(' + str(desired_twist.linear.x) + ',' + str(desired_twist.linear.y) + ',' + str(desired_twist.linear.z) + ')')
                pub.publish(desired_twist)
            rospy.sleep(1.0)

    def stereoCallback(self, msg):
        self.desiredPoint = msg

        """
        trans_msg = self.listener.transformPoint('l_wrist_roll_link', msg)
        self.x = trans_msg.point.x
        self.y = trans_msg.point.y
        self.z = trans_msg.point.z
        """
        #rospy.loginfo('(' + self.x + ',' + self.y + ',' + self.z + ')')

if __name__ == '__main__':
    CommandTwistClass()
