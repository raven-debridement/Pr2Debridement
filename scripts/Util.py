#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion
import tf

def xyzAndFrameToPointStamped(x, y, z, frame_id):
    pStamped = PointStamped()
    
    pStamped.header.frame_id = frame_id
    pStamped.point.x = x
    pStamped.point.y = y
    pStamped.point.z = z

    return pStamped

def pointStampedToPoseStamped(pointStamped):
    """
    Quaternion of new poseStamped defaults to no rotation
    """
    poseStamped = PoseStamped()
    poseStamped.header = point.header
    poseStamped.pose.position.x = pointStamped.point.x
    poseStamped.pose.position.y = pointStamped.point.y
    poseStamped.pose.position.z = pointStamped.point.z

    poseStamped.pose.orienation.w = 1

    return poseStamped

def poseStampedToPointStamped(poseStamped):
    pointStamped = PointStamped()
    pointStamped.header = poseStamped.header
    pointStamped.point.x = poseStamped.pose.position.x
    pointStamped.point.y = poseStamped.pose.position.y
    pointStamped.point.z = poseStamped.pose.position.z

def reverseQuaternion(quat):
    newQuat = Quaternion()

    newQuat.w = quat.w
    newQuat.x = -quat.x
    newQuat.y = -quat.y
    newQuat.z = -quat.z

    return newQuat
