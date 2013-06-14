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
    Quaternion of new poseStamped is left as default
    """
    poseStamped = PoseStamped()
    poseStamped.header = point.header
    poseStamped.pose.point.x = pointStamped.point.x
    poseStamped.pose.point.y = pointStamped.point.y
    poseStamped.pose.point.z = pointStamped.point.z

    return poseStamped

def poseStampedToPointStamped(poseStamped):
    pointStamped = PointStamped()
    pointStamped.header = poseStamped.header
    pointStamped.point.x = poseStamped.pose.point.x
    pointStamped.point.y = poseStamped.pose.point.y
    pointStamped.point.z = poseStamped.pose.point.z

def reverseQuaternion(quat):
    newQuat = Quaternion()

    newQuat.w = quat.w
    newQuat.x = -quat.x
    newQuat.y = -quat.y
    newQuat.z = -quat.z

    return newQuat
