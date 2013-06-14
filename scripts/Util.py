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

def posAndOrientAndFrameToPoseStamped(x_trans, y_trans, z_trans, w, x_rot, y_rot, z_rot, frame_id):
    pStamped = PoseStamped()
    
    pStamped.header.frame_id = frame_id
    pStamped.pose.position.x = x_trans
    pStamped.pose.position.y = y_trans
    pStamped.pose.position.z = z_tranas

    pStamped.pose.orienation.w = w
    pStamped.pose.orienation.x = x_rot
    pStamped.pose.orienation.y = y_rot
    pStamped.pose.orienation.z = z_rot

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

    return pointStamped

def reverseQuaternion(quat):
    newQuat = Quaternion()

    newQuat.w = quat.w
    newQuat.x = -quat.x
    newQuat.y = -quat.y
    newQuat.z = -quat.z

    return newQuat

def convertToSameFrameAndTime(ps0, ps1, listener):
    """
    Converts point 0 and point 1 to the same frame
    """
    
    ps0frame, ps1frame = ps0.header.frame_id, ps1.header.frame_id

    # need to be on same time so transformation will work
    listener.waitForTransform(ps0frame, ps1frame, rospy.Time.now(), rospy.Duration(10.0))
    commonTime = listener.getLatestCommonTime(ps0frame, ps1frame)
    ps0.header.stamp = ps1.header.stamp = commonTime
    
    return (ps0, listener.transformPoint(ps0frame, ps1))


def euclidianDistance(ps0, ps1):
    """
    Returns euclidean distance between two PointStamped
    """
    # must be in same reference frame
    ps0, ps1 = convertToSameFrameAndTime(ps0, ps1, tf.TransformListener())

    x0, y0, z0 = ps0.point.x, ps0.point.y, ps0.point.z
    x1, y1, z1 = ps1.point.x, ps1.point.y, ps1.point.z

    return ((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)**.5
