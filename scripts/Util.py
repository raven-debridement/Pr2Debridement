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


def pointStampedToPoseStamped(pointStamped, orientation=None):
    """
    Quaternion of new poseStamped defaults to no rotation
    """
    poseStamped = PoseStamped()
    poseStamped.header = pointStamped.header
    poseStamped.pose.position.x = pointStamped.point.x
    poseStamped.pose.position.y = pointStamped.point.y
    poseStamped.pose.position.z = pointStamped.point.z

    if orientation == None:
        poseStamped.pose.orientation.w = 1
    else:
        poseStamped.pose.orientation = orientation

    return poseStamped

def poseStampedToPointStamped(poseStamped):
    pointStamped = PointStamped()
    pointStamped.header = poseStamped.header
    pointStamped.point.x = poseStamped.pose.position.x
    pointStamped.point.y = poseStamped.pose.position.y
    pointStamped.point.z = poseStamped.pose.position.z

    return pointStamped

def makeQuaternion(w, x, y, z):
    newQuat = Quaternion()
    
    newQuat.w = w
    newQuat.x = x
    newQuat.y = y
    newQuat.z = z

    return newQuat
    
def reversePoseStamped(poseStamped):
    poseStamped.pose.orientation = reverseQuaternion(poseStamped.pose.orientation)
    return poseStamped

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
    # sometimes exceptions are thrown, DEAL WITH THIS
    commonTime = listener.getLatestCommonTime(ps0frame, ps1frame)
    ps0.header.stamp = ps1.header.stamp = commonTime

    return (listener.transformPoint(ps1frame, ps0), ps1)
    #return (ps0, listener.transformPoint(ps0frame, ps1))


def euclideanDistance(ps0, ps1, listener=None):
    """
    Returns euclidean distance between two PointStamped
    """
    # must be in same reference frame
    if listener != None:
        ps0, ps1 = convertToSameFrameAndTime(ps0, ps1, listener)

    x0, y0, z0 = ps0.point.x, ps0.point.y, ps0.point.z
    x1, y1, z1 = ps1.point.x, ps1.point.y, ps1.point.z

    return ((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)**.5

def timeoutFunc(loopTest, update, timeout, sleepTime=.1):
    """
    Higher-order function

    loopTest - must take one argument, which is the output of update
    update - must take no arguments
    timeout - time until failure

    Output - True if loopTest(update()) returns False once
    """
    success = True
    stopTime = rospy.Time.now() + timeout
    
    while loopTest(update()):
        if rospy.Time.now() > stopTime:
            return False
        rospy.sleep(sleepTime)
    
    return success

class TimeoutClass():
    def __init__(self, timeoutTime):
        self.timeoutTime = timeoutTime

    def start(self):
        self.endTime = rospy.Time.now() + rospy.Duration(self.timeoutTime)

    def hasTimedOut(self):
        return rospy.Time.now() > self.endTime 
