#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion
from sensor_msgs.msg import Image
import tf
import tf.transformations as tft
import math

import Util
from Constants import ConstantsClass

from threading import Lock

class ImageDetectionClass():
      """
      Used to detect object, grippers, and receptacle
      """
      def __init__(self, normal=None):
            #PointStamped list
            self.objectPoints = []
            #gripper pose. Must both have frame_id of respective tool frame
            self.leftGripperPose = None
            self.rightGripperPose = None
            #receptacle point. Must have frame_id of global (or main camera) frame
            #is the exact place to drop off (i.e. don't need to do extra calcs to move away
            self.receptaclePoint = None
            #table normal. Must be according to global (or main camera) frame
            if normal != None:
                  self.normal = normal
            else:
                  # default to straight up
                  #self.normal = Util.makeQuaternion(.5**.5, 0, -.5**.5, 0)
                  
                  # default to sideways
                  quat = tft.quaternion_from_euler(0,0,-math.pi/2)
                  self.normal = Util.makeQuaternion(quat[3], quat[0], quat[1], quat[2])


            #Lock so two arms can access one ImageDetectionClass
            self.objectLock = Lock()

            # Temporary. Will eventually be placed with real image detection
            # Will subscribe to camera feeds eventually
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            rospy.Subscriber('/wide_stereo/right/image_rect', Image,self.imageCallback)

      def stereoCallback(self, msg):
            """
            Temporary. First click sets receptaclePoint, all others are objectPoints
            """
            if self.receptaclePoint == None:
                  self.receptaclePoint = msg
                  self.receptaclePoint.point.z += .25
            else:
                  self.objectLock.acquire()
                  msg.point.z -= .03 # so gripper doesn't pick up on lip of can
                  self.objectPoints.append(msg)
                  self.objectLock.release()

      def imageCallback(self, msg):
            """
            Temporary. Sets gripper poses to absolutely correct value
            """
            # gripperPoses in own frames
            #rospy.loginfo('Image received')
            rgp = PoseStamped()
            rgp.header.stamp = msg.header.stamp
            rgp.header.frame_id = ConstantsClass.ToolFrame.Right
            rgp.pose.orientation.w = 1
            self.rightGripperPose = rgp

            lgp = PoseStamped()
            lgp.header.stamp = msg.header.stamp
            lgp.header.frame_id = ConstantsClass.ToolFrame.Left
            lgp.pose.orientation.w = 1
            self.leftGripperPose = lgp
            

      def hasFoundObject(self):
            return len(self.objectPoints) > 0

      def getObjectPose(self):
            """
            Returns object point plus the table normal as the orientation
            """
            objectPoint = self.getObjectPoint()
            return Util.pointStampedToPoseStamped(objectPoint, self.normal)
      
      def getObjectPoint(self):
            """
            May update to take argument currPos, and then choose object closest to currPos
            
            Also, keep track of object points and not object poses because we assume the object will be on a flat table.
            """
            if not self.hasFoundObject():
                  return None

            self.objectLock.acquire()
            objectPoint = self.objectPoints[0]
            objectPoint.header.stamp = rospy.Time.now()
            #self.objectPoints = self.objectPoints[:-1]
            self.objectLock.release()

            return objectPoint
      
      def removeFirstObjectPoint(self):
            """
            Debug tool to remove object point from list
            """
            if not self.hasFoundObject():
                  return None

            self.objectLock.acquire()
            self.objectPoints = self.objectPoints[1:]
            self.objectLock.release()


      def hasFoundGripper(self, gripperName):
            """
            gripperName must be from ConstantsClass.GripperName
            """
            if gripperName == ConstantsClass.GripperName.Left:
                  return (self.leftGripperPose != None)
            else:
                  return (self.rightGripperPose != None)

      def getGripperPose(self, gripperName):
            """
            gripperName must be from ConstantsClass.GripperName

            returns PoseStamped with frame_id of gripperName
            """
            if not self.hasFoundGripper(gripperName):
                  return None

            if gripperName == ConstantsClass.GripperName.Left:
                  return self.leftGripperPose
            else:
                  return self.rightGripperPose


      def getGripperPoint(self, gripperName):
            """
            gripperName must be from ConstantsClass.GripperName

            returns PointStamped with frame_id of gripperName
            """
            if not self.hasFoundGripper(gripperName):
                  return None

            return Util.poseStampedToPointStamped(self.getGripperPose(gripperName))
      
      
      def hasFoundReceptacle(self):
            return (self.receptaclePoint != None)

      def getReceptaclePose(self):
            """
            Returns PoseStamped with position of centroid of receptacle and
            orientation of the table normal
            """
            return Util.pointStampedToPoseStamped(self.receptaclePoint, self.normal)

      def getReceptaclePoint(self):
            """
            Returns PointStamped of the centroid of the receptacle
            """
            return self.receptaclePoint

            

if __name__ == '__main__':
      rospy.init_node('image_detection_node')
      imageDetector = ImageDetectionClass()
      while True:
            rospy.sleep(10)
