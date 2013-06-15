#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion
from sensor_msgs.msg import Image
import tf

import Util
from Constants import *

from threading import *

class ImageDetectionClass():
      """
      Used to detect cancer, grippers, and receptacle
      """
      def __init__(self, normal=None):
            #PointStamped list
            self.cancerPoints = []
            #gripper pose. Must both have frame_id of respective tool frame
            self.leftGripperPose = None
            self.rightGripperPose = None
            #receptacle point. Must have frame_id of global (or main camera) frame
            self.receptaclePoint = None
            #table normal. Must be according to global (or main camera) frame
            if normal != None:
                  self.normal = normal
            else:
                  # default to straight up
                  self.normal = Util.makeQuaternion(.5**.5, 0, -.5**.5, 0)


            #Lock so two arms can access one ImageDetectionClass
            self.cancerLock = Lock()

            # Temporary. Will eventually be placed with real image detection
            # Will subscribe to camera feeds eventually
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            rospy.Subscriber('/wide_stereo/right/image_rect', Image,self.imageCallback)

      def stereoCallback(self, msg):
            """
            Temporary. First click sets receptaclePoint, all others are cancerPoints
            """
            if self.receptaclePoint == None:
                  self.receptaclePoint = msg
                  self.receptaclePoint.point.z += .1
            else:
                  self.cancerLock.acquire()
                  self.cancerPoints.append(msg)
                  self.cancerLock.release()

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
            

      def hasFoundCancer(self):
            return len(self.cancerPoints) > 0

      def getCancerPose(self):
            """
            Returns cancer point plus the table normal as the orientation

            Removes specific cancer from list
            """
            cancerPoint = self.getCancerPoint()
            return Util.pointStampedToPoseStamped(cancerPoint, self.normal)
      
      def getCancerPoint(self):
            """
            May update to take argument currPos, and then choose cancer closest to currPos
            
            Also, keep track of cancer points and not cancer poses because we assume the cancer will be on a flat table.

            Removes specific cancer from list
            """
            if not self.hasFoundCancer():
                  return None

            self.cancerLock.acquire()
            cancerPoint = self.cancerPoints[-1]
            self.cancerPoints = self.cancerPoints[:-1]
            self.cancerLock.release()

            return cancerPoint
      
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
