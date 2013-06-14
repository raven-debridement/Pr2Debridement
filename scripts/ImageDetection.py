#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from sensor_msgs.msg import Image
import tf

from Util import *
from Constants import *

from threading import *

class ImageDetectionClass():
      def __init__(self):
            #PointStamped list
            self.cancerPoints = []
            #gripper pose
            self.leftGripperPose = None
            self.rightGripperPose = None
            #receptacle pose
            self.receptaclePose = None

            # may only temporarily need
            self.listener = tf.TransformListener()

            #Lock so two arms can access one ImageDetectionClass
            self.cancerLock = Lock()

            # Temporary. Will eventually be placed with real image detection
            # Will subscribe to camera feeds eventually
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            rospy.Subscriber('/wide_stereo/right/image_rect', Image,self.imageCallback)

      def stereoCallback(self, msg):
            """
            Temporary.
            """
            self.cancerLock.acquire()
            self.cancerPoints.append(msg)
            self.cancerLock.release()

      def imageCallback(self, msg):
            # gripperPoses in own frames
            rospy.loginfo('Image received')
            rgp = PoseStamped()
            rgp.header.stamp = msg.header.stamp
            rgp.header.frame_id = ConstantsClass.ToolFrame.Right
            rgp.pose.orientation.w = 1
            self.rightGripperPose = rgp

            lgp = PoseStamped()
            lgp.header.stamp = msg.header.stamp
            lgp.header.frame_id = ConstantsClass.ToolFrame.Left
            lgp.pose.orientation.w = 1
            self.rightGripperPose = lgp
            

      def hasFoundCancer(self):
            return len(self.cancerPoints) > 0
      
      def getCancerPoint(self):
            """
            May update to take argument currPos, and then choose cancer closest to currPos
            
            Also, keep track of cancer points and not cancer poses because we assume the cancer will be on a flat table. May have to reevaluate this assumption
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
            """
            if not self.hasFoundGripper(gripperName):
                  return None

            return Util.poseStampedToPointStamped(self.getGripperPose(gripperName))
      
      
      def hasFoundReceptacle(self):
            return (self.receptaclePose != None)

      def getReceptaclePose(self):
            return self.receptaclePose

      def getReceptaclePoint(self):
            return Util.poseStampedToPointStamped(self.receptaclePose)
            

if __name__ == '__main__':
      rospy.init_node('image_detection_node')
      imageDetector = ImageDetectionClass()
      while True:
            rospy.sleep(10)
