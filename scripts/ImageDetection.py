#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Image
import tf

from threading import *

class ImageDetectionClass():
      def __init__(self):
            #PointStamped list
            self.cancerPoints = []
            #gripper pose
            self.gripperPose = None
            
            # may only temporarily need
            self.listener = tf.TransformListener()

            #Lock so two arms can access one ImageDetectionClass
            self.cancerLock = Lock()

            # Temporary. Will eventually be placed with real image detection
            # Will subscribe to camera feeds eventually
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            rospy.Subscriber('/wide_stereo/right/image_rect', sensor_msgs.Image,self.imageCallback)

      def stereoCallback(self, msg):
            """
            Temporary.
            """
            self.cancerLock.acquire()
            self.cancerPoints.append(msg)
            self.cancerLock.release()

      def imageCallback(self, msg):
            return 0

      def hasFoundCancer(self):
            return len(self.cancerPoints) > 0
      
      def getCancerPoint(self):
            """
            May update to take argument currPos, and then choose cancer closest to currPos
            """
            if not self.hasFoundCancer():
                  return None

            self.cancerLock.acquire()
            cancerPoint = self.cancerPoints[-1]
            self.cancerPoints = self.cancerPoints[:-1]
            self.cancerLock.release()

            return cancerPoint
      
      def hasFoundGripper(self):
            return (self.gripperPose != None)

      def getGripperPose(self):
            if not self.hasFoundGripper():
                  return None

            return self.gripperPose
      
      

            
