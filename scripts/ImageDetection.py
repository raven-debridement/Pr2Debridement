#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped
import tf

from threading import *

class ImageDetectionClass():
      def __init__(self):
            #PointStamped list
            self.cancerPoints = []
            #gripper pose
            self.gripperPose = None


            #Lock so two arms can access one ImageDetectionClass
            self.lock = Lock()

            # Temporary. Will eventually be placed with real image detection
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            #rospy.spin()

      def stereoCallback(self, msg):
            """
            Temporary.
            """
            self.lock.acquire()
            self.cancerPoints.append(msg)
            self.lock.release()

      def hasFoundCancer(self):
            return len(self.cancerPoints) > 0
      
      def getCancerPoint(self):
            """
            May update to take argument currPos, and then choose cancer closest to currPos
            """
            if not self.hasFoundCancer():
                  return None

            self.lock.acquire()
            cancerPoint = self.cancerPoints[-1]
            self.cancerPoints = self.cancerPoints[:-1]
            self.lock.release()

            return cancerPoint
      
      def hasFoundGripper(self):
            return (self.gripperPose != None)

      def getGripperPose(self):
            
      
      

            
