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
            #Lock so two arms can access one ImageDetectionClass
            self.lock = Lock()

            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            #rospy.spin()

      def stereoCallback(self, msg):
            self.lock.acquire()
            self.cancerPoints.append(msg)
            self.lock.release()

      def hasFoundCancer(self):
            return len(self.cancerPoints) > 0
      
      def getCancerPoint(self):
            if not self.hasFoundCancer():
                  return None

            self.lock.acquire()
            cancerPoint = self.cancerPoints[-1]
            self.cancerPoints = self.cancerPoints[:-1]
            self.lock.release()

            return cancerPoint

      

            
