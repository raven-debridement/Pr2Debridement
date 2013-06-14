#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped
import tf

class ImageDetectionClass():
      def __init__(self):
            #PointStamped list
            self.cancerPoints = []

            #rospy.init_node('image_detect', anonymous=True)

            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            #rospy.spin()

      def stereoCallback(self, msg):
            self.cancerPoints.append(msg)

      def hasFoundCancer(self):
            return len(self.cancerPoints) > 0
      
      def getCancerPoint(self):
            if not self.hasFoundCancer():
                  return None

            cancerPoint = self.cancerPoints[-1]
            self.cancerPoints = self.cancerPoints[:-1]
            
            return cancerPoint

      

            
