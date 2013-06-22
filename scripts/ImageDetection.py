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
from image_proc import ImageProcessingClass

from threading import Lock

class ImageDetectionClass():
      """
      Used to detect object, grippers, and receptacle
      """
      def __init__(self, normal=None):
            
            #gripper pose. Must both have frame_id of respective tool frame
            self.leftGripperPose = None
            self.rightGripperPose = None
            #receptacle point. Must have frame_id of global (or main camera) frame
            #is the exact place to drop off (i.e. don't need to do extra calcs to move away)
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


            # image processing to find object
            self.objectProcessing = ImageProcessingClass()

            # Temporary. For finding the receptacle
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            # Temporary. For continuously updating the gripper poses
            rospy.Subscriber('/wide_stereo/right/image_rect', Image,self.imageCallback)

      def stereoCallback(self, msg):
            """
            Temporary. Initialize receptaclePoint
            """
            if self.receptaclePoint == None:
                  self.receptaclePoint = msg
                  self.receptaclePoint.point.z += .2
            

      def imageCallback(self, msg):
            """
            Temporary. Sets gripper poses to absolutely correct value
            """
            # gripperPoses in own frames
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
            

      def getObjectPose(self):
            """
            Returns a PoseStamped of the object point
            plus the table normal as the orientation

            Returns None if no object found
            """
            objectPoint = self.getObjectPoint()
            if objectPoint == None:
                  return None

            return Util.pointStampedToPoseStamped(objectPoint, self.normal)
      
      def getObjectPoint(self):
            """
            Returns PointStamped of the object point

            Returns None if no object found
            """
            objectPoint = self.objectProcessing.getClosestToCentroid()
            
            if objectPoint == None:
                  return None
            
            # TO BE MODIFIED DEPENDING ON OBJECT
            # determines how low to grip the object
            objectPoint.point.z -=.03 
            
            return objectPoint
            

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



def test():
      """
      Prints when an objectPoint has been detected
      Mostly a test of the ImageProcessing class
      """
      rospy.init_node('image_detection_node')
      imageDetector = ImageDetectionClass()
      while not rospy.is_shutdown():
            objectPoint = imageDetector.getObjectPoint()
            if objectPoint != None:      
                  print(objectPoint)
            else:
                  print('Not Found')
            rospy.sleep(.5)
      

if __name__ == '__main__':
      test()
