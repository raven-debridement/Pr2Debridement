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

ids_to_joints = {0: ConstantsClass.GripperName.Left,
                 1: ConstantsClass.GripperName.Right}

class ImageDetectionClass():
      """
      Used to detect object, grippers, and receptacle
      """
      def __init__(self, normal=None):
            #PointStamped TEMP!!!!!!!!!!!!!!
            self.objectPoint = None
            
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
            # TEMP!!!!
            self.objectLock = Lock()

            self.listener = tf.TransformListener()

            # image processing to find object
            self.objectProcessing = ImageProcessingClass()

            # Temporary. Will eventually be placed with real image detection
            # Will subscribe to camera feeds eventually
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)

            #rospy.Subscriber('/wide_stereo/right/image_rect', Image,self.imageCallback)
            # Get grippers using AR
            rospy.Subscriber('/stereo_pose', ARMarkers, self.arCallback)

      def stereoCallback(self, msg):
            """
            Temporary. First click sets receptaclePoint, all others are objectPoints
            """
            if self.receptaclePoint == None:
                  self.receptaclePoint = msg
                  self.receptaclePoint.point.z += .2
            else:
                  self.objectLock.acquire()
                  msg.point.z -= .03 # so gripper doesn't pick up on lip of can
                  self.objectPoint = msg
                  self.objectLock.release()
            

      def arCallback(self, msg):
            markers = msg.markers
            for marker in markers:
                pose = PoseStamped()
                pose.header.stamp = marker.header.stamp
                pose.header.frame_id = ConstantsClass.Camera
                pose.pose = marker.pose.pose
                if ids_to_joints(marker.id) == ConstantsClass.GripperName.Left:
                    self.listener.waitForTransform(ConstantsClass.GripperName.Left,
                                                   ConstantsClass.Camera,
                                                   rospy.Time(),
                                                   rospy.Duration(4.0))
                    gp = self.listener.transformPose(ConstantsClass.GripperName.Left, pose)
                    print "left", gp
                    self.imageCallback(msg)
                    #self.leftGripperPose = pose
                if ids_to_joints(marker.id) == ConstantsClass.GripperName.Right:
                    self.listener.waitForTransform(ConstantsClass.GripperName.Right,
                                                   ConstantsClass.Camera,
                                                   rospy.Time(),
                                                   rospy.Duration(4.0))
                    gp = self.listener.transformPose(ConstantsClass.GripperName.Right, pose)
                    print "right", gp
                    self.imageCallback(msg)
                    #self.rightGripperPose = pose
                

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
            

      """
      def hasFoundObject(self):
          return self.objectProcessing.canProcess()  
          #return self.objectPoint != None
      """

      def getObjectPose(self):
            """
            Returns object point plus the table normal as the orientation

            Returns None if no object found
            """
            objectPoint = self.getObjectPoint()
            if objectPoint == None:
                  return None

            return Util.pointStampedToPoseStamped(objectPoint, self.normal)
      
      def getObjectPoint(self):
            """
            May update to take argument currPos, and then choose object closest to currPos
            
            Also, keep track of object points and not object poses because we assume the object will be on a flat table.

            Returns None if no object found
            """
            #if not self.hasFoundObject():
            #      return None

            #objectPoint = self.objectProcessing.getClosestToCentroid()
            objectPoint = self.objectPoint

            if objectPoint == None:
                  return None
            #objectPoint.point.z -=.08 #TEMP depends on height of object
            
            # TEMP!!!!!!!!!
            self.objectLock.acquire()
            #objectPoint = self.objectPoint
            objectPoint.header.stamp = rospy.Time.now()
            self.objectLock.release()
            return objectPoint
            
      
      def removeObjectPoint(self):
            #Debug tool to remove object point
            if not self.hasFoundObject():
                  return None

            self.objectLock.acquire()
            self.objectPoint = None
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

def test():     
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
