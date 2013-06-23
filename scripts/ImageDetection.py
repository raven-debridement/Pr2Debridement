#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import sys
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Quaternion, TransformStamped
from sensor_msgs.msg import Image
from ar_pose.msg import *
import tf
import tf.transformations as tft
import math

import Util
from Constants import ConstantsClass
from image_proc import ImageProcessingClass

from threading import Lock

ids_to_joints = {0: ConstantsClass.GripperName.Right,
                 1: ConstantsClass.GripperName.Left}

class ImageDetectionClass():
    
      class State():
            CalibrateLeft = 0
            CalibrateRight = 1
            Calibrated = 2
            Calibrating = 3 # waiting for signal to calibrate left or right

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
            self.listener = tf.TransformListener()
            self.state = ImageDetectionClass.State.Calibrating
            self.objectProcessing = ImageProcessingClass()

            # Temporary. For finding the receptacle
            rospy.Subscriber('stereo_points_3d', PointStamped, self.stereoCallback)
            # Get grippers using AR
            rospy.Subscriber('/stereo_pose', ARMarkers, self.arCallback)

      def setState(self, state):
            self.state = state

      def stereoCallback(self, msg):
            """
            Temporary. Initialize receptaclePoint
            """
            if self.receptaclePoint == None:
                  self.receptaclePoint = msg
                  self.receptaclePoint.point.z += .2
            

      def arCallback(self, msg):
            markers = msg.markers
            for marker in markers:
                arframe = ConstantsClass.StereoAR + "_" + str(marker.id)
                if ids_to_joints[marker.id] == ConstantsClass.GripperName.Left:
                    self.arHandler(arframe, "left")
                elif ids_to_joints[marker.id] == ConstantsClass.GripperName.Right:
                    self.arHandler(arframe, "right")

      def arHandler(self, arframe, armname):
        if armname == 'left':
            gripper_name = ConstantsClass.GripperName.Left
            gripper_frame = 'calculated_gripper_frame_l'
            tool_frame = ConstantsClass.ToolFrame.Left
        else:
            gripper_name = ConstantsClass.GripperName.Right
            gripper_frame = 'calculated_gripper_frame_r'
            tool_frame = ConstantsClass.ToolFrame.Right
        if ((self.state == ImageDetectionClass.State.CalibrateLeft and armname == "left") or
            (self.state == ImageDetectionClass.State.CalibrateRight and armname == "right")):
            self.listener.waitForTransform(ConstantsClass.Camera, arframe,
                                           rospy.Time.now(), rospy.Duration(2.0))
            time = self.listener.getLatestCommonTime(tool_frame,
                                                     arframe)
            (trans,rot) = self.listener.lookupTransform(arframe,
                                                        tool_frame,
                                                        time)
            self.transform = Util.makeTransform(arframe, gripper_frame, trans, rot, rospy.Time())
            print armname + " offset", (trans, rot)
            self.state = ImageDetectionClass.State.Calibrated
        elif self.isCalibrated():
            self.listener.waitForTransform(ConstantsClass.Camera, arframe,
                                           rospy.Time.now(), rospy.Duration(2.0))
            time = self.listener.getLatestCommonTime(ConstantsClass.Camera, arframe)
            self.transform.header.stamp = time
            self.listener.setTransform(self.transform)
            calibrated_pose = PoseStamped()
            calibrated_pose.header.stamp = time
            calibrated_pose.header.frame_id = gripper_frame
            gp = self.listener.transformPose(ConstantsClass.Camera, calibrated_pose)
            gpfpose = self.listener.transformPose(ConstantsClass.ToolFrame.Left, gp)
            if armname == "left":
                self.leftGripperPose = gp
            else:
                self.rightGripperPose = gp

      def isCalibrated(self):
            return self.state == ImageDetectionClass.State.Calibrated
                

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

def testCalibration():
    rospy.init_node('image_detection_node')
    imageDetector = ImageDetectionClass()
    while not rospy.is_shutdown():
        while not imageDetector.isCalibrated():
            imageDetector.setState(ImageDetectionClass.State.CalibrateLeft)
        rospy.sleep(.5)
      

if __name__ == '__main__':
      testCalibration()
