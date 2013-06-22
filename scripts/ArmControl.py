#!/usr/bin/env python

"""
Note: environment loads a xml file for the table. Must change depending on what table you are using
"""

import roslib; roslib.load_manifest('Pr2Debridement')
import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku
import os
import random

import rospy
import openravepy as rave
from pr2 import pr2

import time
import tf
import tf.transformations as tft

from Constants import ConstantsClass
from Util import *
from Pr2Debridement.srv import ReturnJointStates

from geometry_msgs.msg import PointStamped, PoseStamped

import code

class ArmControlClass (pr2.PR2):
    """
    Class for controlling the arms of the PR2
    """

    joint_name_suffixes = ["_shoulder_pan_joint",
                           "_shoulder_lift_joint",
                           "_upper_arm_roll_joint",
                           "_elbow_flex_joint",
                           "_forearm_roll_joint",
                           "_wrist_flex_joint",
                           "_wrist_roll_joint"]

    tool_frame_suffix = '_gripper_tool_frame'

    

    def __init__ (self, armName):      
        pr2.PR2.__init__(self)
        
        if armName == ConstantsClass.ArmName.Left:
            self.arm = self.larm
            self.armName = 'leftarm'
        else:
            self.arm = self.rarm
            self.armName = 'rightarm'

        self.joint_names = [self.armName[0]+suffix for suffix in self.joint_name_suffixes]
        self.toolframe = self.armName[0] + self.tool_frame_suffix

        self.listener = tf.TransformListener()

        # number of iterations for trajopt
        self.n_steps = 60

        ##############################
        # CHOOSE THE TABLE    ########
        ##############################
        table_path = os.path.dirname(__file__) + '/../data/SDHtable.xml'
        #table_path = os.path.dirname(__file__) + '/../data/table.xml'
        self.env.Load(table_path)
        
        # used to slow down the pr2 motions
        slow_down_ratio = .5

        self.robot.SetDOFVelocityLimits(slow_down_ratio*self.robot.GetDOFVelocityLimits())
        self.robot.SetDOFAccelerationLimits(slow_down_ratio*self.robot.GetDOFAccelerationLimits())
 
        # slow down velocites
        self.arm.vel_limits = np.array([slow_down_ratio*limit for limit in self.arm.vel_limits])
        # slow down acceleration
        self.arm.acc_limits = np.array([slow_down_ratio*limit for limit in self.arm.acc_limits])

        time.sleep(1)

 
    def goToArmPose(self, pose, isPlanned, reqName=ConstantsClass.Request.noRequest):
        """
        Go to PoseStamped pose

        If isPlanned is True, then trajopt is used to plan the trajectory
        Otherwise, just IK is used to plan trajectory

        If IK fails, arm doesn't move
        
        The methods for executing the trajectories are in pr2/pr2.py
        """
        # must convert to BaseLink frame
        if self.listener != None:
            try:
                commonTime = self.listener.getLatestCommonTime(ConstantsClass.BaseLink,pose.header.frame_id)
                pose.header.stamp = commonTime
                pose = self.listener.transformPose(ConstantsClass.BaseLink,pose)
            except tf.Exception:
                return
            
        if pose.header.frame_id != ConstantsClass.BaseLink:
            return

        # get the current joint state for joint_names
        rospy.wait_for_service("return_joint_states")
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(self.joint_names)
        
        # set the start joint position
        joint_start = resp.position
        self.robot.SetDOFValues(joint_start, self.robot.GetManipulator(self.armName).GetArmIndices())

        # initialize trajopt inputs
        quat = pose.pose.orientation
        xyz = pose.pose.position
        quat_target = [quat.w, quat.x, quat.y, quat.z]
        xyz_target = [xyz.x, xyz.y, xyz.z]
        hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )
        
        # if no planning
        if not isPlanned:
            self.arm.goto_pose_matrix(hmat_target, ConstantsClass.BaseLink, self.toolframe)
            return True

        # inverse kinematics
        manip = self.robot.GetManipulator(self.armName)
        init_joint_target = ku.ik_for_link(hmat_target, manip, self.toolframe, filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
        
        if init_joint_target == None:
            # inverse kinematics failed
            # will do nothing for now, may want to alter xyz_target a little
            rospy.loginfo('IK failed')
            return False




        request = self.getRequest(reqName, xyz_target, quat_target, init_joint_target)
        
        if request == None:
            return
               
        # convert dictionary into json-formatted string
        s = json.dumps(request) 
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.env)
        # do optimization
        result = trajoptpy.OptimizeProblem(prob)

        self.arm.follow_joint_trajectory(result.GetTraj())

        return True

    def getRequest(self, reqName, xyz_target, quat_target, init_joint_target):
        """
        Different request types for trajopt costs/constraints. See ConstantsClass.Request
        """
        if reqName == ConstantsClass.Request.goReceptacle:
            
            request = {
            "basic_info" : {
                "n_steps" : self.n_steps,
                "manip" : self.armName, 
                "start_fixed" : True 
                },
            "costs" : [
                {
                    "type" : "joint_vel",
                    "params": {"coeffs" : [1]} 
                    },
                {
                    "type" : "collision",
                    "params" : {
                        "coeffs" : [20],
                        "dist_pen" : [0.025] 
                        }
                    },
                {
                        "type" : "pose",
                        "name" : "target_pose",
                        "params" : {"xyz" : xyz_target, 
                                    "wxyz" : quat_target,
                                    "link": self.toolframe,
                                    "rot_coeffs" : [1,1,0],
                                    "pos_coeffs" : [0,0,0],
                                    "coeffs" : [20]
                                    }
                        },
                ],
            "constraints" : [
                # BEGIN pose_target
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target,
                                "link": self.toolframe,
                                "rot_coeffs" : [0,0,0],
                                "pos_coeffs" : [1,1,1]
                                }
                    
                    },
                #END pose_target
                ],
            "init_info" : {
                "type" : "straight_line",
                "endpoint" : init_joint_target.tolist()
                }
            }

        else:
            request = {
                "basic_info" : {
                    "n_steps" : self.n_steps,
                    "manip" : self.armName, 
                    "start_fixed" : True 
                    },
                "costs" : [
                    {
                        "type" : "joint_vel",
                        "params": {"coeffs" : [1]} 
                        },
                    {
                        "type" : "collision",
                        "params" : {
                            "coeffs" : [20],
                            "dist_pen" : [0.025] 
                            }
                        },
                    ],
                "constraints" : [
                    {
                        "type" : "pose",
                        "name" : "target_pose",
                        "params" : {"xyz" : xyz_target, 
                                    "wxyz" : quat_target,
                                    "link": self.toolframe,
                                    "rot_coeffs" : [1,1,1],
                                    "pos_coeffs" : [1,1,1]
                                    }
                        
                        },
                    ],

                "init_info" : {
                    "type" : "straight_line",
                    "endpoint" : init_joint_target.tolist()
                    }
                }
            

        return request



    def goToSide(self):
        """
        Commands the arm to the side

        Useful for debugging
        """
        self.arm.goto_posture('side')

    def servoToPose(self, currPoseStamped, desPoseStamped):
        """
        Given currPoseStamped and desPoseStamped, uses the local
        inverse jacobian to determine the new joint positions
        needed to move closer to desPoseStamped

        Because the jacobian is a local approximation, incorrect
        behavior becomes more prevalent the further currPoseStamped
        is from desPoseStamped

        Follows page 4 (specifically slide 16)
        http://public.cranfield.ac.uk/c5354/teaching/av/LECTURE_NOTES/lecture11-2x2.pdf

        Some problems I've run into:
        
        The movement is jerky. I'm not sure if this is due to an error
        in the below calculations or just in the execution of the new
        joint positions. I'm inclined to think it is in the execution
        of the new joint positions (i.e. the last call of this method).
        When trying to debug this, make sure you have already executed
        sudo ntpdate pr2base

        As currPoseStamped approaches desPoseStamped, the movements
        become so small to a point the arm doesn't even really move.
        You can change this by increasing alpha, but then note the arm moves
        a lot in the beginning of the trajectory and will have less
        time to correct for errors. One semi-hackish solution I have sort
        of tried is to pass in a desPoseStamped that is slightly to the right
        of the actual objectPose (by subtracting a few centimeters from
        objectPose. I think I do this in Master.py)




        When testing the servo, I suggest testing one step at a time.
        I currently have it set up that way in Master.py
        """
        
        try:
            currPoseStamped, desPoseStamped = convertToSameFrameAndTime(currPoseStamped, desPoseStamped, self.listener)
        except tf.Exception:
            return
        
        # get the current joint state for joint_names
        rospy.wait_for_service("return_joint_states")
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(self.joint_names)
        
        # set current joint values on robot
        self.robot.SetDOFValues(resp.position, self.robot.GetManipulator(self.armName).GetArmIndices())
        currJoints = np.array(resp.position)
        
        # calculate position and rotation jacobians
        manip = self.robot.GetManipulator(self.armName)
        Jpos = manip.CalculateJacobian()
        Jrot = manip.CalculateRotationJacobian()

        # combine to form Jacobian, then find pseudo-inverse
        J = np.vstack((Jpos, Jrot))
        Jinv = np.linalg.pinv(J)
        
        

        # get current position
        pos = currPoseStamped.pose.position
        currPos = np.array([pos.x, pos.y, pos.z])
        # get current quaternion
        quat = currPoseStamped.pose.orientation
        currQuat = np.array([quat.w, quat.x, quat.y, quat.z])

        # get desired position
        pos = desPoseStamped.pose.position
        desPos = np.array([pos.x, pos.y, pos.z])
        # get desired quaternion
        quat = desPoseStamped.pose.orientation
        desQuat = np.array([quat.w, quat.x, quat.y, quat.z])

        # change in pos/quat
        deltaPos = desPos - currPos
        
        # the following method for computing deltaQuat
        # was derived by Ankush
        # also, simply desQuat - currQuat serves as
        # a good approximation also
        d = desQuat - currQuat
        deltaQuat = d - np.dot(d, currQuat)*currQuat

        # change in pose
        deltaPose = np.hstack((deltaPos, deltaQuat))
        
        # now do the actually calculation
        # alpha determines movement/decay rate
        alpha = .5
        desJoints = alpha*np.dot(Jinv, deltaPose) + currJoints
        
        self.arm.goto_joint_positions(desJoints)
        
        


def test_goToArmPose():
    """
    Waits for stereo click,
    then moves to the clicked point
    """
    rospy.init_node('main_node')
    leftArm = ArmControlClass(ConstantsClass.ArmName.Left)
    
    # uncomment to have arms go to side, good for resetting
    #leftArm.goToSide()
    #return

    import tf.transformations as tft
    import tf
    from math import pi

    def stereoCallback(msg):
        test.desiredPose = PoseStamped()
        
        x,y,z = msg.point.x, msg.point.y, msg.point.z

        # set orientation to horizontal right
        eulerx = 0
        eulery = 0
        eulerz = -pi/2
        quat = tft.quaternion_from_euler(eulerx,eulery,eulerz)
        test.desiredPose.pose.orientation.w = quat[3]
        test.desiredPose.pose.orientation.x = quat[0]
        test.desiredPose.pose.orientation.y = quat[1]
        test.desiredPose.pose.orientation.z = quat[2]

        test.desiredPose.header = msg.header
        test.desiredPose.pose.position = msg.point
        
        # currently set to return to the left and above
        # can change depending on where you want the
        # arm to go
        test.desiredPose.pose.position.z += .03
        test.desiredPose.pose.position.y += .1

    test.desiredPose = None


    rospy.Subscriber(ConstantsClass.StereoName, PointStamped, stereoCallback)

    while not rospy.is_shutdown():
        rospy.loginfo('outer loop')
        if test.desiredPose != None:
            rospy.loginfo('inner loop')            
            

            leftArm.goToArmPose(test.desiredPose, True, ConstantsClass.Request.goNear)
            test.desiredPose = None
            return
            
            
        rospy.sleep(.5)


def test_servo():
    """
    Can expand to become an actual test of the
    servoing method. Or just test from Master.py,
    which is what I have been doing
    """
    rospy.init_node('main_node')
    leftArm = ArmControlClass(ConstantsClass.ArmName.Left)
    leftArm.servoToPose(PoseStamped(), PoseStamped())
    return

if __name__ == '__main__':    
    test_goToArmPose()
    #test_servo()
