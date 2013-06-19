#!/usr/bin/env python

import roslib; roslib.load_manifest('Pr2Debridement')
import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku

import rospy
import openravepy as rave
from pr2 import pr2

import time
import tf

from Constants import ConstantsClass
from joint_states_listener.srv import ReturnJointStates

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

        # number of iterations for trajopt
        self.n_steps = 60

        self.env.Load('/home/gkahn/Berkeley/Research/ros_ws/sandbox/Pr2Debridement/data/table.xml')
        
        #trajoptpy.SetInteractive(True)
        slow_down_ratio = .5

        self.robot.SetDOFVelocityLimits(slow_down_ratio*self.robot.GetDOFVelocityLimits())
        self.robot.SetDOFAccelerationLimits(slow_down_ratio*self.robot.GetDOFAccelerationLimits())
 
        # can slow down limits here!
        self.arm.vel_limits = np.array([slow_down_ratio*limit for limit in self.arm.vel_limits])
        # slow down acceleration
        self.arm.acc_limits = np.array([slow_down_ratio*limit for limit in self.arm.acc_limits])

        time.sleep(1)

    def goToArmPose(self, pose, listener=None):
        """
        No path planning. Straight IK
        """
        # must convert to BaseLink frame
        if listener != None:
            while True:
                try:
                    commonTime = listener.getLatestCommonTime(ConstantsClass.BaseLink,pose.header.frame_id)
                    pose.header.stamp = commonTime
                    pose = listener.transformPose(ConstantsClass.BaseLink,pose)
                    rospy.loginfo('converted successully')
                    break
                except tf.Exception:
                    rospy.sleep(.1)

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

        self.arm.goto_pose_matrix(hmat_target, ConstantsClass.BaseLink, self.toolframe)



    def planAndGoToArmPose(self, pose, reqName=ConstantsClass.Request.noRequest, listener=None):
        """
        Path plan using trajopt. Then execute trajectory
        """
        # must convert to BaseLink frame
        if listener != None:
            while True:
                try:
                    commonTime = listener.getLatestCommonTime(ConstantsClass.BaseLink,pose.header.frame_id)
                    pose.header.stamp = commonTime
                    pose = listener.transformPose(ConstantsClass.BaseLink,pose)
                    rospy.loginfo('converted successully')
                    break
                except tf.Exception:
                    rospy.sleep(.1)

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
        Different request types. See ConstantsClass.Request
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
        Useful for debugging
        """
        self.arm.goto_posture('side')



def test():
    rospy.init_node('main_node')
    leftArm = ArmControlClass(ConstantsClass.ArmName.Left)
    
    #leftArm.goToSide()
    #return

    import tf.transformations as tft
    import tf
    from math import pi
    

    listener = tf.TransformListener()

    def stereoCallback(msg):
        test.desiredPose = PoseStamped()
        
        x,y,z = msg.point.x, msg.point.y, msg.point.z

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
        test.desiredPose.pose.position.z += .03
        test.desiredPose.pose.position.y += .1

    test.desiredPose = None


    rospy.Subscriber(ConstantsClass.StereoName, PointStamped, stereoCallback)

    while True:
        rospy.loginfo('outer loop')
        if test.desiredPose != None:
            rospy.loginfo('inner loop')            
            

            leftArm.planAndGoToArmPose(test.desiredPose, ConstantsClass.Request.goNear, listener)
            #leftArm.goToArmPose(test.desiredPose, listener)
            test.desiredPose = None
            return
            
            
        rospy.sleep(.5)




if __name__ == '__main__':    
    test()
