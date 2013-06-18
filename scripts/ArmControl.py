#!/usr/bin/env python

import roslib; roslib.load_manifest('Pr2Debridement')
import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku

import rospy
import openravepy as rave
from PR2 import PR2, Arm
import time

from Constants import *
from joint_states_listener.srv import ReturnJointStates

from geometry_msgs.msg import PoseStamped, PointStamped

class ArmControlClass (PR2):
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

    def __init__ (self, armName, rave_robot=None):      
        PR2.__init__(self, rave_robot)
        
        if armName == ConstantsClass.ArmName.Left:
            self.arm = self.larm
            self.armName = 'leftarm'
        else:
            self.arm = self.rarm
            self.armName = 'rightarm'

        self.joint_names = [self.armName[0]+suffix for suffix in self.joint_name_suffixes]
        self.toolframe = self.armName[0] + self.tool_frame_suffix

        # number of iterations for trajopt
        self.n_steps = 20

        time.sleep(1)

    def goToArmPose(self, pose):
        self.arm.cart_command.publish(pose)
        quat = [pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z]
        xyz = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        ref_frame = pose.header.frame_id
        self.arm.set_cart_target(quat,xyz,ref_frame)

    def planAndGoToArmPose(self, pose):
        
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
        # keep elbow above table
        elbow_target = [0, 0, xyz.z + .15]
        hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

        # inverse kinematics
        manip = self.robot.GetManipulator(self.armName)
        init_joint_target = ku.ik_for_link(hmat_target, manip, self.toolframe, filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
        
        if init_joint_target == None:
            # inverse kinematics failed
            # will do nothing for now, may want to alter xyz_target a little
            return False


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
                # BEGIN pose_target
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target,  # unused
                                "link": self.toolframe,
                                "rot_coeffs" : [1,1,1],
                                "pos_coeffs" : [1,1,1]
                                }
                    
                    },
                #END pose_target
                {
                    "type" : "pose",
                    "params" : {
                            "coeffs" : [20],
                            "xyz" : elbow_target,
                            "wxyz" : quat_target,
                            "link": "l_elbow_flex_link",
                            "rot_coeffs" : [0,0,0],
                            "pos_coeffs" : [0,0,1]
                            }
                    },
                ],
            "init_info" : {
                "type" : "straight_line",
                "endpoint" : init_joint_target.tolist()
                }
            }

       
        # convert dictionary into json-formatted string
        s = json.dumps(request) 
        # create object that stores optimization problem
        prob = trajoptpy.ConstructProblem(s, self.env)
        # do optimization
        result = trajoptpy.OptimizeProblem(prob)

        self.arm.follow_joint_trajectory(result.GetTraj())

        return True

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
        commonTime = listener.getLatestCommonTime("base_link",msg.header.frame_id)
        msg.header.stamp = commonTime
        msg = listener.transformPoint("base_link",msg)

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
        #test.desiredPose.pose.position.z -=.02

    test.desiredPose = None


    rospy.Subscriber(ConstantsClass.StereoName, PointStamped, stereoCallback)

    while True:
        rospy.loginfo('outer loop')
        if test.desiredPose != None:
            rospy.loginfo('inner loop')            
            
            leftArm.planAndGoToArmPose(test.desiredPose)
            test.desiredPose = None
            
            
        rospy.sleep(.5)




if __name__ == '__main__':    
    test()
