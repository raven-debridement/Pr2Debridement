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

from joint_states_listener.srv import ReturnJointStates

from geometry_msgs.msg import PoseStamped, PointStamped

class PlannerArm(Arm):
    """
    Planner class for the Arm.
    """

    def __init__ (self, pr2, rl):
        Arm.__init__(self, pr2, rl)
        self.openrave_dofs = self.manip.GetArmIndices()

       
    def move_arm(self, goal_tfm):
        """
        Generate a collision free trajectory given a goal transform
        """
        print goal_tfm
        self.pr2.robot.SetActiveManipulator(self.manip)
        basemanip = rave.interfaces.BaseManipulation(self.pr2.robot)

        traj = basemanip.MoveToHandPosition(matrices=[goal_tfm],execute=False,outputtrajobj=True)
        if traj:
            self.execute_openrave_trajectory(traj)
        else:
            rospy.loginfo('OpenRAVE plan failed!')
        

    def execute_openrave_trajectory(self, traj, sample_freq=10):
        """
        executes the trajectory on the real robot.
        traj is a trajectory generated through openrave planners.
        """
        sample_times    = np.arange(0, traj.GetDuration(), 1.0/sample_freq)
        print sample_times
        print self.openrave_dofs

        cspec           = traj.GetConfigurationSpecification()
        time_derivative = 0
        
        traj_joints = []
        for t in sample_times:
            joints = cspec.ExtractJointValues(traj.Sample(t), self.pr2.robot, self.openrave_dofs, time_derivative)
            print joints
            traj_joints.append(joints)

        self.follow_joint_trajectory(traj_joints)


class PlannerPR2 (PR2):
    """
    Planner class for PR2 with planning arms.
    """

    joint_name_suffixes = ["_shoulder_pan_joint",
                           "_shoulder_lift_joint",
                           "_upper_arm_roll_joint",
                           "_elbow_flex_joint",
                           "_forearm_roll_joint",
                           "_wrist_flex_joint",
                           "_wrist_roll_joint"]

    tool_frame_suffix = '_gripper_tool_frame'

    def __init__ (self, rave_robot=None):      
        PR2.__init__(self, rave_robot)
        time.sleep(1)
        #self.rarm = PlannerArm(self, 'r')
        #self.larm = PlannerArm(self, 'l')


    def goToArmPose(self, lr, pose):

        body = rave.RaveCreateKinBody(self.env,'')
        body.SetName('testbody')
        body.InitFromBoxes(np.array([[0,0,0,20,20,pose.pose.position.z - .05]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
        #self.env.Add(body,True)
        print(pose.pose.position.z)

        if lr == 'l':
            arm = self.larm
            armName = 'leftarm'
        else:
            arm = self.rarm
            armName = 'rightarm'

        joint_names = [lr+suffix for suffix in self.joint_name_suffixes]
        toolframe = lr + self.tool_frame_suffix
        
        rospy.wait_for_service("return_joint_states")
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
        #print(pplist(resp.position))
        
        joint_start = resp.position
        self.robot.SetDOFValues(joint_start, self.robot.GetManipulator(armName).GetArmIndices())

        quat = pose.pose.orientation
        xyz = pose.pose.position
        quat_target = [quat.w, quat.x, quat.y, quat.z]
        xyz_target = [xyz.x, xyz.y, xyz.z]
        elbow_target = [0, 0, xyz.z + .1]
        #quat_target = [1,0,0,0] # wxyz
        #xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
        hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

# BEGIN ik
        manip = self.robot.GetManipulator(armName)
        init_joint_target = ku.ik_for_link(hmat_target, manip, toolframe, filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
        # if init_joint_target is NONE, return failure. or try a nearby point
# END ik
        n_steps = 10

        request = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : armName, 
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
            "constraints" : [
                # BEGIN pose_target
                {
                    "type" : "pose",
                    "name" : "target_pose",
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target,  # unused
                                "link": toolframe,
                                "rot_coeffs" : [1,1,0],
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

        """
        request = {
            "basic_info" : {
                "n_steps" : 10,
                "manip" : armName, # see below for valid values
                "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
                },
            "costs" : [
                {
                    "type" : "joint_vel", # joint-space velocity cost
                    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
                    # Also valid: "coeffs" : [7,6,5,4,3,2,1]
                    },
                {
                    "type" : "continuous_collision",
                    "name" :"cont_coll", # shorten name so printed table will be prettier
                    "params" : {
                        "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
                        "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
                        }
                    }
                ],
            "constraints" : [
                # BEGIN pose_constraint
                {
                    "type" : "pose", 
                    "params" : {"xyz" : xyz_target, 
                                "wxyz" : quat_target, 
                                "link": toolframe,
                                # "timestep" : 9
                                # omitted because timestep = n_steps-1 is default
                                # "pos_coeffs" : [1,1,1], # omitted because that's default
                                "pos_coeffs" : [1,1,1],
                                "rot_coeffs" : [1,1,1]
                                }


                    
                    } ,
                # END pose_constraint
                ],
            # BEGIN init
            "init_info" : {
                "type" : "straight_line", # straight line in joint space.
                "endpoint" : init_joint_target.tolist() # need to convert numpy array to list
                }
            # END init
            }
        """
        s = json.dumps(request) # convert dictionary into json-formatted string
        prob = trajoptpy.ConstructProblem(s, self.env) # create object that stores optimization problem
        result = trajoptpy.OptimizeProblem(prob) # do optimization
        #print(result.GetTraj())
        arm.follow_joint_trajectory(result.GetTraj())

#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])

def test():
    rospy.init_node('main_node')
    #jointStatesServer = LatestJointStates()
    pr2 = PlannerPR2()
    #pr2.goToArmPose('l',PoseStamped())
    
    import tf.transformations as tft
    from Constants import ConstantsClass
    from math import pi
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
        test.desiredPose.pose.position.z += .15

    test.desiredPose = None


    rospy.Subscriber(ConstantsClass.StereoName, PointStamped, stereoCallback)



    while True:
        rospy.loginfo('outer loop')
        if test.desiredPose != None:
            rospy.loginfo('inner loop')            
            
            pr2.goToArmPose('l',test.desiredPose)
            test.desiredPose = None
            
            
        rospy.sleep(.5)




if __name__ == '__main__':
    """
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    env.Load("../data/table.xml")

    robot = env.GetRobots()[0]
    print(dir(robot))
    print(dir(robot.GetManipulator('rightarm')))
    print(robot.GetManipulator('rightarm').GetArmJoints())
    print((robot.GetJoints())[27:33])
    joint = (robot.GetJoints())[1]
    #print(dir(joint))
    #print(joint.GetValue)
    """
    
    test()
    """
    rospy.init_node('main_node')
    pr2 = PlannerPR2()
    pr2.goToArmPose('l',PoseStamped())
    """
    


"""
env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
env.Load("../data/table.xml")

robot = env.GetRobots()[0]
print(robot.GetManipulator('rightarm'))

# NEED TO CHANGE
joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074]
robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())

quat_target = [1,0,0,0] # wxyz
xyz_target = [6.51073449e-01,  -1.87673551e-01, 4.91061915e-01]
hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

# BEGIN ik
manip = robot.GetManipulator("rightarm")
init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
    filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
# END ik


request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : "rightarm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
    # Also valid: "coeffs" : [7,6,5,4,3,2,1]
  },
  {
    "type" : "continuous_collision",
    "name" :"cont_coll", # shorten name so printed table will be prettier
    "params" : {
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    }
  }
  ],
  "constraints" : [
  # BEGIN pose_constraint
  {
    "type" : "pose", 
    "params" : {"xyz" : xyz_target, 
                "wxyz" : quat_target, 
                "link": "r_gripper_tool_frame",
                # "timestep" : 9
                # omitted because timestep = n_steps-1 is default
                # "pos_coeffs" : [1,1,1], # omitted because that's default
                "rot_coeffs" : ([0,0,0] if args.position_only else [1,1,1])
                }
                 
  }
  # END pose_constraint
  ],
  # BEGIN init
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : init_joint_target.tolist() # need to convert numpy array to list
  }
  # END init
}
s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
result = trajoptpy.OptimizeProblem(prob) # do optimization
print 'Result below:'
print (result.GetTraj())
print result
"""


"""
                # BEGIN elbow_target
                {
                    "type" : "pose",
                    "name" : "elbow_pose",
                    "params" : {
                        "xyz" : elbow_target,
                        "wxyz" : quat_target,
                        "link" : "l_elbow_flex_link",
                        "rot_coeffs" : [0,0,0],
                        "pos_coeffs" : [0,0,1],
                        "coeffs" : [20]
                        },
                    }
                # END elbow_target
"""
