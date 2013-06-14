#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy

class ConstantsClass:
    class ArmName:
        LeftArm = 'l_arm'
        RightArm = 'r_arm'
        
    class ControllerName:
        JointTractoryAction = 'controller'
        CartesianTwist = 'twist_controller'

    # ArmName + '_' + ControllerName = full controller name

    class ToolFrame:
        Left = 'l_gripper_tool_frame'
        Right = 'r_gripper_tool_frame'
