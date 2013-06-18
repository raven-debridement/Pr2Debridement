#!/usr/bin/env python

# Import required Python code.
import roslib
roslib.load_manifest('Pr2Debridement')
import rospy

class ConstantsClass:
    class ArmName:
        Left = 'l_arm'
        Right = 'r_arm'

    class ControllerName:
        JTCartesian = 'pose_controller'
        CartesianTwist = 'twist_controller'

    # ArmName + '_' + ControllerName = full controller name

    class GripperName:
        Left = 'l_gripper'
        Right = 'r_gripper'

    class ToolFrame:
        Left = 'l_gripper_tool_frame'
        Right = 'r_gripper_tool_frame'

    BaseLink = 'base_link'
    StereoName = 'stereo_points_3d'
