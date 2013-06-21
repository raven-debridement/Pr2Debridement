#!/usr/bin/env python

"""
Based off of MIT ee_cart project

https://svn.csail.mit.edu/mit-ros-pkg/stacks/fuerte/trunk/ee_cart_imped/ee_cart_imped_control/src/ee_cart_imped_control/control_switcher.py
"""

import roslib
roslib.load_manifest('Pr2Debridement')
import rospy
import pr2_controller_manager.pr2_controller_manager_interface

from Constants import *

class PR2CMClient:
    '''
    This class switches the arm controllers between JointTrajectoryAction controllers and CartesianTwist controllers. The JointTrajectoryAction controllers are default on the PR2 and can be used with the move_arm package. The CartesianTwist controllers are not default, but are useful for visual servoing, where commanding the velocity of the end effector is necessary
    '''
    
    @staticmethod
    def change_arm_controller(arm_name, controller_name):
        stop_success = PR2CMClient.stop_arm_controller(arm_name)
        
        if not stop_success:
            return False

        load_success = PR2CMClient.load_arm_controller(arm_name, controller_name)
        return (stop_success and load_success)

    @staticmethod
    def stop_arm_controller(arm_name):
        success = True
        success &= pr2_controller_manager.pr2_controller_manager_interface.stop_controller(arm_name + '_' + ConstantsClass.ControllerName.JTCartesian)
        success &= pr2_controller_manager.pr2_controller_manager_interface.stop_controller(arm_name + '_' + ConstantsClass.ControllerName.CartesianTwist)        
        return success

    @staticmethod
    def load_arm_controller(arm_name, controller_name):
        return pr2_controller_manager.pr2_controller_manager_interface.start_controller(arm_name + '_' + controller_name)
        
   
