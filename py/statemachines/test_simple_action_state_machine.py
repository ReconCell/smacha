#!/usr/bin/env python

import os
import roslib
import rospy
import smach
import smach_ros

import socket
import struct
import time
import math
from std_msgs.msg import Bool

# Brings in the SimpleActionClient
import actionlib

# Action messages
import robot_module.msg

# Robot functions
from robotfunctions.getrobotpose import getrobotpose_quat
from robotfunctions.getrobotpose import getrobotpose_rmat
from robotfunctions.setdesiredgpio import setdesiredgpio
from robotfunctions.gravcompon import gravcompon
from robotfunctions.gravcompoff import gravcompoff
from robotfunctions.getrobotpose import getrobotpose
from robotfunctions.getrobotjoints import getrobotjoints
from robotfunctions.getrobotfinishsuccessful import getrobotfinishsuccessful 

# GPIO Service
import gpio.srv as gpio
gpio_service_1 = None
gpio_service_2 = None

# Set robot tool service
from robot_module.srv import SetRobotTool
set_robot_tool_service_1 = None
set_robot_tool_service_2 = None

TOOL_EXCHANGE_GPIO = 0
TOOL_EXCHANGE_CLOSE = 0
TOOL_EXCHANGE_OPEN = 1

TOOL_EXCHANGE_AIR_GPIO = 1
TOOL_EXCHANGE_AIR_OFF = 0
TOOL_EXCHANGE_AIR_ON = 1

HOUSING_GRIPPER_VACUUM_GPIO = 2
HOUSING_GRIPPER_VACUUM_OFF = 0
HOUSING_GRIPPER_VACUUM_ON = 1
HOUSING_GRIPPER_FINGER_GPIO = 3
HOUSING_GRIPPER_FINGER_OPEN = 0
HOUSING_GRIPPER_FINGER_CLOSE = 1

LWRDRIVE_FINGER_GPIO = 2
LWRDRIVE_FINGER_OPEN = 1
LWRDRIVE_FINGER_CLOSE = 0
BULBHOLDER_FINGER_GPIO = 3
BULBHOLDER_FINGER_OPEN = 1
BULBHOLDER_FINGER_CLOSE = 0

CART_SLOW_MOVEMENT = 5
CART_AVARAGE_MOVEMENT = 2
CART_FAST_MOVEMENT = 1
CART_FASTFAST_MOVEMENT = 0.6

JOINT_SLOW_MOVEMENT = 0.05
JOINT_AVARAGE_MOVEMENT = 0.1
JOINT_FAST_MOVEMENT = 0.7
JOINT_FASTFAST_MOVEMENT = 0.9

HEXAPOD_CLAMPS = (5, -1, 5)
CLAMP_OPEN = 0
CLAMP_CLOSE = 1

# Scipy I/O for reading .mat files
# import scipy.io
# from scipy.io import savemat

# Import SimpleActionState for action server integration into states
from smach_ros import SimpleActionState
from smach_ros import ServiceState

import numpy as np

# from tf.transformations import quaternion_from_matrix
# from tf.transformations import quaternion_inverse
# from tf.transformations import rotation_matrix

# Creates the global Action client variables
# jointclient = actionlib.SimpleActionClient('ur_p2p_action', robot_module.msg.CartTrapVelAction)
# cartclient = actionlib.SimpleActionClient('ur_cartesian_action', robot_module.msg.CartLinTaskAction)

# Create some global variables
# xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]

# main
def main():

    # Initialize the ROS node
    rospy.init_node('test_simple_action_state_machine')

    
    # Create a SMACH state machine
    # sm_top = smach.StateMachine("")
    sm_top = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Create the Introspection server
    sis = smach_ros.IntrospectionServer('test_simple_action_state_machine_server', sm_top, '/SM_TOP')
    sis.start()
	
    # Open the container
    with sm_top:
        # Add states to the container

        clt_p_new = np.asarray([0.0, 0.0, 0.0])
        clt_q_new = np.asarray([0.0, 0.0, 0.0, 0.0])
        clt_t_des = 3
        clt_goal = robot_module.msg.CartLinTaskGoal(clt_p_new, clt_q_new, clt_t_des)
        smach.StateMachine.add('CART_LIN_TASK_ACTION_STATE',
                               smach_ros.SimpleActionState('/ur10_1/cart_lin_task_action_server', robot_module.msg.CartLinTaskAction,
                                                           goal = clt_goal),
                               {'succeeded':'CART_TRAP_VEL_ACTION_STATE'})

        ctv_p_new = np.asarray([0.0, 0.0, 0.0])
        ctv_q_new = np.asarray([0.0, 0.0, 0.0, 0.0])
        ctv_t_des = 3
        ctv_goal = robot_module.msg.CartTrapVelGoal(ctv_p_new, ctv_q_new, ctv_t_des)
        smach.StateMachine.add('CART_TRAP_VEL_ACTION_STATE',
                               smach_ros.SimpleActionState('/ur10_1/cart_trap_vel_action_server', robot_module.msg.CartTrapVelAction,
                                                           goal = ctv_goal),
                               {'succeeded':'OPEN_GRIPPER_STATE'})

        open_gripper_request = gpio.OutputRequest(HOUSING_GRIPPER_FINGER_GPIO, HOUSING_GRIPPER_FINGER_OPEN)
        smach.StateMachine.add('OPEN_GRIPPER_STATE',
                               smach_ros.ServiceState('/ur10_1/set_output',
                                                      gpio.Output,
                                                      request = open_gripper_request),
                               {'succeeded':'succeeded'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
