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

# TF
import tf

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
# import gpio.srv as gpio
# gpio_service_1 = None
# gpio_service_2 = None

# Digital output service
from robot_module.srv import Output as DigitalOutput
from robot_module.srv import OutputRequest as DigitalOutputRequest
from robot_module.srv import OutputResponse as DigitalOutputResponse

# Set robot tool service
# from robot_module.srv import SetRobotTool
# set_robot_tool_service_1 = None
# set_robot_tool_service_2 = None

# Set robot tool service
from robot_module.srv import SetRobotTool
from robot_module.srv import SetRobotToolRequest
from robot_module.srv import SetRobotToolResponse

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

HEXAPOD_BRAKE_1_GPIO = 7
HEXAPOD_BRAKE_OPEN = 1
HEXAPOD_BRAKE_CLOSE = 0

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
# Define tf listener state class
class TFListenerState(smach.State):
    def __init__(self, target_frame, source_frame, output_key, time=rospy.Time(0), duration=rospy.Duration(1)):

        self._target_frame = target_frame
        self._source_frame = source_frame
        self._output_key = output_key
        self._time = time
        self._duration = duration

        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=[],
                             output_keys=[self._output_key])

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(self._target_frame, self._source_frame, self._time, self._duration)

    def execute(self, userdata):
        rospy.loginfo('Executing TFListenerState for target_frame ' + self._target_frame + ' and source_frame ' + self._source_frame)
        setattr(userdata, self._output_key, self.tf_listener.lookupTransform(self._target_frame, self._source_frame, self._time))
        return 'succeeded'
          
def cart_trap_vel_goal_cb(userdata, goal):

  #
  # TODO: Fix the rotational offset!
  #
  new_position = userdata.cart_trap_vel_pose_input[0] + userdata.cart_trap_vel_position_offset_input
  new_rotation = userdata.cart_trap_vel_pose_input[1] # + userdata.cart_trap_vel_rotation_offset_input
  
  cart_trap_vel_goal = robot_module.msg.CartTrapVelGoal(new_position, new_rotation, userdata.cart_trap_vel_desired_velocity_input)
  
  return cart_trap_vel_goal


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
        
        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
          
        #----------------------------------------------------------------------------------------
        # BEGIN: RECONFIGURE_HEXAPOD
        # TEMPLATE: DefineBlock
        #
        sis2 = smach_ros.IntrospectionServer('test_simple_action_state_machine_server', sm_sub, '/SM_TOP/RECONFIGURE_HEXAPOD')
        sis2.start()

        with sm_sub:

          #----------------------------------------------------------------------------------------
          # BEGIN: READ_HEXAPOD_CURRENT_POSE
          # TEMPLATE: ReadTransform
          #
          smach.StateMachine.add('READ_HEXAPOD_CURRENT_POSE', TFListenerState('ur10_1/base', 'hexapod_1/top', 'hexapod_current_pose'),
                                                                                  transitions={'succeeded':'MOVE_ABOVE_HEXAPOD_1'},
                                                                                  remapping={'hexapod_current_pose':'hexapod_current_pose'})
          # END: READ_HEXAPOD_CURRENT_POSE
          #----------------------------------------------------------------------------------------

          #----------------------------------------------------------------------------------------
          # BEGIN: MOVE_ABOVE_HEXAPOD_1
          # TEMPLATE: CartTrapVelAction
          #
          sm_sub.userdata.move_above_hexapod_1_position_offset = np.asarray([0.0, 0.0, -0.20])
          sm_sub.userdata.move_above_hexapod_1_rotation_offset = np.asarray([0.0, 0.0, -0.20])
          sm_sub.userdata.move_above_hexapod_1_desired_velocity = 0.1

          smach.StateMachine.add('MOVE_ABOVE_HEXAPOD_1',
                                 smach_ros.SimpleActionState('/ur10_1/cart_trap_vel_action_server', robot_module.msg.CartTrapVelAction,
                                                             goal_cb = cart_trap_vel_goal_cb,
                                                             input_keys=['cart_trap_vel_pose_input',
                                                                         'cart_trap_vel_position_offset_input',
                                                                         'cart_trap_vel_rotation_offset_input',
                                                                         'cart_trap_vel_desired_velocity_input']),
                                 transitions={'succeeded':'OPEN_TOOL_EXCHANGE_1'},
                                 remapping={'cart_trap_vel_pose_input':'hexapod_current_pose',
                                            'cart_trap_vel_position_offset_input':'move_above_hexapod_1_position_offset',
                                            'cart_trap_vel_rotation_offset_input':'move_above_hexapod_1_rotation_offset',
                                            'cart_trap_vel_desired_velocity_input':'move_above_hexapod_1_desired_velocity'})
          # END: MOVE_ABOVE_HEXAPOD_1
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: OPEN_TOOL_EXCHANGE_1
          # TEMPLATE: SetOutput
          #
          open_tool_exchange_request = DigitalOutputRequest(TOOL_EXCHANGE_GPIO, TOOL_EXCHANGE_OPEN)

          smach.StateMachine.add('OPEN_TOOL_EXCHANGE_1',
                                 smach_ros.ServiceState('/ur10_1/set_output',
                                                        DigitalOutput,
                                                        request = open_tool_exchange_request),
                                 transitions={'succeeded':'COUPLE_WITH_HEXAPOD'})
          # END: OPEN_TOOL_EXCHANGE_1
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: COUPLE_WITH_HEXAPOD
          # TEMPLATE: CartLinTaskAction
          #
          clt_p_new = np.asarray([0.0, 0.0, 0.0])
          clt_q_new = np.asarray([0.0, 0.0, 0.0, 0.0])
          clt_t_des = 3
          clt_goal = robot_module.msg.CartLinTaskGoal(clt_p_new, clt_q_new, clt_t_des)

          sm_sub.userdata.couple_with_hexapod_position_offset = np.asarray([0.0, 0.0, -0.20])
          sm_sub.userdata.couple_with_hexapod_rotation_offset = np.asarray([0.0, 0.0, -0.20])
          sm_sub.userdata.couple_with_hexapod_desired_velocity = 0.1


          smach.StateMachine.add('COUPLE_WITH_HEXAPOD',
                                 smach_ros.SimpleActionState('/ur10_1/cart_lin_task_action_server', robot_module.msg.CartLinTaskAction,
                                                             goal = clt_goal),
                                 transitions={'succeeded':'CLOSE_TOOL_EXCHANGE'})
          # END: COUPLE_WITH_HEXAPOD
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: CLOSE_TOOL_EXCHANGE
          # TEMPLATE: SetOutput
          #
          close_tool_exchange_request = DigitalOutputRequest(TOOL_EXCHANGE_GPIO, TOOL_EXCHANGE_CLOSE)

          smach.StateMachine.add('CLOSE_TOOL_EXCHANGE',
                                 smach_ros.ServiceState('/ur10_1/set_output',
                                                        DigitalOutput,
                                                        request = close_tool_exchange_request),
                                 transitions={'succeeded':'RELEASE_HEXAPOD_BRAKES'})
          # END: CLOSE_TOOL_EXCHANGE
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: RELEASE_HEXAPOD_BRAKES
          # TEMPLATE: SetOutput
          #
          release_hexapod_brakes_request = DigitalOutputRequest(HEXAPOD_BRAKE_1_GPIO, HEXAPOD_BRAKE_OPEN)

          smach.StateMachine.add('RELEASE_HEXAPOD_BRAKES',
                                 smach_ros.ServiceState('/ur10_1/set_output',
                                                        DigitalOutput,
                                                        request = close_tool_exchange_request),
                                 transitions={'succeeded':'READ_HEXAPOD_MIDDLE_POSE'})
          
          # END: RELEASE_HEXAPOD_BRAKES
          #----------------------------------------------------------------------------------------
          
          
          #----------------------------------------------------------------------------------------
          # BEGIN: READ_HEXAPOD_MIDDLE_POSE
          # TEMPLATE: ReadTransform
          #
          smach.StateMachine.add('READ_HEXAPOD_MIDDLE_POSE', TFListenerState('ur10_1/base', 'hexapod_1/mid', 'hexapod_middle_position'),
          
          # END: READ_HEXAPOD_MIDDLE_POSE
          #----------------------------------------------------------------------------------------
                                                                                 {'succeeded':'MOVE_TO_MIDPOINT'})
          #----------------------------------------------------------------------------------------
          # BEGIN: MOVE_TO_MIDPOINT
          # TEMPLATE: CartLinTaskAction
          #
          clt_p_new = np.asarray([0.0, 0.0, 0.0])
          clt_q_new = np.asarray([0.0, 0.0, 0.0, 0.0])
          clt_t_des = 3
          clt_goal = robot_module.msg.CartLinTaskGoal(clt_p_new, clt_q_new, clt_t_des)
          smach.StateMachine.add('MOVE_TO_MIDPOINT',
                                 smach_ros.SimpleActionState('/ur10_1/cart_lin_task_action_server', robot_module.msg.CartLinTaskAction,
                                                             goal = clt_goal),
                                 transitions={'succeeded':'READ_HEXAPOD_NEW_POSE'})
          # END: MOVE_TO_MIDPOINT
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: READ_HEXAPOD_NEW_POSE
          # TEMPLATE: ReadTransform
          #
          smach.StateMachine.add('READ_HEXAPOD_NEW_POSE', TFListenerState('ur10_1/base', 'elvez/housingx07/hexapod_1/top', 'hexapod_new_position'),
                                                                              {'succeeded':'MOVE_TO_NEW_POSE'})
          # END: READ_HEXAPOD_NEW_POSE
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: MOVE_TO_NEW_POSE
          # TEMPLATE: CartLinTaskAction
          #
          clt_p_new = np.asarray([0.0, 0.0, 0.0])
          clt_q_new = np.asarray([0.0, 0.0, 0.0, 0.0])
          clt_t_des = 3
          clt_goal = robot_module.msg.CartLinTaskGoal(clt_p_new, clt_q_new, clt_t_des)
          smach.StateMachine.add('MOVE_TO_NEW_POSE',
                                 smach_ros.SimpleActionState('/ur10_1/cart_lin_task_action_server', robot_module.msg.CartLinTaskAction,
                                                             goal = clt_goal),
                                 transitions={'succeeded':'LOCK_HEXAPOD_BRAKES'})
          # END: MOVE_TO_NEW_POSE
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: LOCK_HEXAPOD_BRAKES
          # TEMPLATE: SetOutput
          #
          lock_hexapod_brakes_request = DigitalOutputRequest(HEXAPOD_BRAKE_1_GPIO, HEXAPOD_BRAKE_CLOSE)

          smach.StateMachine.add('LOCK_HEXAPOD_BRAKES',
                                 smach_ros.ServiceState('/ur10_1/set_output',
                                                        DigitalOutput,
                                                        request = close_tool_exchange_request),
                                 transitions={'succeeded':'OPEN_TOOL_EXCHANGE_2'})
          # END: LOCK_HEXAPOD_BRAKES
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: OPEN_TOOL_EXCHANGE_2
          # TEMPLATE: SetOutput
          #
          open_tool_exchange_request = DigitalOutputRequest(TOOL_EXCHANGE_GPIO, TOOL_EXCHANGE_OPEN)

          smach.StateMachine.add('OPEN_TOOL_EXCHANGE_2',
                                 smach_ros.ServiceState('/ur10_1/set_output',
                                                        DigitalOutput,
                                                        request = open_tool_exchange_request),
                                 transitions={'succeeded':'MOVE_ABOVE_HEXAPOD_2'})
          # END: OPEN_TOOL_EXCHANGE_2
          #----------------------------------------------------------------------------------------
          
          #----------------------------------------------------------------------------------------
          # BEGIN: MOVE_ABOVE_HEXAPOD_2
          # TEMPLATE: SetOutput
          #
          clt_p_new = np.asarray([0.0, 0.0, 0.0])
          clt_q_new = np.asarray([0.0, 0.0, 0.0, 0.0])
          clt_t_des = 3
          clt_goal = robot_module.msg.CartLinTaskGoal(clt_p_new, clt_q_new, clt_t_des)
          smach.StateMachine.add('MOVE_ABOVE_HEXAPOD_2',
                                 smach_ros.SimpleActionState('/ur10_1/cart_lin_task_action_server', robot_module.msg.CartLinTaskAction,
                                                             goal = clt_goal),
                                 transitions={'succeeded':'succeeded'})
          # END: MOVE_ABOVE_HEXAPOD_2
          #----------------------------------------------------------------------------------------
          

        smach.StateMachine.add('RECONFIGURE_HEXAPOD', sm_sub,
                             transitions={'succeeded':'succeeded'})
        # END: RECONFIGURE_HEXAPOD
        #----------------------------------------------------------------------------------------

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis2.stop()
    sis.stop()

    rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    main()
