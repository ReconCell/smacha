#----------------------------------------------------------------------------------------
# BEGIN: READ_HEXAPOD_CURRENT_POSE
# TEMPLATE: ReadTransformState
#
smach.StateMachine.add('READ_HEXAPOD_CURRENT_POSE', TFListenerState('ur10_1/base', 'hexapod_1/top', 'hexapod_current_pose'),
                                                                        transitions={'succeeded':'MOVE_ABOVE_HEXAPOD_1'},
                                                                        remapping={'hexapod_current_pose':'hexapod_current_pose'})
# END: READ_HEXAPOD_CURRENT_POSE
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
# BEGIN: MOVE_ABOVE_HEXAPOD_1
# TEMPLATE: CartTrapVelActionState
#
sm_sub.userdata.MOVE_ABOVE_HEXAPOD_1_position_offset = np.asarray([0.0, 0.0, -0.2])
sm_sub.userdata.MOVE_ABOVE_HEXAPOD_1_rotation_offset = np.asarray([0.0, 0.0, 0.0])
sm_sub.userdata.MOVE_ABOVE_HEXAPOD_1_desired_velocity = 0.1

smach.StateMachine.add('MOVE_ABOVE_HEXAPOD_1',
                       smach_ros.SimpleActionState('/ur10_1/cart_trap_vel_action_server', robot_module.msg.CartTrapVelAction,
                                                   goal_cb = cart_trap_vel_goal_cb,
                                                   input_keys=['cart_trap_vel_pose_input',
                                                               'cart_trap_vel_position_offset_input',
                                                               'cart_trap_vel_rotation_offset_input',
                                                               'cart_trap_vel_desired_velocity_input']),
                       transitions={'succeeded':'OPEN_TOOL_EXCHANGE_1'},
                       remapping={'cart_trap_vel_pose_input':'hexapod_current_pose',
                                  'cart_trap_vel_position_offset_input':'MOVE_ABOVE_HEXAPOD_1_position_offset',
                                  'cart_trap_vel_rotation_offset_input':'MOVE_ABOVE_HEXAPOD_1_rotation_offset',
                                  'cart_trap_vel_desired_velocity_input':'MOVE_ABOVE_HEXAPOD_1_desired_velocity'})
# END: MOVE_ABOVE_HEXAPOD_1
#----------------------------------------------------------------------------------------

