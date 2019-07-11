#!/usr/bin/env python





import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros


from smacha_ros.srv import GripperSrv
from smacha_ros.srv import GripperSrvRequest
from smacha_ros.srv import GripperSrvResponse
from geometry_msgs.msg import *






def gripper_srv(req):
    if req.max_effort > 5.0:
        print('gripper_srv() returning True')
        return GripperSrvResponse(True)
    else:
        print('gripper_srv() returning False')
        return GripperSrvResponse(False)














def main():
    rospy.init_node('smach_example_actionlib_service_state')

    

    # Register a gripper service
    s = rospy.Service('gripper_srv', GripperSrv, gripper_srv)



    sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])


    sm0.userdata.max_effort = 9.0

    sm0.userdata.position = Point()

    sm0.userdata.gripper_input = 9.0

    with sm0:

        smach.StateMachine.add('TRIGGER_GRIPPER_EMPTY_REQUEST',
                               smach_ros.ServiceState('gripper_srv', GripperSrv),
                               transitions={'succeeded':'TRIGGER_GRIPPER_FIXED_REQUEST'})

        smach.StateMachine.add('TRIGGER_GRIPPER_FIXED_REQUEST',
                               smach_ros.ServiceState('gripper_srv', GripperSrv,
                                                           request = GripperSrvRequest(4.0, Point())),
                               transitions={'succeeded':'TRIGGER_GRIPPER_USER_DATA_REQUEST'})

        smach.StateMachine.add('TRIGGER_GRIPPER_USER_DATA_REQUEST',
                               smach_ros.ServiceState('gripper_srv', GripperSrv,
                                                           request_slots = ['max_effort', 'position']),
                               transitions={'succeeded':'TRIGGER_GRIPPER_REQUEST_CALLBACK'})

        @smach.cb_interface(input_keys=['gripper_input'])
        def gripper_request_cb(userdata, request):
           gripper_request = GripperSrvRequest()
           gripper_request.position.x = 2.0
           gripper_request.max_effort = userdata.gripper_input
           return gripper_request

        smach.StateMachine.add('TRIGGER_GRIPPER_REQUEST_CALLBACK',
                               smach_ros.ServiceState('gripper_srv', GripperSrv,
                                                           request_cb = gripper_request_cb,
                                             input_keys = ['gripper_input']),
                               transitions={'succeeded':'succeeded'})



        





    

    outcome = sm0.execute()






    

    rospy.signal_shutdown('All done.')



if __name__ == '__main__':
    main()