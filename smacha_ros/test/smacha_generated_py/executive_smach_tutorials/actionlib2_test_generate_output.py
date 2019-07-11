#!/usr/bin/env python





import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros
from actionlib import *

from smacha_ros.msg import TestGoal

from smacha_ros.msg import TestAction
from actionlib_msgs.msg import *










# Create a trivial action server
class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                TestAction,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
        elif msg.goal == 1:
            self._sas.set_aborted()
        elif msg.goal == 2:
            self._sas.set_preempted()










def main():
    rospy.init_node('smach_example_actionlib')

    

    # Start an action server
    server = TestServer('test_action')



    sm0 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])




    with sm0:

        smach.StateMachine.add('GOAL_DEFAULT',
                               smach_ros.SimpleActionState('test_action', TestAction),
                               transitions={'succeeded':'GOAL_STATIC'})

        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.



        smach.StateMachine.add('GOAL_STATIC',
                               smach_ros.SimpleActionState('test_action', TestAction,
                                                           goal = TestGoal(goal=1)),
                               transitions={'aborted':'GOAL_CB'})

        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        def goal_callback(userdata, default_goal):
            goal = TestGoal()
            goal.goal = 2
            return goal



        smach.StateMachine.add('GOAL_CB',
                               smach_ros.SimpleActionState('test_action', TestAction,
                                                           goal_cb = goal_callback),
                               transitions={'aborted':'succeeded'})



        





    

    outcome = sm0.execute()






    

    rospy.signal_shutdown('All done.')



if __name__ == '__main__':
    main()