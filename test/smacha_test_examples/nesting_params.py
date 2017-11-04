#!/usr/bin/env python

import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self, name, outcome):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

        self._name = name
        self._outcome = outcome

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self._name))
        rospy.loginfo('Returning {}'.format(self._outcome))

        return self._outcome


def main():
    rospy.init_node('smacha_nesting_params_test')

    sm_top = smach.StateMachine(outcomes=['final_outcome'])

    with sm_top:

        sm_sub = smach.StateMachine(outcomes=['outcome3'])
        
        with sm_sub:
        
            smach.StateMachine.add('FOO_0', Foo('FOO_0', 'outcome1'), 
                                   transitions={'outcome1':'FOO_1',
                                                'outcome2':'outcome3'})
            
            smach.StateMachine.add('FOO_1', Foo('FOO_1', 'outcome2'), 
                                   transitions={'outcome1':'FOO_1',
                                                'outcome2':'outcome3'})
            
        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome3':'final_outcome'})

    outcome = sm_top.execute()

if __name__ == '__main__':
    main()
