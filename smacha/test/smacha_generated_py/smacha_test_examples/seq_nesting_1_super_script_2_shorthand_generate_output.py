#!/usr/bin/env python




import roslib
import rospy
import smach
import smach_ros








# define state Foo
class Foo(smach.State):
    def __init__(self, name, outcome):
        smach.State.__init__(self, outcomes=['outcome_a','outcome_b'])

        self._name = name
        self._outcome = outcome

    def execute(self, userdata):
        rospy.loginfo('Executing state {}'.format(self._name))
        rospy.loginfo('Returning {}'.format(self._outcome))

        return self._outcome









def main():
    rospy.init_node('sm')

    



    sm = smach.StateMachine(outcomes=['final_outcome_a', 'final_outcome_b', 'final_outcome_c'])




    with sm:

        sm_sub_0 = smach.StateMachine(outcomes=['sub_outcome_1', 'sub_outcome_2'])




        with sm_sub_0:

            smach.StateMachine.add('FOO_0', Foo('SUB_0_FOO_0', 'outcome_a'), 
                                   transitions={'outcome_a':'FOO_1',
                                                'outcome_b':'sub_outcome_1'})

            smach.StateMachine.add('FOO_1', Foo('SUB_0_FOO_1', 'outcome_a'), 
                                   transitions={'outcome_a':'sub_outcome_2',
                                                'outcome_b':'sub_outcome_1'})



        smach.StateMachine.add('SUB_0', sm_sub_0,
                               transitions={'sub_outcome_1':'final_outcome_b',
                                            'sub_outcome_2':'FOO_2'})

        sm_sub_1 = smach.StateMachine(outcomes=['sub_outcome_1', 'sub_outcome_2'])




        with sm_sub_1:

            smach.StateMachine.add('FOO_0', Foo('SUB_1_FOO_0', 'outcome_a'), 
                                   transitions={'outcome_a':'FOO_1',
                                                'outcome_b':'sub_outcome_1'})

            smach.StateMachine.add('FOO_1', Foo('SUB_1_FOO_1', 'outcome_a'), 
                                   transitions={'outcome_a':'sub_outcome_2',
                                                'outcome_b':'sub_outcome_1'})



        smach.StateMachine.add('SUB_1', sm_sub_1,
                               transitions={'sub_outcome_1':'final_outcome_b',
                                            'sub_outcome_2':'FOO_2'})



        





    

    outcome = sm.execute()





    



if __name__ == '__main__':
    main()