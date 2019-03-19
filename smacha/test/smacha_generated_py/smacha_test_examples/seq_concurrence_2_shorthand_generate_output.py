#!/usr/bin/env python




import roslib; roslib.load_manifest('smacha')
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
    rospy.init_node('smacha_params_test')

    



    sm_top = smach.StateMachine(outcomes=['final_outcome_a', 'final_outcome_b', 'final_outcome_c'])




    with sm_top:

        smach.StateMachine.add('FOO_0', Foo('FOO_0', 'outcome_a'), 
                               transitions={'outcome_a':'CON',
                                            'outcome_b':'final_outcome_b'})

        sm_con = smach.Concurrence(outcomes=['con_outcome_1', 'con_outcome_2', 'con_outcome_3', 'con_default_outcome'],
                                           default_outcome='con_default_outcome',
                                           outcome_map={'con_outcome_1': { 'FOO_1': 'outcome_b'},
                                                        'con_outcome_2': { 'FOO_1': 'outcome_a', 'FOO_2': 'outcome_a'},
                                                        'con_outcome_3': { 'FOO_2': 'outcome_b'}})




        with sm_con:

            smach.Concurrence.add('FOO_1', Foo('FOO_1', 'outcome_a'))

            smach.Concurrence.add('FOO_2', Foo('FOO_2', 'outcome_a'))



        smach.StateMachine.add('CON', sm_con,
                               transitions={'con_default_outcome':'CON',
                                            'con_outcome_1':'final_outcome_b',
                                            'con_outcome_2':'final_outcome_a',
                                            'con_outcome_3':'final_outcome_c'})



        





    

    outcome = sm_top.execute()





    



if __name__ == '__main__':
    main()