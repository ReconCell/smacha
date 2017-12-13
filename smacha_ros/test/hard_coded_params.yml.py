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

    

   

    sm_top = smach.StateMachine(outcomes=['final_outcome'])




    with sm_top:

        smach.StateMachine.add('FOO_0', Foo('FOO_0', 'outcome_a'), 
                               transitions={'outcome_a':'FOO_1',
                                            'outcome_b':'final_outcome'})
        
        smach.StateMachine.add('FOO_1', Foo('FOO_1', 'outcome_b'), 
                               transitions={'outcome_a':'FOO_1',
                                            'outcome_b':'final_outcome'})



        



    sis = smach_ros.IntrospectionServer('smacha_params_test', sm_top, '/sm_top')
    sis.start()



    

    outcome = sm_top.execute()
    

   
    rospy.spin()



    



if __name__ == '__main__':
    main()