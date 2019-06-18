#!/usr/bin/env python




import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros








# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        return 'outcome3'

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR') 
        return 'outcome1'









def main():
    rospy.init_node('smach_example_state_machine')

    



    sm_top = smach.StateMachine(outcomes=['outcome6'])




    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3':'CON'})

        sm_con = smach.Concurrence(outcomes=['outcome4', 'outcome5'],
                                           default_outcome='outcome4',
                                           outcome_map={'outcome5': { 'BAR': 'outcome1', 'FOO': 'outcome2'}})




        with sm_con:

            smach.Concurrence.add('FOO', Foo())

            smach.Concurrence.add('BAR', Bar())



        smach.StateMachine.add('CON', sm_con,
                               transitions={'outcome4':'CON',
                                            'outcome5':'outcome6'})



        





    

    outcome = sm_top.execute()





    



if __name__ == '__main__':
    main()