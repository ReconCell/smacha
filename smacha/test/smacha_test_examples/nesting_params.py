#!/usr/bin/env python
import smach


# define state Foo
class Foo(smach.State):
    def __init__(self, name, outcome):
        smach.State.__init__(self, outcomes=['outcome_a','outcome_b'])

        self._name = name
        self._outcome = outcome

    def execute(self, userdata):
        smach.loginfo('Executing state {}'.format(self._name))
        smach.loginfo('Returning {}'.format(self._outcome))

        return self._outcome


def main():

    sm = smach.StateMachine(outcomes=['final_outcome'])

    with sm:

        sm_sub = smach.StateMachine(outcomes=['outcome_c'])

        with sm_sub:

            smach.StateMachine.add('FOO_0', Foo('FOO_0', 'outcome_a'), 
                                   transitions={'outcome_a':'FOO_1',
                                                'outcome_b':'outcome_c'})

            smach.StateMachine.add('FOO_1', Foo('FOO_1', 'outcome_b'), 
                                   transitions={'outcome_a':'FOO_1',
                                                'outcome_b':'outcome_c'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome_c':'final_outcome'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()