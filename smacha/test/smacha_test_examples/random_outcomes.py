#!/usr/bin/env python


import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros
import random


class RandomOutcomeState(smach.State):
    def __init__(self, input_keys = ['outcome'], output_keys = ['outcome'], callbacks = {}, outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

        self._cbs = []

        if callbacks:
            for cb in sorted(callbacks):
                if cb in globals():
                    self._cbs.append(globals()[cb])
                elif cb in locals():
                    self._cbs.append(locals()[cb])
                elif cb in dir(self):
                    self._cbs.append(getattr(self, cb))

        self._cb_input_keys = []
        self._cb_output_keys = []
        self._cb_outcomes = []

        for cb in self._cbs:
            if cb and smach.has_smach_interface(cb):
                self._cb_input_keys.append(cb.get_registered_input_keys())
                self._cb_output_keys.append(cb.get_registered_output_keys())
                self._cb_outcomes.append(cb.get_registered_outcomes())

                self.register_input_keys(self._cb_input_keys[-1])
                self.register_output_keys(self._cb_output_keys[-1])
                self.register_outcomes(self._cb_outcomes[-1])


    def execute(self, userdata):

        # Call callbacks
        for (cb, ik, ok) in zip(self._cbs,
                                self._cb_input_keys,
                                self._cb_output_keys):

            # Call callback with limited userdata
            try:
                cb_outcome = cb(self, smach.Remapper(userdata,ik,ok,{}))
            except:
                cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))


        return userdata.outcome

class CallbacksState(smach.State):
    def __init__(self, input_keys=[], output_keys=[], callbacks=[]):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        self._cbs = []

        if callbacks:
            for cb in sorted(callbacks):
                if cb in globals():
                    self._cbs.append(globals()[cb])
                elif cb in locals():
                    self._cbs.append(locals()[cb])
                elif cb in dir(self):
                    self._cbs.append(getattr(self, cb))

        self._cb_input_keys = []
        self._cb_output_keys = []
        self._cb_outcomes = []

        for cb in self._cbs:
            if cb and smach.has_smach_interface(cb):
                self._cb_input_keys.append(cb.get_registered_input_keys())
                self._cb_output_keys.append(cb.get_registered_output_keys())
                self._cb_outcomes.append(cb.get_registered_outcomes())

                self.register_input_keys(self._cb_input_keys[-1])
                self.register_output_keys(self._cb_output_keys[-1])
                self.register_outcomes(self._cb_outcomes[-1])


    def execute(self, userdata):

        # Call callbacks
        for (cb, ik, ok) in zip(self._cbs,
                                self._cb_input_keys,
                                self._cb_output_keys):

            # Call callback with limited userdata
            try:
                cb_outcome = cb(self, smach.Remapper(userdata,ik,ok,{}))
            except:
                cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))


        return 'succeeded'


@smach.cb_interface(input_keys=[], 
                    output_keys=['outcome'],
                    outcomes=['foo_0', 'foo_1', 'foo_2'])
def outcome_randomize_lambda_cb(self, userdata):
    lambda_cb = lambda ud: random.choice(list(self._outcomes))
    userdata.outcome = lambda_cb(userdata)
    return 'succeeded'

RandomOutcomeState.outcome_randomize_lambda_cb = outcome_randomize_lambda_cb

@smach.cb_interface(input_keys=[], 
                    output_keys=['outcome'],
                    outcomes=[])
def outcome_foo_0_lambda_cb(self, userdata):
    lambda_cb = lambda ud: random.choice(list(self._outcomes))
    userdata.outcome = lambda_cb(userdata)
    return 'succeeded'

CallbacksState.outcome_foo_0_lambda_cb = outcome_foo_0_lambda_cb

@smach.cb_interface(input_keys=[], 
                    output_keys=['outcome'],
                    outcomes=[])
def outcome_foo_1_lambda_cb(self, userdata):
    lambda_cb = lambda ud: random.choice(list(self._outcomes))
    userdata.outcome = lambda_cb(userdata)
    return 'succeeded'

CallbacksState.outcome_foo_1_lambda_cb = outcome_foo_1_lambda_cb

@smach.cb_interface(input_keys=[], 
                    output_keys=['outcome'],
                    outcomes=[])
def outcome_foo_2_lambda_cb(self, userdata):
    lambda_cb = lambda ud: random.choice(list(self._outcomes))
    userdata.outcome = lambda_cb(userdata)
    return 'succeeded'

CallbacksState.outcome_foo_2_lambda_cb = outcome_foo_2_lambda_cb


def main():
    rospy.init_node('smacha_random_outcomes_test')

    sm = smach.StateMachine(outcomes=['final_outcome'])

    with sm:

        smach.StateMachine.add('RANDOMIZE',
                                       RandomOutcomeState(callbacks = ['outcome_randomize_lambda_cb'], outcomes=['foo_0', 'foo_1', 'foo_2']),
                               transitions={'foo_0':'FOO_0',
                                            'foo_1':'FOO_1',
                                            'foo_2':'FOO_2'})

        smach.StateMachine.add('FOO_0',
                                       CallbacksState(callbacks = ['outcome_foo_0_lambda_cb']),
                               transitions={'succeeded':'RANDOMIZE'})

        smach.StateMachine.add('FOO_1',
                                       CallbacksState(callbacks = ['outcome_foo_1_lambda_cb']),
                               transitions={'succeeded':'RANDOMIZE'})

        smach.StateMachine.add('FOO_2',
                                       CallbacksState(callbacks = ['outcome_foo_2_lambda_cb']),
                               transitions={'succeeded':'final_outcome'})

    outcome = sm.execute()


if __name__ == '__main__':
    main()
