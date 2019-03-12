#!/usr/bin/env python

import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros
import random


# Define normal callback for 'animals' output key
@smach.cb_interface(input_keys=['animals'],
                    output_keys=['animals'],
                    outcomes=['succeeded'])
def foo_animals_cb(userdata):
    userdata['animals'].append('turtles')
    return 'succeeded'

# Define normal callback for 'numbers' output key
@smach.cb_interface(input_keys=['numbers'],
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def foo_numbers_cb(userdata):
    userdata['numbers'].append(userdata['numbers'][-1]+1)
    return 'succeeded'

@smach.cb_interface(input_keys=['animals'], 
                    output_keys=['animals'],
                    outcomes=['succeeded'])
def animals_foo_1_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.animals if ud.animals.append('ducks') else ud.animals
    userdata.animals = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def numbers_foo_3_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(ud.numbers[-1]+1) else ud.numbers
    userdata.numbers = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['animals', 'numbers'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def numbers_foo_4_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(ud.numbers[-1]+1) else ud.numbers
    userdata.numbers = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['animals', 'numbers'], 
                    output_keys=['animals'],
                    outcomes=['succeeded'])
def animals_foo_5_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.animals if ud.animals.append('ducks') else ud.animals
    userdata.animals = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=[], 
                    output_keys=['random_number'],
                    outcomes=['succeeded'])
def random_number_foo_6_lambda_cb(userdata):
    lambda_cb = lambda ud: random.random()
    userdata.random_number = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers', 'random_number'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def numbers_foo_7_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(ud.random_number) else ud.numbers
    userdata.numbers = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers', 'number'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def numbers_foo_8_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(ud.number) else ud.numbers
    userdata.numbers = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers', 'a_random_number_1', 'a_random_number_2', 'b_random_number_sum'], 
                    output_keys=['a_random_number_1'],
                    outcomes=['succeeded'])
def a_random_number_1_foo_9_lambda_cb(userdata):
    lambda_cb = lambda ud: random.random()
    userdata.a_random_number_1 = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers', 'a_random_number_1', 'a_random_number_2', 'b_random_number_sum'], 
                    output_keys=['a_random_number_2'],
                    outcomes=['succeeded'])
def a_random_number_2_foo_9_lambda_cb(userdata):
    lambda_cb = lambda ud: random.random()
    userdata.a_random_number_2 = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers', 'a_random_number_1', 'a_random_number_2', 'b_random_number_sum'], 
                    output_keys=['b_random_number_sum'],
                    outcomes=['succeeded'])
def b_random_number_sum_foo_9_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.a_random_number_1 + ud.a_random_number_2
    userdata.b_random_number_sum = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers', 'a_random_number_1', 'a_random_number_2', 'b_random_number_sum'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def numbers_foo_9_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(ud.b_random_number_sum) else ud.numbers
    userdata.numbers = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def numbers_foo_10_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(42) else ud.numbers
    userdata.numbers = lambda_cb(userdata)
    return 'succeeded'


class Foo(smach.State):
    def __init__(self, name, input_keys=[], output_keys=[], callbacks=[]):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        self._name = name

        self._cbs = []

        if callbacks:
            for cb in sorted(callbacks):
                if cb in globals():
                    self._cbs.append(globals()[cb])

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
        for input_key in self._input_keys:
            rospy.loginfo('Userdata input key \'{}\' BEFORE callback execution: {}'.format(input_key, userdata[input_key]))

        # Call callbacks
        for (cb, ik, ok) in zip(self._cbs,
                                self._cb_input_keys,
                                self._cb_output_keys):

            # Call callback with limited userdata
            cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))


        for input_key in self._input_keys:
            rospy.loginfo('Userdata input key \'{}\' AFTER callback execution: {}'.format(input_key, userdata[input_key]))

        return 'succeeded'


class CallbacksState(smach.State):
    def __init__(self, input_keys = [], output_keys = [], callbacks = []):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        self._cbs = []

        if callbacks:
            for cb in sorted(callbacks):
                if cb in globals():
                    self._cbs.append(globals()[cb])

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
            cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))

        return 'succeeded'





def main():
    rospy.init_node('smacha_callbacks_test')

    sm = smach.StateMachine(outcomes=['final_outcome'])


    sm.userdata.animals = ['cats', 'dogs', 'sharks']
    sm.userdata.numbers = [1, 2, 3]
    sm.userdata.number = 123
    sm.userdata.a_random_number_1 = 0
    sm.userdata.a_random_number_2 = 0
    sm.userdata.b_random_number_sum = 0

    with sm:

        smach.StateMachine.add('FOO_0', Foo('FOO_0', input_keys = ['animals'], output_keys = ['animals'], callbacks = ['foo_animals_cb']),
                               transitions={'succeeded':'FOO_1'})

        smach.StateMachine.add('FOO_1',
                                       CallbacksState(input_keys = ['animals'], output_keys = ['animals'], callbacks = ['animals_foo_1_lambda_cb']),
                               transitions={'succeeded':'FOO_2'})

        smach.StateMachine.add('FOO_2', Foo('FOO_2', input_keys = ['numbers'], output_keys = ['numbers'], callbacks = ['foo_numbers_cb']),
                               transitions={'succeeded':'FOO_3'})

        smach.StateMachine.add('FOO_3',
                                       CallbacksState(input_keys = ['numbers'], output_keys = ['numbers'], callbacks = ['numbers_foo_3_lambda_cb']),
                               transitions={'succeeded':'FOO_4'})

        smach.StateMachine.add('FOO_4', Foo('FOO_4', input_keys = ['animals', 'numbers'], output_keys = ['animals', 'numbers'], callbacks = ['foo_animals_cb', 'numbers_foo_4_lambda_cb']),
                               transitions={'succeeded':'FOO_5'})

        smach.StateMachine.add('FOO_5', Foo('FOO_5', input_keys = ['animals', 'numbers'], output_keys = ['animals', 'numbers'], callbacks = ['animals_foo_5_lambda_cb', 'foo_numbers_cb']),
                               transitions={'succeeded':'FOO_6'})

        smach.StateMachine.add('FOO_6',
                                       CallbacksState(output_keys = ['random_number'], callbacks = ['random_number_foo_6_lambda_cb']),
                               transitions={'succeeded':'FOO_7'})

        smach.StateMachine.add('FOO_7', Foo('FOO_7', input_keys = ['numbers', 'random_number'], output_keys = ['numbers'], callbacks = ['numbers_foo_7_lambda_cb']),
                               transitions={'succeeded':'FOO_8'})

        smach.StateMachine.add('FOO_8', Foo('FOO_8', input_keys = ['numbers', 'number'], output_keys = ['numbers'], callbacks = ['numbers_foo_8_lambda_cb']),
                               transitions={'succeeded':'FOO_9'})

        smach.StateMachine.add('FOO_9', Foo('FOO_9', input_keys = ['numbers', 'a_random_number_1', 'a_random_number_2', 'b_random_number_sum'], output_keys = ['numbers', 'a_random_number_1', 'a_random_number_2', 'b_random_number_sum'], callbacks = ['a_random_number_1_foo_9_lambda_cb', 'a_random_number_2_foo_9_lambda_cb', 'b_random_number_sum_foo_9_lambda_cb', 'numbers_foo_9_lambda_cb']),
                               transitions={'succeeded':'FOO_10'})

        smach.StateMachine.add('FOO_10',
                                       CallbacksState(input_keys = ['numbers'], output_keys = ['numbers'], callbacks = ['numbers_foo_10_lambda_cb']),
                               transitions={'succeeded':'final_outcome'})

    outcome = sm.execute()


if __name__ == '__main__':
    main()
