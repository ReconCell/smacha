#!/usr/bin/env python




import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros




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
def foo_1_animals_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.animals if ud.animals.append('ducks') else ud.animals
    userdata['animals'] = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['numbers'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def foo_3_numbers_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(ud.numbers[-1]+1) else ud.numbers
    userdata['numbers'] = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['animals', 'numbers'], 
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def foo_4_numbers_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.numbers if ud.numbers.append(ud.numbers[-1]+1) else ud.numbers
    userdata['numbers'] = lambda_cb(userdata)
    return 'succeeded'

@smach.cb_interface(input_keys=['animals', 'numbers'], 
                    output_keys=['animals'],
                    outcomes=['succeeded'])
def foo_5_animals_lambda_cb(userdata):
    lambda_cb = lambda ud: ud.animals if ud.animals.append('ducks') else ud.animals
    userdata['animals'] = lambda_cb(userdata)
    return 'succeeded'



class Foo(smach.State):
    def __init__(self, name, input_keys=None, output_keys=None, callbacks=None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        self._name = name
        
        self._cbs = []
        for cb in callbacks:
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
            rospy.loginfo('userdata[\'{}\'] BEFORE callback execution: {}'.format(input_key, userdata[input_key]))

        # Call callbacks
        for (cb, ik, ok) in zip(self._cbs,
                                self._cb_input_keys,
                                self._cb_output_keys):

            # Call callback with limited userdata
            cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))

        for output_key in self._output_keys:
            rospy.loginfo('userdata[\'{}\'] AFTER callback execution: {}'.format(output_key, userdata[output_key]))

        return 'succeeded'





def main():
    rospy.init_node('smacha_callbacks_test')

    

   

    sm = smach.StateMachine(outcomes=['final_outcome'])



    sm.userdata.animals = ['cats', 'dogs', 'sharks']

    sm.userdata.numbers = [1, 2, 3]




    with sm:

        smach.StateMachine.add('FOO_0', Foo('FOO_0', input_keys = ['animals'], output_keys = ['animals'], callbacks = ['foo_animals_cb']), 
                               transitions={'succeeded':'FOO_1'})

        smach.StateMachine.add('FOO_1', Foo('FOO_1', input_keys = ['animals'], output_keys = ['animals'], callbacks = ['foo_1_animals_lambda_cb']), 
                               transitions={'succeeded':'FOO_2'})

        smach.StateMachine.add('FOO_2', Foo('FOO_2', input_keys = ['numbers'], output_keys = ['numbers'], callbacks = ['foo_numbers_cb']), 
                               transitions={'succeeded':'FOO_3'})

        smach.StateMachine.add('FOO_3', Foo('FOO_3', input_keys = ['numbers'], output_keys = ['numbers'], callbacks = ['foo_3_numbers_lambda_cb']), 
                               transitions={'succeeded':'FOO_4'})

        smach.StateMachine.add('FOO_4', Foo('FOO_4', input_keys = ['animals', 'numbers'], output_keys = ['animals', 'numbers'], callbacks = ['foo_animals_cb', 'foo_4_numbers_lambda_cb']), 
                               transitions={'succeeded':'FOO_5'})

        smach.StateMachine.add('FOO_5', Foo('FOO_5', input_keys = ['animals', 'numbers'], output_keys = ['animals', 'numbers'], callbacks = ['foo_5_animals_lambda_cb', 'foo_numbers_cb']), 
                               transitions={'succeeded':'final_outcome'})



        





    

    outcome = sm.execute()
    




    



if __name__ == '__main__':
    main()