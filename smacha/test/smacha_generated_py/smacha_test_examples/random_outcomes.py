#!/usr/bin/env python




import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros
import random







class RandomOutcomeState(smach.State):
    def __init__(self, input_keys = [], output_keys = [], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

        
        self._cbs = []
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


        # Select random outcome
        random_outcome = random.choice(list(self._outcomes))
        
        return random_outcome

class CallbacksState(smach.State):
    def __init__(self, input_keys = [], output_keys = [], callbacks = []):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        
        self._cbs = []
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
    rospy.init_node('smacha_random_outcomes_test')

    

   

    sm = smach.StateMachine(outcomes=['final_outcome'])




    with sm:

        smach.StateMachine.add('RANDOMIZE',
                                       RandomOutcomeState(outcomes=['foo_0', 'foo_1', 'foo_2']),
                               transitions={'foo_0':'FOO_0',
                                            'foo_1':'FOO_1',
                                            'foo_2':'FOO_2'})
        
        smach.StateMachine.add('FOO_0',
                                       CallbacksState(),
                               transitions={'succeeded':'RANDOMIZE'})
        
        smach.StateMachine.add('FOO_1',
                                       CallbacksState(),
                               transitions={'succeeded':'RANDOMIZE'})
        
        smach.StateMachine.add('FOO_2',
                                       CallbacksState(),
                               transitions={'succeeded':'final_outcome'})



        





    

    outcome = sm.execute()
    




    



if __name__ == '__main__':
    main()