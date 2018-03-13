#!/usr/bin/env python




import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros








class PrintUserdataState(smach.State):
    def __init__(self, input_keys = [], output_keys = [], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

    def execute(self, userdata):

        # Print input keys to terminal
        for input_key in self._input_keys:
            rospy.loginfo('userdata.{}: {}'.format(input_key, userdata[input_key]))
        
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
    rospy.init_node('smacha_print_userdata_test')

    

   

    sm = smach.StateMachine(outcomes=['final_outcome'])


    
    sm.userdata.foo = 'Hello World!'
    
    sm.userdata.bar = 'Goodbye World!'

    with sm:

        smach.StateMachine.add('FOO_0',
                                       PrintUserdataState(input_keys = ['foo']),
                               transitions={'succeeded':'FOO_1'})
        
        smach.StateMachine.add('FOO_1',
                                       CallbacksState(),
                               transitions={'succeeded':'FOO_2'})
        
        smach.StateMachine.add('FOO_2',
                                       PrintUserdataState(input_keys = ['foobar']),
                               transitions={'succeeded':'final_outcome'},
                               remapping={'foobar':'bar'})



        





    

    outcome = sm.execute()
    




    



if __name__ == '__main__':
    main()
