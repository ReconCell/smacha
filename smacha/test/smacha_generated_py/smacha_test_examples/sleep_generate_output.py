#!/usr/bin/env python




import roslib; roslib.load_manifest('smacha')
import rospy
import smach
import smach_ros








class SleepState(smach.State):
    def __init__(self, time, input_keys = [], output_keys = [], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

        self._time = time

    def execute(self, userdata):

        rospy.sleep(self._time)

        return 'succeeded'









def main():
    rospy.init_node('smacha_sleep_test')

    



    sm = smach.StateMachine(outcomes=['final_outcome'])




    with sm:

        smach.StateMachine.add('FOO_0',
                                       SleepState(5),
                               transitions={'succeeded':'FOO_1'})

        smach.StateMachine.add('FOO_1',
                                       SleepState(10),
                               transitions={'succeeded':'final_outcome'})



        





    

    outcome = sm.execute()





    



if __name__ == '__main__':
    main()