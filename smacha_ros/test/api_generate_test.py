#!/usr/bin/env python

import sys
import argparse
import os

import rospy
import rospkg
import rostest

from rosgraph_msgs.msg import Log

from smacha.util import Tester
from smacha.util import bcolors

TEST_SCRIPTS_DIR = 'smacha_scripts/smacha_test_examples/'
ROS_TEMPLATES_DIR = '../src/smacha_ros/templates'
TEMPLATES_DIR = 'smacha_templates/smacha_test_examples'
DEBUG_LEVEL = 1

class TestClass(Tester):
    """Tester class for general unit testing of various SMACHA tool
    functionalities.
    """

    _logmsg = ''
    _logerr = False

    def __init__(self, *args, **kwargs):
        # Set Tester member variables
        self.set_debug_level(DEBUG_LEVEL)

        # Store the base path
        self._base_path = os.path.dirname(os.path.abspath(__file__))

        # Call the parent constructor
        super(TestClass, self).__init__(
            *args,
            script_dirs=[os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples')],
            template_dirs=[
                os.path.join(self._base_path, ROS_TEMPLATES_DIR),
                os.path.join(self._base_path, TEMPLATES_DIR)
                ],
            **kwargs)

        rospy.Subscriber('/rosout', Log, self._std_out_cb)

    def _std_out_cb(self, data):
        
        # Check if there are any errors in the execution 
        # of the state machine that might not throw an
        # exception during the exec().
        msg = data.msg
        level = data.level
        if level >= 8:
            self._logmsg = msg
            self._logerr = True

    def test_code_generation_and_execution(self):
        """Generate Python code from all the available scripts in the test folder
        and run them."""

        success = False
        generated_code = ''
        script_file = ''
        try:
            # Get a list of available files
            script_names = os.listdir(os.path.join(self._base_path, TEST_SCRIPTS_DIR))
            script_names.sort()
            # Remove all subscripts from the test array
            script_names = [ script_name for script_name in script_names if script_name.find('sub_script') == -1]
            # Loop to generate Python code from each YML script
            for script_name in script_names:
                script_file = TEST_SCRIPTS_DIR + script_name
                # Pass the script into the generator and write the result into a variable
                generated_code = self._generate(os.path.join(self._base_path, script_file))
                # We need to take the "init_node" from the generated code as this program here already initializes a node
                generated_code = generated_code.replace(
                "rospy.init_node",
                "# rospy.init_node"
                )

                # Execute the generated python code
                exec(generated_code, globals())
                if self._logerr:
                    raise Exception("Error in state machine execution !!")
            success = True
        except Exception as e:
            rospy.logerr(
                'Failed due to exception when generating [{0}]!!'.format(script_file))
            rospy.logerr('Exception: {0}'.format(e))

        self.assertTrue(success) 

if __name__=="__main__":
    
    rospy.init_node('test_smacha_api_generate',log_level=rospy.DEBUG)
    rostest.rosrun('smacha_ros', 'test_smacha_api_generate', TestClass)
