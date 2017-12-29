#!/usr/bin/env python

import rospy
import rospkg
import rostest
import unittest
import argparse
import os
import re
import difflib
import yaml as pyyaml
import stat

import smacha
from smacha_ros.srv import *
from smacha.util import Tester

class TestServices(Tester):
    
    write_output_files = False
    debug_level = 1
    
    def __init__(self, *args, **kwargs):
        super(TestServices, self).__init__(*args, **kwargs)

        # Use the SMACHA parser for loading YAML scripts
        self._parser = smacha.Parser()

        # Get an instance of RosPack with the default search paths
        self._rospack = rospkg.RosPack()
        

    def _generate(self, smacha_script_filename):
        # Load the SMACHA YAML script
        try:
            # Load the script
            script_str, _ = self._parser.load(smacha_script_filename)

            # Parse the script
            #
            # NOTE: We must use PyYAML instead of ruamel.yaml to parse here in order to successfully
            # interact with the ROS param server.
            script = pyyaml.load(script_str)
        except Exception as e:
            raise IOError('Error when loading {} SMACHA YAML script: {}'.format(smacha_script_filename, str(e)))

        # Name the script param
        script_param = 'smacha/scripts/' + os.path.splitext(os.path.basename(smacha_script_filename))[0]

        # Load the script onto the param server
        try:
            rospy.set_param(script_param, script)
        except Exception as e:
            raise rospy.ROSException('Error when loading {} SMACHA YAML script into param server: {}'.format(smacha_script_filename, str(e)))

        # Call the SMACHA generate service to generate the code
        rospy.wait_for_service('smacha/generate')
        try:
            generate = rospy.ServiceProxy('smacha/generate', Generate)
            generate_response = generate(script_param)
            smach_code = generate_response.code
        except rospy.ServiceException as e:
            print('Service call failed: {}'.format(str(e)))
        
        # Write the final output to a SMACH python file
        if self.write_output_files:
            with open(smacha_script_filename + '.py', 'w') as smach_file:
                smach_file.write(smach_code)
            os.chmod(smacha_script_filename + '.py',
                        stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR |
                        stat.S_IRGRP | stat.S_IXGRP)

        return smach_code
    
    def test_generate_hard_coded_params(self):
        """Test generating hard_coded_params.yml"""
        smacha_path = self._rospack.get_path('smacha')
        with open(os.path.join(smacha_path, 'test/smacha_test_examples/params.py')) as original_file:
            generated_code = self._generate(os.path.join(smacha_path, 'test/smacha_scripts/smacha_test_examples/hard_coded_params.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_generate_assigned_params(self):
        """Test generating assigned_params.yml"""
        smacha_path = self._rospack.get_path('smacha')
        with open(os.path.join(smacha_path, 'test/smacha_test_examples/params.py')) as original_file:
            generated_code = self._generate(os.path.join(smacha_path, 'test/smacha_scripts/smacha_test_examples/assigned_params.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_generate_nesting_params(self):
        """Test generating nesting_params.yml"""
        smacha_path = self._rospack.get_path('smacha')
        with open(os.path.join(smacha_path, 'test/smacha_test_examples/nesting_params.py')) as original_file:
            generated_code = self._generate(os.path.join(smacha_path, 'test/smacha_scripts/smacha_test_examples/nesting_params.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_generate_nesting_params_with_sub_script(self):
        """Test generating nesting_params_with_sub_script.yml"""
        smacha_path = self._rospack.get_path('smacha')
        with open(os.path.join(smacha_path, 'test/smacha_test_examples/nesting_params.py')) as original_file:
            generated_code = self._generate(os.path.join(smacha_path, 'test/smacha_scripts/smacha_test_examples/nesting_params_with_sub_script.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))


if __name__=="__main__":
    
    rospy.init_node('test_smacha_ros_generate',log_level=rospy.DEBUG)
    rostest.rosrun('smacha_ros', 'test_smacha_ros_generate', TestServices)
