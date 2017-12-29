#!/usr/bin/env python

import sys
import argparse
import os
import unittest

from smacha.util import Tester

class TestGenerator(Tester):

    def test_state_machine2(self):
        """Test state_machine2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'executive_smach_tutorials/smach_tutorials/examples/state_machine2.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/executive_smach_tutorials/state_machine2.yml'),
                                            [],
                                            [os.path.join(base_path, 'smacha_templates/executive_smach_tutorials')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_user_data2(self):
        """Test user_data2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'executive_smach_tutorials/smach_tutorials/examples/user_data2.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/executive_smach_tutorials/user_data2.yml'),
                                            [],
                                            [os.path.join(base_path, 'smacha_templates/executive_smach_tutorials')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_state_machine_nesting2(self):
        """Test state_machine_nesting2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'executive_smach_tutorials/smach_tutorials/examples/state_machine_nesting2.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml'),
                                            [],
                                            [os.path.join(base_path, 'smacha_templates/executive_smach_tutorials')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_concurrence2(self):
        """Test concurrence2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'executive_smach_tutorials/smach_tutorials/examples/concurrence2.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/executive_smach_tutorials/concurrence2.yml'),
                                            [],
                                            [os.path.join(base_path, 'smacha_templates/executive_smach_tutorials')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_actionlib2_test(self):
        """Test actionlib2_test.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'executive_smach_tutorials/smach_tutorials/examples/actionlib2_test.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/executive_smach_tutorials/actionlib2_test.yml'),
                                            [],
                                            [os.path.join(base_path, 'smacha_templates/executive_smach_tutorials')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_service_test(self):
        """Test service_test.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'executive_smach_tutorials/smach_tutorials/examples/service_test.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/executive_smach_tutorials/service_test.yml'),
                                            [],
                                            [os.path.join(base_path, 'smacha_templates/executive_smach_tutorials')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))


if __name__=="__main__":
    
    # Parse arguments
    arg_parser = argparse.ArgumentParser(description='SMACHA executive_smach_tutorials unit tests.')
    
    arg_parser.add_argument('-w', '--write',
                            action='store_true',
                            help='Write generated output files to disk.')
    
    arg_parser.add_argument('-op', '--output-py-dir',
                            dest='output_py_dir',
                            default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'smacha_generated_py/executive_smach_tutorials/'),
                            action='store',
                            help='Specify output SMACH Python executable directory.')
    
    arg_parser.add_argument('-oy', '--output-yml-dir',
                            dest='output_yml_dir',
                            default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'smacha_generated_scripts/executive_smach_tutorials/'),
                            action='store',
                            help='Specify output SMACHA YAML script directory.')
    
    arg_parser.add_argument('-d', '--debug-level',
                            dest='debug_level',
                            action='store',
                            default=1,
                            help='Set debug level (0-2. Default: 1)')
    
    # Some trickery to avoid disrupting unittest with args
    if len(sys.argv) > 1:
        argv = sys.argv[1:]
        sys.argv = sys.argv[:1]
        args = arg_parser.parse_args(argv)
        TestGenerator.write_output_files = args.write
        TestGenerator.output_py_dir = args.output_py_dir
        TestGenerator.output_yml_dir = args.output_yml_dir
        TestGenerator.debug_level = int(args.debug_level)

    unittest.main()
