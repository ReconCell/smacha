#!/usr/bin/env python

import sys
import argparse
import os
import unittest

from smacha.util import Tester

WRITE_OUTPUT_FILES = False
OUTPUT_PY_DIR = '/tmp/smacha/executive_smach_tutorials/smacha_generated_py'
OUTPUT_YML_DIR = '/tmp/smacha/executive_smach_tutorials/smacha_generated_scripts'
DEBUG_LEVEL = 1


class TestGenerator(Tester):
    """Tester class for comparing SMACHA generated code to code from the
    executive_smach_tutorials.

    The tests run by this class are performed by generating code using SMACHA
    scripts and templates and comparing the generated output code to the
    expected code samples from the executive_smach_tutorials package.
    """

    def __init__(self, *args, **kwargs):
        # Set Tester member variables
        self.set_write_output_files(WRITE_OUTPUT_FILES)
        self.set_output_py_dir(OUTPUT_PY_DIR)
        self.set_output_yml_dir(OUTPUT_YML_DIR)
        self.set_debug_level(DEBUG_LEVEL)

        # Store the base path
        self._base_path = os.path.dirname(os.path.abspath(__file__))

        # Call the parent constructor
        super(TestGenerator, self).__init__(*args,
                                            script_dirs=[os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials')],
                                            template_dirs=[os.path.join(self._base_path, 'smacha_templates/executive_smach_tutorials')],
                                            **kwargs)

    def test_state_machine2(self):
        """Test state_machine2.py"""
        with open(os.path.join(self._base_path, 'executive_smach_tutorials/smach_tutorials/examples/state_machine2.py')) as original_file:
            generated_code = self._generate(os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials/state_machine2.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_user_data2(self):
        """Test user_data2.py"""
        with open(os.path.join(self._base_path, 'executive_smach_tutorials/smach_tutorials/examples/user_data2.py')) as original_file:
            generated_code = self._generate(os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials/user_data2.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_state_machine_nesting2(self):
        """Test state_machine_nesting2.py"""
        with open(os.path.join(self._base_path, 'executive_smach_tutorials/smach_tutorials/examples/state_machine_nesting2.py')) as original_file:
            generated_code = self._generate(os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_concurrence2(self):
        """Test concurrence2.py"""
        with open(os.path.join(self._base_path, 'executive_smach_tutorials/smach_tutorials/examples/concurrence2.py')) as original_file:
            generated_code = self._generate(os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials/concurrence2.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_actionlib2_test(self):
        """Test actionlib2_test.py"""
        with open(os.path.join(self._base_path, 'executive_smach_tutorials/smach_tutorials/examples/actionlib2_test.py')) as original_file:
            generated_code = self._generate(os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials/actionlib2_test.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_service_test(self):
        """Test service_test.py"""
        with open(os.path.join(self._base_path, 'executive_smach_tutorials/smach_tutorials/examples/service_test.py')) as original_file:
            generated_code = self._generate(os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials/service_test.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_generate_state_machine_nesting2_shorthand(self):
        """Test generate state_machine_nesting2_shorthand.yml"""
        with open(os.path.join(self._base_path, 'executive_smach_tutorials/smach_tutorials/examples/state_machine_nesting2.py')) as original_file:
            generated_code = self._generate(os.path.join(self._base_path, 'smacha_scripts/executive_smach_tutorials/state_machine_nesting2_shorthand.yml'))
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))


if __name__=="__main__":

    # Parse arguments
    arg_parser = argparse.ArgumentParser(description='SMACHA executive_smach_tutorials unit tests.')

    arg_parser.add_argument('-w', '--write',
                            action='store_true',
                            default=WRITE_OUTPUT_FILES,
                            help='Write generated output files to disk.')

    arg_parser.add_argument('-op', '--output-py-dir',
                            dest='output_py_dir',
                            default=OUTPUT_PY_DIR,
                            action='store',
                            help='Specify output SMACH Python executable directory.')

    arg_parser.add_argument('-oy', '--output-yml-dir',
                            dest='output_yml_dir',
                            default=OUTPUT_YML_DIR,
                            action='store',
                            help='Specify output SMACHA YAML script directory.')

    arg_parser.add_argument('-d', '--debug-level',
                            dest='debug_level',
                            action='store',
                            default=DEBUG_LEVEL,
                            help='Set debug level (0-2. Default: 1)')

    # Some trickery to avoid disrupting unittest with args
    if len(sys.argv) > 1:
        argv = sys.argv[1:]
        sys.argv = sys.argv[:1]
        args = arg_parser.parse_args(argv)
        WRITE_OUTPUT_FILES = args.write
        OUTPUT_PY_DIR = args.output_py_dir
        OUTPUT_YML_DIR = args.output_yml_dir
        DEBUG_LEVEL = int(args.debug_level)

    unittest.main()
