#!/usr/bin/env python

import sys
import argparse
import os
import unittest

from smacha.util import Tester

WRITE_OUTPUT_FILES = False
OUTPUT_PY_DIR = '/tmp/smacha/smacha_test_examples/smacha_generated_py'
OUTPUT_YML_DIR = '/tmp/smacha/smacha_test_examples/smacha_generated_scripts'
DEBUG_LEVEL = 1

class TestTools(Tester):
    
    def __init__(self, *args, **kwargs):
        # Set Tester member variables
        self.set_write_output_files(WRITE_OUTPUT_FILES)
        self.set_output_py_dir(OUTPUT_PY_DIR)
        self.set_output_yml_dir(OUTPUT_YML_DIR)
        self.set_debug_level(DEBUG_LEVEL)
        
        # Call the parent constructor
        super(TestTools, self).__init__(*args, **kwargs)

    def test_hard_coded_params(self):
        """Test hard_coded_params.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/params.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/hard_coded_params.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_assigned_params(self):
        """Test assigned_params.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/params.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/assigned_params.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_nesting_params(self):
        """Test nesting_params.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/nesting_params.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/nesting_params.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_nesting_params_with_sub_script(self):
        """Test nesting_params_with_sub_script.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/nesting_params.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/nesting_params_with_sub_script.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_containerize_seq_1(self):
        """Test containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_1.yml')) as original_file:
            generated_code = self._contain(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            'SUB', 'StateMachine', ['FOO_0', 'FOO_1'],
                                            output_file = 'seq_nesting_1_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_containerize_seq_2(self):
        """Test containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_2.yml')) as original_file:
            generated_code = self._contain(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            'SUB', 'StateMachine', ['FOO_1', 'FOO_2'],
                                            output_file = 'seq_nesting_2_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_containerize_seq_3(self):
        """Test containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_3.yml')) as original_file:
            generated_code = self._contain(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            'SUB', 'StateMachine', ['FOO_0', 'FOO_1', 'FOO_2'],
                                            output_file = 'seq_nesting_3_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_extract_seq_nesting_1(self):
        """Test extracting container state as sub-script from seq_nesting_1.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_1_sub_script.yml')) as original_sub_script_file, open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_1_super_script.yml')) as original_super_script_file:
            generated_sub_script_code, generated_super_script_code = (
                self._extract(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_1.yml'),
                              [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                              'SUB', '/smacha_scripts/seq_nesting_1_sub_script.yml'))
            original_sub_script_code = original_sub_script_file.read()
            original_super_script_code = original_super_script_file.read()
            self.assertTrue(
                self._compare(generated_sub_script_code, original_sub_script_code, file_a='generated', file_b='original') and
                self._compare(generated_super_script_code, original_super_script_code, file_a='generated', file_b='original'))
    
    def test_extract_seq_nesting_2(self):
        """Test extracting container state as sub-script from seq_nesting_2.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_2_sub_script.yml')) as original_sub_script_file, open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_2_super_script.yml')) as original_super_script_file:
            generated_sub_script_code, generated_super_script_code = (
                self._extract(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_2.yml'),
                              [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                              'SUB', '/smacha_scripts/seq_nesting_2_sub_script.yml'))
            original_sub_script_code = original_sub_script_file.read()
            original_super_script_code = original_super_script_file.read()
            self.assertTrue(
                self._compare(generated_sub_script_code, original_sub_script_code, file_a='generated', file_b='original') and
                self._compare(generated_super_script_code, original_super_script_code, file_a='generated', file_b='original'))
    
    def test_extract_seq_nesting_3(self):
        """Test extracting container state as sub-script from seq_nesting_3.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_3_sub_script.yml')) as original_sub_script_file, open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_3_super_script.yml')) as original_super_script_file:
            generated_sub_script_code, generated_super_script_code = (
                self._extract(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_nesting_3.yml'),
                              [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                              'SUB', '/smacha_scripts/seq_nesting_3_sub_script.yml'))
            original_sub_script_code = original_sub_script_file.read()
            original_super_script_code = original_super_script_file.read()
            self.assertTrue(
                self._compare(generated_sub_script_code, original_sub_script_code, file_a='generated', file_b='original') and
                self._compare(generated_super_script_code, original_super_script_code, file_a='generated', file_b='original'))
    
    def test_concurrence_containerize_seq_1(self):
        """Test concurrence containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_concurrence_1.yml')) as original_file:
            generated_code = self._contain(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            'CON', 'Concurrence', ['FOO_0', 'FOO_1'],
                                            output_file = 'seq_concurrence_1_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_concurrence_containerize_seq_2(self):
        """Test concurrence containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_concurrence_2.yml')) as original_file:
            generated_code = self._contain(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            'CON', 'Concurrence', ['FOO_1', 'FOO_2'],
                                            output_file = 'seq_concurrence_2_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_concurrence_containerize_seq_3(self):
        """Test concurrence containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq_concurrence_3.yml')) as original_file:
            generated_code = self._contain(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/seq.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            'CON', 'Concurrence', ['FOO_0', 'FOO_1', 'FOO_2'],
                                            output_file = 'seq_concurrence_3_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_callbacks(self):
        """Test callbacks.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/callbacks.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/callbacks.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_random_outcomes(self):
        """Test random_outcomes.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/random_outcomes.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/random_outcomes.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_sleep(self):
        """Test sleep.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/sleep.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/sleep.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_print_userdata(self):
        """Test print_userdata.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(base_path, 'smacha_test_examples/print_userdata.py')) as original_file:
            generated_code = self._generate(os.path.join(base_path, 'smacha_scripts/smacha_test_examples/print_userdata.yml'),
                                            [os.path.join(base_path, 'smacha_scripts/smacha_test_examples')],
                                            [os.path.join(base_path, 'smacha_templates/smacha_test_examples')])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

if __name__=="__main__":
    
    # Parse arguments
    arg_parser = argparse.ArgumentParser(description='SMACHA test examples unit tests.')
    
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
