#!/usr/bin/env python

import os

from smacha.util import Tester

class TestGenerator(Tester):

    def test_state_machine2(self):
        """Test state_machine2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/executive_smach_tutorials/smach_tutorials/examples/state_machine2.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/executive_smach_tutorials/state_machine2.yml',
                                            [],
                                            [base_path + '/smacha_templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_user_data2(self):
        """Test user_data2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/executive_smach_tutorials/smach_tutorials/examples/user_data2.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/executive_smach_tutorials/user_data2.yml',
                                            [],
                                            [base_path + '/smacha_templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_state_machine_nesting2(self):
        """Test state_machine_nesting2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/executive_smach_tutorials/smach_tutorials/examples/state_machine_nesting2.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml',
                                            [],
                                            [base_path + '/smacha_templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_concurrence2(self):
        """Test concurrence2.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/executive_smach_tutorials/smach_tutorials/examples/concurrence2.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/executive_smach_tutorials/concurrence2.yml',
                                            [],
                                            [base_path + '/smacha_templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_actionlib2_test(self):
        """Test actionlib2_test.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/executive_smach_tutorials/smach_tutorials/examples/actionlib2_test.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/executive_smach_tutorials/actionlib2_test.yml',
                                            [],
                                            [base_path + '/smacha_templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_service_test(self):
        """Test service_test.py"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/executive_smach_tutorials/smach_tutorials/examples/service_test.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/executive_smach_tutorials/service_test.yml',
                                            [],
                                            [base_path + '/smacha_templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))


if __name__=="__main__":
    
    # Parse arguments
    arg_parser = argparse.ArgumentParser(description='SMACHA executive_smach_tutorials unit tests.')
    
    arg_parser.add_argument('-w', '--write',
                            action='store_true',
                            help='Write generated output files to disk.')
    
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
        TestGenerator.debug_level = int(args.debug_level)

    unittest.main()
