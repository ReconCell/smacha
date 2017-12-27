#!/usr/bin/env python

import os

from smacha.util import Tester

class TestTools(Tester):

    def test_hard_coded_params(self):
        """Test hard_coded_params.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_test_examples/params.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/smacha_test_examples/hard_coded_params.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            [base_path + '/smacha_templates/smacha_test_examples'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_assigned_params(self):
        """Test assigned_params.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_test_examples/params.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/smacha_test_examples/assigned_params.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            [base_path + '/smacha_templates/smacha_test_examples'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_nesting_params(self):
        """Test nesting_params.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_test_examples/nesting_params.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/smacha_test_examples/nesting_params.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            [base_path + '/smacha_templates/smacha_test_examples'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_nesting_params_with_sub_script(self):
        """Test nesting_params_with_sub_script.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_test_examples/nesting_params.py') as original_file:
            generated_code = self._generate(base_path + '/smacha_scripts/smacha_test_examples/nesting_params_with_sub_script.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            [base_path + '/smacha_templates/smacha_test_examples'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_containerize_seq_1(self):
        """Test containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_1.yml') as original_file:
            generated_code = self._contain(base_path + '/smacha_scripts/smacha_test_examples/seq.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            'SUB', 'StateMachine', ['FOO_0', 'FOO_1'],
                                            output_file_stub = '_nesting_1_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_containerize_seq_2(self):
        """Test containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_2.yml') as original_file:
            generated_code = self._contain(base_path + '/smacha_scripts/smacha_test_examples/seq.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            'SUB', 'StateMachine', ['FOO_1', 'FOO_2'],
                                            output_file_stub = '_nesting_2_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_containerize_seq_3(self):
        """Test containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_3.yml') as original_file:
            generated_code = self._contain(base_path + '/smacha_scripts/smacha_test_examples/seq.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            'SUB', 'StateMachine', ['FOO_0', 'FOO_1', 'FOO_2'],
                                            output_file_stub = '_nesting_3_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_extract_seq_nesting_1(self):
        """Test extracting container state as sub-script from seq_nesting_1.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_1_sub_script.yml') as original_sub_script_file, open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_1_super_script.yml') as original_super_script_file:
            generated_sub_script_code, generated_super_script_code = (
                self._extract(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_1.yml',
                              [base_path + '/smacha_scripts/smacha_test_examples'],
                              'SUB', '/smacha_scripts/seq_nesting_1_sub_script.yml'))
            original_sub_script_code = original_sub_script_file.read()
            original_super_script_code = original_super_script_file.read()
            self.assertTrue(
                self._compare(generated_sub_script_code, original_sub_script_code, file_a='generated', file_b='original') and
                self._compare(generated_super_script_code, original_super_script_code, file_a='generated', file_b='original'))
    
    def test_extract_seq_nesting_2(self):
        """Test extracting container state as sub-script from seq_nesting_2.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_2_sub_script.yml') as original_sub_script_file, open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_2_super_script.yml') as original_super_script_file:
            generated_sub_script_code, generated_super_script_code = (
                self._extract(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_2.yml',
                              [base_path + '/smacha_scripts/smacha_test_examples'],
                              'SUB', '/smacha_scripts/seq_nesting_2_sub_script.yml'))
            original_sub_script_code = original_sub_script_file.read()
            original_super_script_code = original_super_script_file.read()
            self.assertTrue(
                self._compare(generated_sub_script_code, original_sub_script_code, file_a='generated', file_b='original') and
                self._compare(generated_super_script_code, original_super_script_code, file_a='generated', file_b='original'))
    
    def test_extract_seq_nesting_3(self):
        """Test extracting container state as sub-script from seq_nesting_3.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_3_sub_script.yml') as original_sub_script_file, open(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_3_super_script.yml') as original_super_script_file:
            generated_sub_script_code, generated_super_script_code = (
                self._extract(base_path + '/smacha_scripts/smacha_test_examples/seq_nesting_3.yml',
                              [base_path + '/smacha_scripts/smacha_test_examples'],
                              'SUB', '/smacha_scripts/seq_nesting_3_sub_script.yml'))
            original_sub_script_code = original_sub_script_file.read()
            original_super_script_code = original_super_script_file.read()
            self.assertTrue(
                self._compare(generated_sub_script_code, original_sub_script_code, file_a='generated', file_b='original') and
                self._compare(generated_super_script_code, original_super_script_code, file_a='generated', file_b='original'))
    
    def test_concurrence_containerize_seq_1(self):
        """Test concurrence containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_concurrence_1.yml') as original_file:
            generated_code = self._contain(base_path + '/smacha_scripts/smacha_test_examples/seq.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            'CON', 'Concurrence', ['FOO_0', 'FOO_1'],
                                            output_file_stub = '_concurrence_1_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_concurrence_containerize_seq_2(self):
        """Test concurrence containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_concurrence_2.yml') as original_file:
            generated_code = self._contain(base_path + '/smacha_scripts/smacha_test_examples/seq.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            'CON', 'Concurrence', ['FOO_1', 'FOO_2'],
                                            output_file_stub = '_concurrence_2_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_concurrence_containerize_seq_3(self):
        """Test concurrence containerizing seq.yml"""
        base_path = os.path.dirname(os.path.abspath(__file__))
        with open(base_path + '/smacha_scripts/smacha_test_examples/seq_concurrence_3.yml') as original_file:
            generated_code = self._contain(base_path + '/smacha_scripts/smacha_test_examples/seq.yml',
                                            [base_path + '/smacha_scripts/smacha_test_examples'],
                                            'CON', 'Concurrence', ['FOO_0', 'FOO_1', 'FOO_2'],
                                            output_file_stub = '_concurrence_3_contain_output.yml')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

if __name__=="__main__":
    
    # Parse arguments
    arg_parser = argparse.ArgumentParser(description='SMACHA test examples unit tests.')
    
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
        TestTools.write_output_files = args.write
        TestTools.debug_level = int(args.debug_level)

    unittest.main()
