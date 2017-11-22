#!/usr/bin/env python

from __future__ import unicode_literals

import sys
import argparse
import os
import smacha
import re
import difflib
import unittest
import stat

class TestGenerator(unittest.TestCase):

    write_output_files = False
    debug_level = 1

    def __init__(self, *args, **kwargs):
        super(TestGenerator, self).__init__(*args, **kwargs)
    
    def _generate(self, smacha_script_filename, script_dirs, template_dirs):
        """Generate smach code using smacha yaml script file and templates."""
        # Load parser
        parser = smacha.Parser(script_dirs = script_dirs)
        
        # Load and parse SMACHA script
        script_str, _ = parser.load(smacha_script_filename)
        script = parser.parse(script_str)
        
        # Load template processor
        templater = smacha.Templater(template_dirs)
        
        # Load code generator
        generator = smacha.Generator(parser, templater, verbose=False)
        
        # Generate the SMACH code
        smach_code = generator.run(script)
        
        # Write the final output to a SMACH python file
        if self.write_output_files:
            with open(smacha_script_filename + '.py', 'w') as smach_file:
                smach_file.write(smach_code)
            os.chmod(smacha_script_filename + '.py',
                        stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR |
                        stat.S_IRGRP | stat.S_IXGRP)

        return smach_code

    def _contain(self,
                 smacha_script_filename, script_dirs,
                 container_name, container_type, states,
                 output_file_stub = '_contain_output.yml'):
        """Containerize a sequence of states in a script."""
        # Load parser
        parser = smacha.Parser(script_dirs = script_dirs)

        # Load and parse SMACHA script
        script_str, _ = parser.load(smacha_script_filename)
        script = parser.parse(script_str)
    
        # Use the contain method in the parser to do script conversion
        contained_script = parser.contain(script, container_name, container_type, states)

        # Dump the script to string
        contained_script_string = parser.dump(contained_script)
        
        # Write the final output to a SMACHA YAML file
        if self.write_output_files:
            with open(os.path.splitext(smacha_script_filename)[0] + output_file_stub, 'w') as contained_script_file:
                contained_script_file.write(contained_script_string)

        return contained_script_string
                
    def _extract(self,
                 smacha_script_filename, script_dirs,
                 container_state_name,
                 sub_script_filename,
                 output_sub_script_file_stub = '_sub_script_extract_output.yml',
                 output_super_script_file_stub = '_super_script_extract_output.yml'):
        """Extract a container state in a script and export to sub-script and super-script."""
        # Load parser
        parser = smacha.Parser(script_dirs = script_dirs)

        # Load and parse SMACHA script
        script_str, _ = parser.load(smacha_script_filename)
        script = parser.parse(script_str)

        # Use the extract() method in the parser to perform the extraction
        sub_script, super_script = parser.extract(script, container_state_name, sub_script_filename=sub_script_filename)
        
        # Dump the sub-script to a string
        extracted_sub_script_string = parser.dump([sub_script])
        
        # Dump the super-script to a string
        extracted_super_script_string = parser.dump(super_script)

        # Write the final output to a SMACHA YAML files
        if self.write_output_files:
            with open(os.path.splitext(smacha_script_filename)[0] + output_sub_script_file_stub, 'w') as extracted_sub_script_file:
                extracted_sub_script_file.write(extracted_sub_script_string)
            
            with open(os.path.splitext(smacha_script_filename)[0] + output_super_script_file_stub, 'w') as extracted_super_script_file:
                extracted_super_script_file.write(extracted_super_script_string)

        return extracted_sub_script_string, extracted_super_script_string

    def _compare(self, code_a, code_b, file_a='code_a', file_b='code_b'):
        """Diff compare code_a with code_b."""
        # Strip single-line comments
        code_a = re.sub('\s*#.*\n', '\n', code_a)
        code_b = re.sub('\s*#.*\n', '\n', code_b)

        # Squeeze whitespace after commas
        code_a = re.sub(',\s+', ',', code_a)
        code_b = re.sub(',\s+', ',', code_b)
        
        # Squeeze whitespace before and after colons
        code_a = re.sub('\s*\:\s*', ':', code_a)
        code_b = re.sub('\s*\:\s*', ':', code_b)

        # Strip (python agnostic!) whitespace from both code strings and convert to lists
        code_a_stripped = [line for line in code_a.strip().splitlines() if line != '' and
                                                                           re.match('^\s+$', line) is None]
        code_b_stripped = [line for line in code_b.strip().splitlines() if line != '' and
                                                                           re.match('^\s+$', line) is None]
        # Strip trailing whitespace from each line
        code_a_stripped = [line.rstrip() for line in code_a_stripped]
        code_b_stripped = [line.rstrip() for line in code_b_stripped]

        if self.debug_level > 1:
            print('\n' + file_a + ':\n')
            print(code_a_stripped)
            print('\n' + file_b + ':\n')
            print(code_b_stripped)

        # Use difflib to compare
        same = True
        for line in difflib.unified_diff(code_a_stripped, code_b_stripped, fromfile=file_a, tofile=file_b, lineterm=''):
            if line:
                if self.debug_level > 0:
                    print(line)
                same = False

        return same

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
        TestGenerator.write_output_files = args.write
        TestGenerator.debug_level = int(args.debug_level)

    unittest.main()
