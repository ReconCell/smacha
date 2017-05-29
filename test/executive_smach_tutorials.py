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
        self._base_path = os.path.dirname(os.path.abspath(__file__))
        super(TestGenerator, self).__init__(*args, **kwargs)
    
    def _generate(self, smachaml_filename, template_dirs):
        """Generate smach code using smachaml file and templates."""
        # Load parser
        parser = smacha.Parser()
        
        # Load and parse smachaml script
        with open(smachaml_filename) as smachaml_file:
            smachaml_script = parser.parse(smachaml_file)
        
        # Load template processor
        templater = smacha.Templater(template_dirs)
        
        # Load code generator
        generator = smacha.Generator(templater, verbose=False)
        
        # Generate the SMACH code
        smach_code = generator.run(smachaml_script)
        
        # Write the final output to a SMACH python file
        if self.write_output_files:
            with open(smachaml_filename + '.py', 'w') as smach_file:
                smach_file.write(smach_code)
            os.chmod(smachaml_filename + '.py',
                        stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR |
                        stat.S_IRGRP | stat.S_IXGRP)

        return smach_code

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

    def test_state_machine2(self):
        """Test state_machine2.py"""
        with open(self._base_path + '/executive_smach_tutorials/smach_tutorials/examples/state_machine2.py') as original_file:
            generated_code = self._generate(self._base_path + '/smachaml/executive_smach_tutorials/state_machine2.yml',
                                            [self._base_path + '/templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_user_data2(self):
        """Test user_data2.py"""
        with open(self._base_path + '/executive_smach_tutorials/smach_tutorials/examples/user_data2.py') as original_file:
            generated_code = self._generate(self._base_path + '/smachaml/executive_smach_tutorials/user_data2.yml',
                                            [self._base_path + '/templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_state_machine_nesting2(self):
        """Test state_machine_nesting2.py"""
        with open(self._base_path + '/executive_smach_tutorials/smach_tutorials/examples/state_machine_nesting2.py') as original_file:
            generated_code = self._generate(self._base_path + '/smachaml/executive_smach_tutorials/state_machine_nesting2.yml',
                                            [self._base_path + '/templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_concurrence2(self):
        """Test concurrence2.py"""
        with open(self._base_path + '/executive_smach_tutorials/smach_tutorials/examples/concurrence2.py') as original_file:
            generated_code = self._generate(self._base_path + '/smachaml/executive_smach_tutorials/concurrence2.yml',
                                            [self._base_path + '/templates/executive_smach_tutorials'])
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
    
    def test_actionlib2_test(self):
        """Test actionlib2_test.py"""
        with open(self._base_path + '/executive_smach_tutorials/smach_tutorials/examples/actionlib2_test.py') as original_file:
            generated_code = self._generate(self._base_path + '/smachaml/executive_smach_tutorials/actionlib2_test.yml',
                                            [self._base_path + '/templates/executive_smach_tutorials'])
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
