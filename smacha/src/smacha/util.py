from __future__ import unicode_literals

import sys
import argparse
import os
import smacha
import re
import difflib
import unittest
import stat

import smacha

class bcolors:
    """Colour terminal print strings."""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class Tester(unittest.TestCase):

    write_output_files = False
    debug_level = 1

    def __init__(self, *args, **kwargs):
        super(Tester, self).__init__(*args, **kwargs)

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
