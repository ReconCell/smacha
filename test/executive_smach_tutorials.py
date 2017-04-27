#!/usr/bin/env python

import os
import smacha
import re
import difflib
import unittest

class TestGenerator(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        self._base_path = os.path.dirname(__file__)
        super(TestGenerator, self).__init__(*args, **kwargs)
    
    def _generate(self, smachaml_filename, templates_dirname):
        """Generate smach code using smachaml file and templates."""
        # Load parser
        parser = smacha.Parser()
        
        # Load and parse smachaml script
        with open(smachaml_filename) as smachaml_file:
            smachaml_script = parser.parse(smachaml_file)
        
        # Load template processor
        templater = smacha.Templater([templates_dirname])
        
        # Load code generator
        generator = smacha.Generator(templater)
        
        # Generate the SMACH code
        smach_code = generator.run(smachaml_script)
        
        # Write the final output to a SMACH python file
        # with open('simple_state_machine_generated.py', 'w') as smach_file:
        #     smach_file.write(smach_code)

        return smach_code

    def _compare(self, code_a, code_b):
        """Diff compare code_a with code_b."""
        # Strip (python agnostic!) whitespace from both code strings and convert to lists
        code_a_stripped = [line for line in code_a.strip().splitlines() if line != '' and re.match('^\s+$', line) is None]
        code_b_stripped = [line for line in code_b.strip().splitlines() if line != '' and re.match('^\s+$', line) is None]
    
        # Use difflib to compare
        for line in difflib.unified_diff(code_a_stripped, code_b_stripped, fromfile='code_a', tofile='code_b', lineterm=''):
            if line:
                return False

        return True

    def test_state_machine2(self):
        """Test state_machine2.py"""
        with open(self._base_path + '/executive_smach_tutorials/smach_tutorials/examples/state_machine2.py') as original_file:
            generated_code = self._generate(self._base_path + '/smachaml/executive_smach_tutorials/state_machine2.yml',
                                            self._base_path + '/templates/executive_smach_tutorials/state_machine2')
            original_code = original_file.read()
            self.assertTrue(self._compare(generated_code, original_code))

if __name__=="__main__":
    unittest.main()
