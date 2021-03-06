#!/usr/bin/env python

import sys
import argparse
import os
import unittest

from smacha.util import Tester
from smacha.util import bcolors

TEST_SCRIPTS_DIR = 'smacha_scripts/executive_smach_tutorials/'
TEMPLATES_DIR = 'smacha_templates/executive_smach_tutorials'
DEBUG_LEVEL = 1
class TestTools(Tester):
    """Tester class for general unit testing of various SMACHA tool
    functionalities.
    """

    def __init__(self, *args, **kwargs):
        # Set Tester member variables
        self.set_debug_level(DEBUG_LEVEL)

        # Store the base path
        self._base_path = os.path.dirname(os.path.abspath(__file__))

        # Call the parent constructor
        super(TestTools, self).__init__(
            *args,
            script_dirs=[os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples')],
            template_dirs=[
                os.path.join(self._base_path, TEMPLATES_DIR)
                ],
            **kwargs)

    def test_code_generation_and_execution(self):
        """Generate Python code from all the available scripts in the test folder
        and run them."""

        success = False
        generated_code = ''
        script_file = ''
        try:
            # Get a list of available files
            script_names = os.listdir(os.path.join(self._base_path, TEST_SCRIPTS_DIR))
            # Remove all subscripts from the test array
            script_names = [ script_name for script_name in script_names if script_name.find('sub_script') == -1]
            # Loop to generate Python code from each YML script
            for script_name in script_names:
                script_file = TEST_SCRIPTS_DIR + script_name
                # Pass the script into the generator and write the result into a variable
                generated_code = self._generate(os.path.join(self._base_path, script_file))
                # Execute the generated python code
                exec(generated_code, globals())
            success = True
        except Exception as e:
            print(
                bcolors.FAIL + 
                'Failed due to exception when generating [{0}]!!'.format(script_file) + 
                bcolors.ENDC)
            print('Exception: {0}'.format(e))

        self.assertTrue(success) 

if __name__=="__main__":
    # Parse arguments
    arg_parser = argparse.ArgumentParser(description='SMACHA test examples unit tests.')

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
        DEBUG_LEVEL = int(args.debug_level)

    unittest.main()
