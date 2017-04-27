#!/usr/bin/env python

import smacha

import re

# import unittest

import difflib

def main():
    
    # Load parser
    parser = smacha.Parser()
    
    # Load and parse SMACHA script
    with open('state_machine2.yml') as smacha_file:
        smacha_script = parser.parse(smacha_file)
    
    # Load template processor
    templater = smacha.Templater(['./templates/state_machine2'])
    
    # Load code generator
    generator = smacha.Generator(templater)
    
    # Generate the SMACH code
    smach_code = generator.run(smacha_script)
    
    # Write the final output to a SMACH python file
    # with open('simple_state_machine_generated.py', 'w') as smach_file:
    #     smach_file.write(smach_code)

    with open('state_machine2.py') as original:
        original_code = [line for line in original.read().strip().splitlines() if line != '' and re.match('^\s+$', line) is None]
        generated_code = [line for line in smach_code.strip().splitlines() if line != '' and re.match('^\s+$', line) is None]

        # print(original_code)
        # print(generated_code)

        for line in difflib.unified_diff(original_code, generated_code, fromfile='original', tofile='generated', lineterm=''):
            print line

if __name__=="__main__":
    main();
