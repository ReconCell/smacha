from __future__ import unicode_literals

import os
import re
import difflib
import unittest
import stat

import smacha


class bcolors:
    """Colour terminal print strings.

    A helper class containing string codes that can be concatenated
    to strings in order to provide colour to terminal output.
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Tester(unittest.TestCase):
    """Utility class for running unit tests.

    The tests run by this class are performed by generating code using SMACHA
    scripts and templates and comparing the generated output code to the
    expected code.
    """

    _write_output_files = False
    _output_py_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  '../../test/smacha_generated_py')
    _output_yml_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   '../../test/smacha_generated_scripts')
    _debug_level = 1

    def __init__(self, *args, **kwargs):
        """
        Constructor.

        :param script_dirs:
            A list of directories in which to search for SMACHA YAML scripts.
        :type script_dirs: list of str
        :param template_dirs:
            A list of directories in which to search for SMACHA templates.
        :type template_dirs: list of str
        :param include_introspection_server:
            A flag indicating whether a SMACH introspection server should be
            included in the generated output code.
        :type include_introspection_server: bool
        """
        # Handle args
        if 'script_dirs' in kwargs.keys():
            self._script_dirs = kwargs.pop('script_dirs')
        else:
            self._script_dirs = []
        if 'template_dirs' in kwargs.keys():
            self._template_dirs = kwargs.pop('template_dirs')
        else:
            self._template_dirs = []
        if 'include_introspection_server' in kwargs.keys():
            self._include_introspection_server = kwargs.pop('include_introspection_server')
        else:
            self._include_introspection_server = False

        super(Tester, self).__init__(*args, **kwargs)

        # Create the output directories if they don't exist
        if not os.path.exists(self._output_py_dir):
            os.makedirs(self._output_py_dir)
        if not os.path.exists(self._output_yml_dir):
            os.makedirs(self._output_yml_dir)

        # Load parser
        self._parser = smacha.Parser(script_dirs=self._script_dirs)

        # Load templater
        self._templater = smacha.Templater(
            self._template_dirs,
            include_introspection_server=self._include_introspection_server)

        # Load code generator
        self._generator = smacha.Generator(self._parser, self._templater, verbose=False)

    def set_write_output_files(self, write_output_files):
        self._write_output_files = write_output_files

    def set_output_py_dir(self, output_py_dir):
        self._output_py_dir = output_py_dir

    def set_output_yml_dir(self, output_yml_dir):
        self._output_yml_dir = output_yml_dir

    def set_debug_level(self, debug_level):
        self._debug_level = debug_level

    def _generate(self,
                  smacha_script_filename,
                  output_file = None):
        """Generate smach code using smacha yaml script file and templates.

        :param smacha_script_filename: The SMACHA YAML script filename.
        :type smacha_script_filename: str
        :param output_file:
            An (optional) output filename for the generated output code.
        :type output_file: str or None
        """
        # Load and parse SMACHA script
        script_str, _ = self._parser.load(smacha_script_filename)
        script = self._parser.parse(script_str)

        # Generate the SMACH code
        smach_code = self._generator.run(script)

        # Write the final output to a SMACH python file
        if self._write_output_files:
            if not output_file:
                output_file = os.path.splitext(os.path.basename(smacha_script_filename))[0] + '_generate_output.py'
            with open(os.path.join(self._output_py_dir, output_file), 'w') as smach_file:
                smach_file.write(smach_code)
            os.chmod(os.path.join(self._output_py_dir, output_file),
                        stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR |
                        stat.S_IRGRP | stat.S_IXGRP)

        return smach_code

    def _contain(self,
                 smacha_script_filename,
                 container_name, container_type, states,
                 output_file = None):
        """Containerize a sequence of states in a script.

        :param smacha_script_filename: The SMACHA YAML script filename.
        :type smacha_script_filename: str
        :param container_name: The name of the container.
        :type container_name: str
        :param container_type:
            The type of container (e.g. 'StateMachine' or 'Concurrence').
        :type container_type: str
        :param states: A sequence of states in the script file to containerize.
        :type states: list of str
        :param output_file:
            An (optional) output filename for the generated output code.
        :type output_file: str or None
        """
        # Load and parse SMACHA script
        script_str, _ = self._parser.load(smacha_script_filename)
        script = self._parser.parse(script_str)

        # Use the contain method in the parser to do script conversion
        contained_script = self._parser.contain(script, container_name, container_type, states)

        # Dump the script to string
        contained_script_string = self._parser.dump(contained_script)

        # Write the final output to a SMACHA YAML file
        if self._write_output_files:
            if not output_file:
                output_file = os.path.splitext(os.path.basename(smacha_script_filename))[0] + '_contain_output.yml'
            with open(os.path.join(self._output_yml_dir, output_file), 'w') as contained_script_file:
                contained_script_file.write(contained_script_string)

        return contained_script_string

    def _extract(self,
                 smacha_script_filename,
                 container_state_name,
                 sub_script_filename,
                 output_sub_script_file=None,
                 output_super_script_file=None):
        """Extract a container state in a script and export to sub-script and
        super-script.

        :param smacha_script_filename: The SMACHA YAML script filename.
        :type smacha_script_filename: str
        :param container_state_name:
            The name of the container state in the script.
        :type container_state_name: str
        :param sub_script_filename:
            Desired name for prospective sub-script file (this is the filename
            used in the super-script to refer to the sub-script).
        :type sub_script_filename: str
        :param output_sub_script_file:
            An (optional) output filename for the generated sub-script.
        :type output_sub_script_file: str or None
        :param output_super_script_file:
            An (optional) output filename for the generated super-script.
        :type output_super_script_file: str or None
        """
        # Load and parse SMACHA script
        script_str, _ = self._parser.load(smacha_script_filename)
        script = self._parser.parse(script_str)

        # Use the extract() method in the parser to perform the extraction
        sub_script, super_script = self._parser.extract(script, container_state_name, sub_script_filename=sub_script_filename)

        # Dump the sub-script to a string
        extracted_sub_script_string = self._parser.dump([sub_script])

        # Dump the super-script to a string
        extracted_super_script_string = self._parser.dump(super_script)

        # Write the final output to a SMACHA YAML files
        if self._write_output_files:
            if not output_sub_script_file:
                output_sub_script_file = os.path.splitext(os.path.basename(smacha_script_filename))[0] + '_sub_script_extract_output.yml'
            if not output_super_script_file:
                output_super_script_file = os.path.splitext(os.path.basename(smacha_script_filename))[0] + '_super_script_extract_output.yml'
            with open(os.path.join(self._output_yml_dir, output_sub_script_file), 'w') as extracted_sub_script_file:
                extracted_sub_script_file.write(extracted_sub_script_string)
            with open(os.path.join(self._output_yml_dir, output_super_script_file), 'w') as extracted_super_script_file:
                extracted_super_script_file.write(extracted_super_script_string)

        return extracted_sub_script_string, extracted_super_script_string

    def _strip_uuids(self, code):
        """Strip unique identifier strings (uuid's) from code string.

        :param code: Code string.
        :type code: str
        :return: Code string stripped of uuid's.
        :rtype: str
        """
        # Remove uuid's from lambda callback defs in generated code
        # See: https://stackoverflow.com/questions/11384589/what-is-the-correct-regex-for-matching-values-generated-by-uuid-uuid4-hex
        code = re.sub(r'_[a-f0-9]{8}-?[a-f0-9]{4}-?4[a-f0-9]{3}-?[89ab][a-f0-9]{3}-?[a-f0-9]{12}_', r'_', code)

        return code

    def _compare(self, code_a, code_b, file_a='code_a', file_b='code_b'):
        """Diff compare code_a with code_b.

        Run a diff comparison between two code strings
        and report whether or not they match.

        :param code_a: First code string.
        :type code_a: str
        :param code_b: Second code string.
        :type code_b: str
        :param file_a:
            Filename for first code string for reporting/debugging purposes.
        :type file_a: str
        :param file_b:
            Filename for second code string for reporting/debugging purposes.
        :type file_b: str
        :return: True if code strings match, False if not.
        :rtype: bool
        """
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

        if self._debug_level > 1:
            print('\n' + file_a + ':\n')
            print(code_a_stripped)
            print('\n' + file_b + ':\n')
            print(code_b_stripped)

        # Use difflib to compare
        same = True
        for line in difflib.unified_diff(code_a_stripped, code_b_stripped, fromfile=file_a, tofile=file_b, lineterm=''):
            if line:
                if self._debug_level > 0:
                    print(line)
                same = False

        return same
