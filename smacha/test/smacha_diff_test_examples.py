#!/usr/bin/env python

import sys
import argparse
import os
import unittest2 as unittest
from ruamel import yaml
from smacha.util import Tester

WRITE_OUTPUT_FILES = False
OUTPUT_PY_DIR = '/tmp/smacha/smacha_test_examples/smacha_generated_py'
OUTPUT_YML_DIR = '/tmp/smacha/smacha_test_examples/smacha_generated_scripts'
CONF_FILE = 'test_examples_config.yml'
DEBUG_LEVEL = 1

class TestTools(Tester):
    """Tester class for general unit testing of various SMACHA tool
    functionalities.

    The tests run by this class are performed by generating code using SMACHA
    scripts and templates and comparing the generated output code to the
    expected code from hand-written code samples.

    This includes testing both SMACHA YAML scripts generated by, e.g. the
    :func:`smacha.parser.contain` and :func:`smacha.parser.extract` methods,
    and Python code generated by the :func:`smacha.generator.run` method.
    """

    def __init__(self, *args, **kwargs):
        # Set Tester member variables
        self.set_write_output_files(WRITE_OUTPUT_FILES)
        self.set_output_py_dir(OUTPUT_PY_DIR)
        self.set_output_yml_dir(OUTPUT_YML_DIR)
        self.set_debug_level(DEBUG_LEVEL)

        # Store the base path
        self._base_path = os.path.dirname(os.path.abspath(__file__))

        # Read the config file
        self._read_config_file(CONF_FILE)

        # Call the parent constructor
        super(TestTools, self).__init__(
            *args,
            script_dirs=[os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples')],
            template_dirs=[os.path.join(self._base_path, 'smacha_templates/smacha_test_examples')],
            **kwargs)

    def _read_config_file(self, conf_file):
        # Read the configuration file before parsing arguments,
        try:
            base_path = os.path.dirname(os.path.abspath(__file__))
            conf_file_loc = os.path.join(base_path, conf_file)
            f = open(conf_file_loc)
            self._config_dict = yaml.load(f)
        except Exception as e:
            print('Failed to read the configuration file. See error:\n{}'.format(e))
            exit()

        if self._config_dict.has_key('WRITE_OUTPUT_FILES'):
            WRITE_OUTPUT_FILES = self._config_dict['WRITE_OUTPUT_FILES']
        if self._config_dict.has_key('OUTPUT_PY_DIR'):
            OUTPUT_PY_DIR = self._config_dict['OUTPUT_PY_DIR']
        if self._config_dict.has_key('OUTPUT_YML_DIR'):
            OUTPUT_YML_DIR = self._config_dict['OUTPUT_YML_DIR']
        if self._config_dict.has_key('DEBUG_LEVEL'):
            DEBUG_LEVEL = self._config_dict['DEBUG_LEVEL']

    def test_generate(self):
        """Test generating against baseline files"""
        try:
            for test_case in self._config_dict['TEST_GENERATE']:
                with self.subTest(test_case=test_case):
                    test_params = test_case.values()[0]
                    script_file = test_params['script']
                    baseline = test_params['baseline']
                    with open(os.path.join(self._base_path, 'smacha_test_examples/{}'.format(baseline))) as original_file:
                        generated_code = self._strip_uuids(self._generate(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(script_file))))
                        original_code = original_file.read()
                    self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))
        except Exception as e:
            print("Exception at test_generate:\n{}".format(e))
            print("self._config_dict:\n{}".format(self._config_dict))
            self.assertTrue(False)

    def test_containerize(self):
        """Test containerizing states against baseline files"""
        for test_case in self._config_dict['TEST_CONTAIN']:
            with self.subTest(test_case=test_case):
                test_params = test_case.values()[0]
                script_file = test_params['script']
                baseline = test_params['baseline']
                contain_into = test_params['contain_into']
                contain_from = test_params['contain_from']
                with open(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(baseline))) as original_file:
                    generated_code = self._contain(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(script_file)),
                                                contain_into, 'StateMachine', contain_from,
                                                output_file = '{}_contain_output.yml'.format(baseline))
                    original_code = original_file.read()
                    self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_containerize_concurrence(self):
        """Test containerizing of concurrent states against baseline files"""
        for test_case in self._config_dict['TEST_CON_CONTAIN']:
            with self.subTest(test_case=test_case):
                test_params = test_case.values()[0]
                script_file = test_params['script']
                baseline = test_params['baseline']
                contain_into = test_params['contain_into']
                contain_from = test_params['contain_from']
                with open(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(baseline))) as original_file:
                    generated_code = self._contain(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(script_file)),
                                                contain_into, 'Concurrence', contain_from,
                                                output_file = '{}_contain_output.yml'.format(baseline))
                    original_code = original_file.read()
                    self.assertTrue(self._compare(generated_code, original_code, file_a='generated', file_b='original'))

    def test_extract(self):
        """Test extracting container state as sub-script against baseline scripts"""
        for test_case in self._config_dict['TEST_EXTRACT']:
            with self.subTest(test_case=test_case):
                test_params = test_case.values()[0]
                test_name = test_case.keys()[0]
                script_file = test_params['script']
                baseline_sub = test_params['baseline_sub']
                baseline_sup = test_params['baseline_sup']
                extract_state = test_params['extract_state']
                with open(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(baseline_sub))) as original_sub_script_file, open(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(baseline_sup))) as original_super_script_file:
                    generated_sub_script_code, generated_super_script_code = (
                        self._extract(os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples/{}'.format(script_file)),
                                    extract_state, '/smacha_scripts/{}_sub_script.yml'.format(test_name)))
                    original_sub_script_code = original_sub_script_file.read()
                    original_super_script_code = original_super_script_file.read()
                    self.assertTrue(
                        self._compare(generated_sub_script_code, original_sub_script_code, file_a='generated', file_b='original') and
                        self._compare(generated_super_script_code, original_super_script_code, file_a='generated', file_b='original'))


if __name__=="__main__":

    # Parse arguments (provided arguments will override values from the conf file)
    arg_parser = argparse.ArgumentParser(description='SMACHA test examples unit tests.\
        Test configuration is read from the file {0}. Parameters provided here will override\
        the ones provided in the file.'.format(CONF_FILE))
    
    arg_parser.add_argument('-c', '--conf',
                            dest='conf_file',
                            default=CONF_FILE,
                            help='Specify the configuration file (YAML). Note that \
                            arguments/flags provided after this one will override \
                            the options specified in the configuration file.')
    
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
        CONF_FILE = args.conf_file
        OUTPUT_PY_DIR = args.output_py_dir
        OUTPUT_YML_DIR = args.output_yml_dir
        DEBUG_LEVEL = int(args.debug_level)


    unittest.main()