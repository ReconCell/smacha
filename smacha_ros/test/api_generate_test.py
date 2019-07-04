#!/usr/bin/env python

import sys
import os

import rospy
import rospkg
import rostest

from rosgraph_msgs.msg import Log

from smacha.util import Tester
from smacha.util import bcolors

import unittest2 as unittest

# YAML parser
from ruamel import yaml

ROS_TEMPLATES_DIR = '../src/smacha_ros/templates'
DEBUG_LEVEL = 1

# Script directory
rospack = rospkg.RosPack()
ROOT_DIR = rospack.get_path('smacha_ros')

# TEMP
TEMPLATES_DIR = 'smacha_templates/smacha_test_examples'

CONF_FILE = 'test_generate.yml'
DEBUG_LEVEL = 1

class TestClass(Tester):
    """Tester class for general unit testing of various SMACHA tool
    functionalities.
    """

    _logmsg = ''
    _logerr = False

    _script_directories = []
    _template_dirs = []
    _conf_dict = None

    def __init__(self, *args, **kwargs):
        # Set Tester member variables
        self.set_debug_level(DEBUG_LEVEL)

        # Store the base path
        self._base_path = os.path.dirname(os.path.abspath(__file__))
        # Read the config file
        self._read_conf(CONF_FILE)
        
        self._template_dirs.append(os.path.join(self._base_path, ROS_TEMPLATES_DIR))
        # Call the parent constructor
        super(TestClass, self).__init__(
            *args,
            script_dirs=[os.path.join(self._base_path, 'smacha_scripts/smacha_test_examples')],
            template_dirs=self._template_dirs,
            **kwargs)

        rospy.Subscriber('/rosout', Log, self._std_out_cb)
        
        

    def _std_out_cb(self, data):
        
        # Check if there are any errors in the execution 
        # of the state machine that might not throw an
        # exception during the exec().
        msg = data.msg
        level = data.level
        if level >= 8:
            self._logmsg = msg
            self._logerr = True

    def _read_conf(self, conf_file):
        # Read the configuration file before parsing arguments,
        try:
            conf_file_loc = os.path.join(self._base_path, conf_file)
            f = open(conf_file_loc)
            self._conf_dict = yaml.load(f)
        except Exception as e:
            print('Failed to read the configuration file. See error:\n{}'.format(e))
            exit()

        try:
            [self._script_directories.append(os.path.join(self._base_path,d)) for d in self._conf_dict['SCRIPT_DIRS']]
        except Exception as e:
            rospy.logerr("Failed to read SCRIPT_DIRS from conf file\n{}".format(e))
            exit()

        try:
            [self._template_dirs.append(os.path.join(self._base_path,d)) for d in self._conf_dict['TEMPLATE_DIRS']]
        except Exception as e:
            rospy.logerr("Failed to read TEMPLATE_DIRS from conf file\n{}".format(e))
            exit()



    def test_code_generation_and_execution(self):
        """Generate Python code from scripts listed in the conf file
        and run them."""
        
        script_files = []
        script_loc = []
        
        generated_code = ''
        script_file = ''
        smacha_script = ''
        script_names = []
        # Get a list of available files
        for d in self._script_directories:
            file_list = os.listdir(os.path.join(self._base_path, d))
            script_files.extend(file_list)
            script_loc.extend([os.path.join(self._base_path, d)]*len(file_list))
        
        for test_case in self._conf_dict['TEST_GENERATE']:
            with self.subTest(test_case=test_case):
                success = False
                try:
                    matching_files = (s for s in script_files if test_case.values()[0]['script'] in s).next()

                    if len(matching_files) == 0:
                        raise Exception("Script not found in directories")
                    try:
                        script_file = script_loc[script_files.index(matching_files)] + '/' + matching_files
                    except:
                        rospy.logerr('script_loc:\n{}'.format(script_loc))
                        rospy.logerr('matching_files:\n{}'.format(matching_files))
                        rospy.logerr('script_files.index(matching_files):\n{}'.format(script_files.index(matching_files)))
                        raise Exception("Failed to compile string")
                    
                    # Pass the script into the generator service and write the result into a variable
                    generated_code = self._generate(script_file)
                    
                    # We need to take the "init_node" from the generated code as this program here already initializes a node
                    generated_code = generated_code.replace(
                    "rospy.init_node",
                    "# rospy.init_node"
                    )

                    # Execute the generated python code
                    exec(generated_code, globals())
                    if self._logerr:
                        raise Exception("Error in state machine execution !!")
                    success = True
                except Exception as e:
                    rospy.logerr(
                        'Failed due to exception when generating [{0}]!!'.format(script_file))
                    rospy.logerr('Exception: {0}'.format(e))
                    rospy.logerr('YAML\n===========\n{0}'.format(smacha_script))
                
                self.assertTrue(success) 


if __name__=="__main__":
    
    rospy.init_node('test_smacha_service_generate',log_level=rospy.DEBUG)

    rostest.rosrun('smacha_ros', 'test_smacha_service_generate', TestClass)



#     def _std_out_cb(self, data):
        
#         # Check if there are any errors in the execution 
#         # of the state machine that might not throw an
#         # exception during the exec().
#         msg = data.msg
#         level = data.level
#         if level >= 8:
#             self._logmsg = msg
#             self._logerr = True

    # def test_code_generation_and_execution(self):
    #     """Generate Python code from all the available scripts in the test folder
    #     and run them."""

    #     success = False
    #     generated_code = ''
    #     script_file = ''
    #     try:
    #         # Get a list of available files
    #         script_names = os.listdir(os.path.join(self._base_path, TEST_SCRIPTS_DIR))
    #         script_names.sort()
    #         # Remove all subscripts from the test array
    #         script_names = [ script_name for script_name in script_names if script_name.find('sub_script') == -1]
    #         # Loop to generate Python code from each YML script
    #         for script_name in script_names:
    #             script_file = TEST_SCRIPTS_DIR + script_name
    #             # Pass the script into the generator and write the result into a variable
    #             generated_code = self._generate(os.path.join(self._base_path, script_file))
    #             # We need to take the "init_node" from the generated code as this program here already initializes a node
    #             generated_code = generated_code.replace(
    #             "rospy.init_node",
    #             "# rospy.init_node"
    #             )

    #             # Execute the generated python code
    #             exec(generated_code, globals())
    #             if self._logerr:
    #                 raise Exception("Error in state machine execution !!")
    #         success = True
    #     except Exception as e:
    #         rospy.logerr(
    #             'Failed due to exception when generating [{0}]!!'.format(script_file))
    #         rospy.logerr('Exception: {0}'.format(e))

    #     self.assertTrue(success) 

# if __name__=="__main__":
    
#     rospy.init_node('test_smacha_api_generate',log_level=rospy.DEBUG)
#     rostest.rosrun('smacha_ros', 'test_smacha_api_generate', TestClass)
