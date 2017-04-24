#! /usr/bin/env python

import argparse
import sys
import os
import re

# Yaml for script parsing
import yaml
from yaml.composer import Composer
from yaml.constructor import Constructor

# Jinja2 for templating & code generation
import jinja2


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class ParseException(BaseException):
    def __init__(self, error=None, line_number=None, line=None):
        if error is not None and line_number is not None and line is not None:
            message = bcolors.FAIL + 'Parsing error at line {0}:\n>>> {1}: {2}'.format(line_number, line, error) + bcolors.ENDC
        elif error is not None and line_number is not None:
            message = bcolors.FAIL + 'Parsing error at line {0}: {1}'.format(line_number, error) + bcolors.ENDC
        elif line_number is not None and line is not None:
            message = bcolors.FAIL +'Parsing error at line {0}:\n>>> {1}'.format(line_number, line) + bcolors.ENDC
        elif error is not None:
            message = bcolors.FAIL + 'Parsing error: {0}'.format(error) + bcolors.ENDC
        else:
            message = bcolors.FAIL + 'Parsing error!' + bcolors.ENDC
        
        super(ParseException, self).__init__(message)


class Parser():
    """SMACHA script parser."""
    def __init__(self):
        self._loader = None
        pass
    
    # Credit to puzzlet@stackoverflow for the following two functions that
    # allow for line numbers to be added to the parsed yaml.
    #
    # http://stackoverflow.com/questions/13319067/parsing-yaml-return-with-line-number
    #
    def _compose_node(self, parent, index):
        # the line number where the previous token has ended (plus empty lines)
        line = self._loader.line
        node = Composer.compose_node(self._loader, parent, index)
        node.__line__ = line + 1
        return node
    
    def _construct_mapping(self, node, deep=False):
        mapping = Constructor.construct_mapping(self._loader, node, deep=deep)
        mapping['__line__'] = node.__line__
        return mapping
    
    def parse(self, script_buffer):
        """Parse SMACHA yaml script."""
        
        self._loader = yaml.Loader(script_buffer)
        
        self._loader.compose_node = self._compose_node
        self._loader.construct_mapping = self._construct_mapping
        
        parsed_script = self._loader.get_single_data()
        
        return parsed_script


class Templater():
    """Jinja template processor."""
    def __init__(self, templates_path):
        self._templates_path = templates_path
        
        # Load templates from the templates folder
        self._template_loader = jinja2.FileSystemLoader(self._templates_path)
        
        # Create an environment for reading and parsing templates
        self._template_env = jinja2.Environment(loader=self._template_loader)
        
        pass
    
    def render(self, template_name, template_vars):
        """Render code template."""
        # Select the right template file based on the template variable
        # template_filename = template_name + '.jinja'
        
        # Read the state template file into a template object using the environment object
        template = self._template_env.select_template([template_name, template_name + '.jinja'])
        
        # Render code
        code = template.render(**template_vars) 
        
        return code
    
    def render_all(self, state_name, template_vars):
        """Render all code templates for a given state."""
        # Compile regular expression to match all templates for the given state_name
        regex = re.compile(state_name + '_(.+)\.jinja')
        
        # Find templates matching the regex
        template_list = self._template_env.list_templates(filter_func = lambda template_name: re.match(regex, template_name))
        
        # Create a dict of {template types : template names} from the template names
        # (where template_type = template name stub: the template name without state name and without file extension)
        template_dict = {regex.match(template_name).group(1) : template_name for template_name in template_list}
        
        # Render template code for each of the templates
        template_code = {t_type : self.render(t_name, template_vars) for t_type, t_name in template_dict.items()}
        
        # Include the main body template, which does not get matched by the above.
        template_code['body'] = self.render(state_name, template_vars)
        
        return template_code
    

class Generator():
    """SMACH code generator."""
    def __init__(self, templater, verbose=False):
        self._verbose = verbose
        # Handle to the Jinja templater
        self._templater = templater
    
    def _process_state_machine(self, code_buffers, container_name, state):
        # Inspect state
        if type(state) is list:
            # Iterate through list of states
            for i_sub_state, sub_state in enumerate(state):
                # Recursively process each state
                code_buffers = self._process_state_machine(code_buffers, container_name, sub_state)
        
        elif type(state) is dict:
        
            # Check if the state is well-formed
            if len(state.items()) > 2:
                raise ParseException(error='Badly formed state!', line_number=state['__line__'])
            
            else:
                # Find the state name and variables
                for state_name, state_vars in state.items():
                    if state_name != '__line__':
                      break
                
                # If we have state_vars that contain a 'states' key,
                # we're dealing with a nested SMACH container.
                if 'states' in state_vars:
                    # Process and render state code from nested container template
                    try:
                        if self._verbose:
                            print(bcolors.OKGREEN +
                                  'Processing nested container state \'' + state_name + '\'' + bcolors.ENDC)
                        
                        # Create a new dictionary for the state template variables
                        template_vars = { 'name' : state_name } 
                        
                        # Add the other state variables to the template variables dictionary
                        for state_var, state_var_val in state_vars.items():
                            if state_var != 'states' and state_var != 'template' and state_var != '__line__':
                                template_vars[state_var] = state_var_val
                        
                        # Initialise list buffers for smach code generated from nested container states
                        container_code_buffers = dict()
                        container_code_buffers['base_header'] = list()
                        container_code_buffers['body'] = list()
                        container_code_buffers['base_footer'] = list()
                        
                        # Recursively process nested states
                        container_code_buffers = self._process_state_machine(container_code_buffers, state_name, state_vars['states'])
                        
                        # Convert nested state code to strings
                        template_vars['body'] = self._gen_code_string(container_code_buffers['body'])
                        
                        # Call the templater object to render the container templates with
                        # the generated nested state code
                        container_code = self._templater.render_all(state_vars['template'], template_vars)
                        
                        # Append relevant generated container code to respective 
                        if 'base_header' in container_code_buffers and 'base_header' in container_code:
                            container_code_buffers['base_header'].insert(0, container_code['base_header'])
                        if 'base_footer' in container_code_buffers and 'base_footer' in container_code:
                            container_code_buffers['base_footer'].append(container_code['base_footer'])
                        
                        # Append the generated container code buffers to the respective base code sections
                        if 'base_header' in code_buffers and 'base_header' in container_code_buffers:
                            code_buffers['base_header'].append(self._gen_code_string(container_code_buffers['base_header']))
                        if 'body' in code_buffers:
                            code_buffers['body'].append(container_code['body'])
                        if 'base_footer' in code_buffers and 'base_footer' in container_code_buffers:
                            code_buffers['base_footer'].insert(0, self._gen_code_string(container_code_buffers['base_footer']))
                    
                    except Exception as e:
                        print(bcolors.WARNING +
                              'WARNING: Error processing template for nested container state \'' + state_name + '\': ' + bcolors.ENDC +
                              str(e))
                        pass
                
                # Otherwise, assume we have hit a leaf.
                else:
                    # Process and render state code from leaf template
                    try:
                        if self._verbose:
                            print(bcolors.OKBLUE + 'Processing state \'' + state_name + '\'' + bcolors.ENDC)
                        
                        # Create a new dictionary for the state template variables
                        template_vars = { 'name' : state_name } 
                        
                        # Add the other state variables to the template variables dictionary
                        for state_var, state_var_val in state_vars.items():
                            if state_var != 'template' and state_var != '__line__':
                                template_vars[state_var] = state_var_val
                        
                        # Add the container name to template_vars so that states can
                        # refer to their parent containers in their templates
                        template_vars['container_name'] = container_name
                        
                        # Call the templater object to render the current state template
                        state_code = self._templater.render_all(state_vars['template'], template_vars)
                        
                        # Append the template for current state to smach_code buffer list
                        if 'base_header' in code_buffers and 'base_header' in state_code:
                            code_buffers['base_header'].append(state_code['base_header'])
                        if 'body' in code_buffers:
                            code_buffers['body'].append(state_code['body'])
                        if 'base_footer' in code_buffers and 'base_footer' in state_code:
                            code_buffers['base_footer'].insert(0, state_code['base_footer'])
                    
                    except Exception as e:
                      print(bcolors.WARNING +
                            'WARNING: Error processing template for state \'' + state_name + '\': ' + bcolors.ENDC +
                            str(e))
                      pass
        
        else:
            pass
        
        return code_buffers
    
    def _gen_code_string(self, code_buffer):
        """Generate code string from code list buffer."""
        code_string = ''
        for code_snippet in code_buffer:
            code_string = code_string + code_snippet + '\n\n'
        return code_string
    
    def run(self, script):
        """Generate SMACH code from a parsed SMACHA yaml script."""
        
        # TODO: Clean up this logic
        if type(script) is not dict:
            raise ParseException(error='Invalid script formatting!')
        elif 'states' not in script:
            raise ParseException(error='Script does not contain states!')
        else:
            # Start processing states from the script
            if self._verbose:
                print(bcolors.HEADER + 'Processing state machine' + bcolors.ENDC)
            
            # Initialise list buffers in which to store generated smach code
            base_code_buffers = dict()
            base_code_buffers['base_header'] = list()
            base_code_buffers['body'] = list()
            base_code_buffers['base_footer'] = list()
            base_code_buffers = self._process_state_machine(base_code_buffers, script['name'], script['states'])
            
            # Create and fill a dict for the base template variables
            base_template_vars = dict()
            base_template_vars['name'] = script['name']
            base_template_vars['node_name'] = script['node_name']
            base_template_vars['header'] = self._gen_code_string(base_code_buffers['base_header'])
            base_template_vars['body'] = self._gen_code_string(base_code_buffers['body'])
            base_template_vars['footer'] = self._gen_code_string(base_code_buffers['base_footer'])
            
            # Render the base state machine template
            base_code = self._templater.render(script['template'], base_template_vars)
            
            return base_code

def run(args):
    """Main SMACHA processing function."""
    
    # Parse arguments
    arg_parser = argparse.ArgumentParser(description='SMACHA: Jinja-based code templating, ' +
                                                     'generation and scripting for SMACH.')
    
    arg_parser.add_argument('script_file',
                            action='store',
                            help='SMACHA script (yaml file).')
    
    arg_parser.add_argument('-t', '--templates',
                            dest='template_dir',
                            action='store',
                            default='./templates',
                            help='SMACHA templates directory (jinja files).')
    
    arg_parser.add_argument('-o', '--output',
                            dest='output_file',
                            action='store',
                            default='./smacha_output.py',
                            help='Generated SMACH output (python file).')
    
    arg_parser.add_argument('-v', '--verbose',
                            action='store_true',
                            help='Print verbose output to terminal')
    
    args = arg_parser.parse_args()
    
    # Load parser
    parser = Parser()
    
    # Load and parse SMACHA script
    with open(args.script_file) as smacha_file:
        smacha_script = parser.parse(smacha_file)
    
    # Load template processor
    templater = Templater(args.template_dir)
    
    # Load code generator
    generator = Generator(templater, verbose=args.verbose)
    
    # Generate the SMACH code
    smach_code = generator.run(smacha_script)
    
    # Write the final output to a SMACH python file
    with open(args.output_file, 'w') as smach_file:
        smach_file.write(smach_code)
