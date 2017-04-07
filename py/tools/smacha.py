import argparse
import sys
import os

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
    self._template_env = jinja2.Environment( loader=self._template_loader )

    pass

  def process(self, template_name, template_vars):
    """Process code template."""
    # Select the right template file based on the template variable
    template_filename = template_name + '.jinja'
    
    # Read the state template file into a template object using the environment object
    template = self._template_env.get_template( template_filename )
    
    # Generate code
    code = template.render( **template_vars ) 

    return code
    

class Generator():
  """SMACH code generator."""
  def __init__(self, templater):
    # Handle to the Jinja templater
    self._templater = templater
    # Initialise a list in which to store generated smach code
    self._state_machine_code_buffer = list()

  def _process_state_machine(self, state):
    if type(state) is list:
      # Iterate through list of states
      for i_sub_state, sub_state in enumerate(state):
        # Recursively process each state
        self._process_state_machine(sub_state)

    elif type(state) is dict:

      # Check if the state is well-formed
      if len(state.items()) > 2:
        raise ParseException(error='Badly formed state!', line_number=state['__line__'])

      else:
        # Find the state name and variables
        for state_name, state_vars in state.items():
          if state_name is not '__line__':
            break

        # Create a new dictionary for the state template variables
        template_vars = { 'state_name' : state_name } 
          
        # If we have state_vars that contain a 'states' key,
        # we're dealing with a nested SMACH container.
        if 'states' in state_vars:
          # Recursively process the nested sub-states
          print(bcolors.OKGREEN +
                'Processing nested container state \'' + state_name + '\'' + bcolors.ENDC)
          self._process_state_machine(state_vars['states'])

        # Otherwise, assume we have hit a leaf.
        else:
          # Add the other state variables to the template variables dictionary
          for state_var, state_var_val in state_vars.items():
            if state_var is not 'template' and state_var is not '__line__':
              template_vars[state_var] = state_var_val

          # Process and render state code from template
          try:
            # Call the templater object to process the current state template
            print(bcolors.OKBLUE + 'Processing state \'' + state_name + '\'' + bcolors.ENDC)
            state_code = self._templater.process(state_vars['template'], template_vars)

            # Append the template for current state to smach_code buffer list
            self._state_machine_code_buffer.append(state_code)

          except Exception as e:
            print(bcolors.WARNING +
                  'WARNING: Error processing state \'' + state_name + '\': ' + bcolors.ENDC +
                  str(e))
            pass

    else:
      pass

  def run(self, script):
    # Clear the buffer
    self._state_machine_code_buffer = list()

    # TODO: Clean up this logic
    if type(script) is not dict:
      raise ParseException(error='Invalid script formatting!')
    elif 'states' not in script:
      raise ParseException(error='Script does not contain states!')
    else:
      # Start processing states from the script
      self._process_state_machine(script['states'])
  
    return self._state_machine_code_buffer


def main(args):
  """Main processing function."""

  # Load parser
  parser = Parser()

  # Load and parse smacha script
  with open(args.script_file) as smacha_file:
    smacha_script = parser.parse(smacha_file)

  # Load template processor
  templater = Templater(args.template_dir)

  # Load code generator
  generator = Generator(templater)

  # Generate the SMACH code
  smach_code = generator.run(smacha_script)
  
  # Write the final output to a smach python file
  # smach_filepath = os.path.join(script_dir_path, '../statemachines/simple_state_machine_script_test.py')
  with open(args.output_file, 'w') as smach_file:
    for code_snippet in smach_code:
      smach_file.write(code_snippet)
      smach_file.write('\n\n')


if __name__ == '__main__':
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

  args = arg_parser.parse_args()

  # Start the engine
  main(args)
