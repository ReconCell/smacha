import argparse
import sys
import os
import yaml
import jinja2


class Parser():
  """SMACHA script parser."""
  def __init__(self, script_buffer):
    self._script_buffer = script_buffer

    pass

  def parse(self):
    """Parse SMACHA yaml script."""
    return yaml.load(self._script_buffer)


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
  def __init__(self, script, templater):
    self._script = script
    self._templater = templater
    pass

  def process_state(self):
    pass

  def generate(self):
    # Initialise a list in which to store generated smach code
    smach_code = list()
    
    # Parse smacha script
    for i_state, state in enumerate(self._script):

      # Grab state name and state variables
      state_name, state_vars = state.items()[0]
    
      # Create a new dictionary for the state template variables
      template_vars = { 'state_name' : state_name } 
      
      # Add the other state variables to the template variables dictionary
      for state_var, state_var_val in state_vars.items():
        if state_var != 'template':
          template_vars[state_var] = state_var_val

      # Generate state code from template
      state_code = self._templater.process(state_vars['template'], template_vars)
      
      # Render the template for current state to smach_code list
      smach_code.append(state_code)
  
    return smach_code


def main(args):
  """Main processing function."""

  # Load and parse smacha script
  with open(args.script_file) as smacha_file:
    smacha_script = Parser(smacha_file).parse()

  # Load template processor
  templater = Templater(args.template_dir)

  # Load code generator
  generator = Generator(smacha_script, templater)

  # Generate the SMACH code
  smach_code = generator.generate()
  
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
