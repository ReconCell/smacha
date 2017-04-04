import os
import yaml
import jinja2

# Load templates from the templates folder
script_dir_path = os.path.dirname(os.path.realpath(__file__))
search_path = os.path.join(script_dir_path, '../templates') 
template_loader = jinja2.FileSystemLoader(search_path)

# Create an environment for reading and parsing templates
template_env = jinja2.Environment( loader=template_loader )

# Load state machine script yaml file
smacha_filepath = os.path.join(script_dir_path, '../statemachines/simple_state_machine_script_test.yml')
with open(smacha_filepath) as smacha_file:
  smacha_script = yaml.load(smacha_file)

# Initialise a list in which to store generated smach code
smach_code = list()

# Parse smacha script
for i_state, state in enumerate(smacha_script):

  # Grab state name and state variables
  state_name, state_vars = state.items()[0]

  # Create a new dictionary for the state variables
  template_vars = { 'state_name' : state_name } 
  
  # Select the right template file based on the template variable
  template_filename = state_vars['template'] + '.jinja'
  
  # Read the state template file into a template object using the environment object
  template = template_env.get_template( template_filename )
  
  # Add the other state variables to the template variables dictionary
  for state_var, state_var_val in state_vars.items():
    if state_var != 'template':
      template_vars[state_var] = state_var_val
  
  # Render the template for current state to smach_code list
  smach_code.append(template.render( **template_vars ))

# Write the final output to a smach python file
smach_filepath = os.path.join(script_dir_path, '../statemachines/simple_state_machine_script_test.py')
with open(smach_filepath, 'w') as smach_file:
  for code_snippet in smach_code:
    smach_file.write(code_snippet)
    smach_file.write('\n\n')
