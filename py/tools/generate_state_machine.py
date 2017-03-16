import yaml
import jinja2

# Load templates from the templates folder
script_dir_path = os.path.dirname(os.path.realpath(__file__))
search_path = os.path.join(script_dir_path, '../templates') 
template_loader = jinja2.FileSystemLoader(search_path)

# Create an environment for reading and parsing templates
template_env = jinja2.Environment( loader=template_loader )

# Load state machine script yaml file
smachgen_file = os.path.join(script_dir_path, '../statemachines/simple_state_machine_script_test.yml')
with open(smachgen_file) as fd:
  smachgen_script = yaml.load(fd)

# Parse smachgen script
for state, state_vars in smachgen_script.items():

  # Create a new dictionary for the state variables
  template_vars = { 'state_name' : state } 

  # Select the right template file based on the template variable
  template_file = os.path.join(state_vars['template'], '.jinja')

  # Read the template file using the environment object.
  # This also constructs our Template object.
  template = template_env.get_template( template_file )

  # Add the other state variables to the template variables dictionary
  for state_var, state_var_val in template_vars.items():
    if state_var != 'template':
      template_vars[state_var] = state_var_val
  
  # Finally, process the template to produce our final text.
  output_text = template.render( template_vars )

  data[state] =  output_text

print(data)

# print yaml.safe_dump(data, default_flow_style=False)
