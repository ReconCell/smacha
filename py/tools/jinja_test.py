import os

# Load the jinja library's namespace into the current module.
import jinja2

# Load templates from the templates folder
script_dir_path = os.path.dirname(os.path.realpath(__file__))
search_path = os.path.join(script_dir_path, '../templates') 
template_loader = jinja2.FileSystemLoader(search_path)

# Create an environment for reading and parsing templates
template_env = jinja2.Environment( loader=template_loader )

# Read the template file using the environment object.
# This also constructs our Template object.
readtransformstate_template = template_env.get_template( 'ReadTransformState.jinja' )

# Specify any input variables to the template as a dictionary.
template_vars = { 'state_name' : 'READ_HEXAPOD_CURRENT_POSE',
                  'target_frame' : 'ur10_1/base',
                  'source_frame' : 'hexapod_1/top',
                  'output_name' : 'hexapod_current_pose',
                  'next_state_name' : 'MOVE_ABOVE_HEXAPOD_1'}

# Finally, process the template to produce our final text.
output_text = readtransformstate_template.render( template_vars )

print(output_text)
