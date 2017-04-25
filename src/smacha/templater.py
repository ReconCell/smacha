# Jinja2 for templating & code generation
import jinja2

import re

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
