# Jinja2 for templating & code generation
import jinja2
from jinja2.ext import Extension

import os
import re

__all__ = ['Templater']


# Credit to Alexander Todorov for this class that allows specified blocks
# to be either skipped or rendered in a given template.
#
# http://atodorov.org/blog/2014/02/21/skip-or-render-specific-blocks-from-jinja2-templates/
# 
class SkipBlockExtension(Extension):
    def __init__(self, environment):
        super(SkipBlockExtension, self).__init__(environment)
        environment.extend(skip_blocks=[])

    def filter_stream(self, stream):
        block_level = 0
        skip_level = 0
        in_endblock = False

        for token in stream:
            if (token.type == 'block_begin'):
                if (stream.current.value == 'block'):
                    block_level += 1
                    if (stream.look().value in self.environment.skip_blocks):
                        skip_level = block_level

            if (token.value == 'endblock' ):
                in_endblock = True

            if skip_level == 0:
                yield token

            if (token.type == 'block_end'):
                if in_endblock:
                    in_endblock = False
                    block_level -= 1

                    if skip_level == block_level+1:
                        skip_level = 0


class Templater():
    """Jinja template processor."""
    def __init__(self, template_dirs=[], include_comments=False):
        # Flag to enable rendering of header and footer comments in templates
        self._include_comments = include_comments

        # Create list of any custom user-defined template dirs + default template dir
        self._template_dirs = template_dirs + [os.path.dirname(__file__) + '/templates']

        # Create template loader for the template directories
        template_loaders = [jinja2.FileSystemLoader(template_dir) for template_dir in self._template_dirs]
        self._template_loader = jinja2.ChoiceLoader(template_loaders)

        # Create an environment for reading and parsing templates
        if self._include_comments:
            self._template_env = jinja2.Environment(loader=self._template_loader,
                                                    trim_blocks=True,
                                                    lstrip_blocks=True)
        else:
            self._template_env = jinja2.Environment(loader=self._template_loader,
                                                    extensions = [SkipBlockExtension],
                                                    trim_blocks=True,
                                                    lstrip_blocks=True)
            self._template_env.skip_blocks.append('header_comments')
            self._template_env.skip_blocks.append('footer_comments')
        
        pass
    
    def render(self, template_name, template_vars):
        """Render code template."""
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
