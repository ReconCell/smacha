# Jinja2 for templating & code generation
import jinja2
from jinja2.ext import Extension
from jinja2.tests import *

import os
import re
import inspect

__all__ = ['Templater']


class SkipBlockExtension(Extension):
    """
    This Jinja2 extension class allows specified blocks to be either skipped or rendered
    in a given template.
    
    Credit to Alexander Todorov:
    http://atodorov.org/blog/2014/02/21/skip-or-render-specific-blocks-from-jinja2-templates/
    """
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

def expression(value):
    """
    Custom test function to check if a template variable is an expression
    wrapped in backticks (e.g. `rospkg.RosPack().get_path('smacha')`).
    
    INPUTS:
        value: A string.

    RETURNS:
        True: If value is wrapped in backticks.
        False: If value is not wrapped in backticks.
    """
    if re.match('`.*`$', value):
        return True
    else:
        return False

def exptostr(value):
    """
    Custom filter to remove expression backtick wrappers from a template variable.
    
    INPUTS:
        value: A string wrapped in backticks.

    RETURNS:
        output: The string with backticks removed.
    """
    return re.sub(r'`(.*)`', r'\g<1>', value)

def not_string(value):
    """
    Custom test function to check if a template variable is not a string.

    If the value passed in is in fact a string, it first checks to see
    if it looks like a viable expression.  Thus, if it looks like a reference
    to an object member (i.e. contains any dots) or if it looks like a function
    call or object instantiation (i.e. contains an expression followed by wrapping
    parentheses, it returns true, otherwise it returns false.

    To force a string, the expression may be wrapped in backslash-escaped single quotes
    (e.g. \'expression\') in the SMACHA YAML script and the above behaviour will
    be overridden, i.e. this function will return false.

    INPUTS:
        value: A string.

    RETURNS:
        True: If value is not a string.
        False: If value is a string.
    """
    def is_number(s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    if isinstance(value, list):
        return True
    elif isinstance(value, dict):
        return True
    elif test_number(value):
        return True
    elif isinstance(value, str):
        if is_number(value):
            return True
        elif re.match('\'.*\'', value):
            return False
        # This one could, and should, be improved:
        elif re.match(r'\D+[.]\D+', value):
            return True
        elif re.match(r'.*\(.*\)', value):
            return True
   
    # Probably safest to fall back to a string assumption in most other cases:
    return False


class Templater():
    """
    Main SMACHA template processor.

    The templater uses Jinja2 as its main engine for processing templates,
    but it also provides some additional features and functionality.
    """

    def __init__(self, template_dirs=[], include_comments=False, include_introspection_server=False):
        """
        Constructor.

        Specifies roundtrip processing for ruamel.yaml by default so that
        comments and script structure can be retained.

        INPUTS:
            template_dirs: A list of directories in which to search for SMACHA templates.
            include_comments: A flag (bool) that specifies whether blocks marked as either 
                              'upper_comments' or 'lower_comments' should be included
                              in rendered templates.
            include_introspection_server: A flag (bool) that specifies whether code for including an
                                          introspection server should be included in rendered templates.
        RETURNS:
            N/A.
        """
        # Flag to enable rendering of header and footer comments in templates
        self._include_comments = include_comments

        # Flag to enable inclusion of introspection server (for use with smach_viewer)
        self._include_introspection_server = include_introspection_server

        # Create list of any custom user-defined template dirs + default template dir
        self._template_dirs = template_dirs + [os.path.join(os.path.dirname(os.path.realpath(__file__)), 'templates')]

        # Create template loader for the template directories
        template_loaders = [jinja2.FileSystemLoader(template_dir) for template_dir in self._template_dirs]
        self._template_loader = jinja2.ChoiceLoader(template_loaders)

        # Create an environment for reading and parsing templates, including
        # the SkipBlockExtension class to allow for skipping certain blocks.
        self._template_env = jinja2.Environment(loader=self._template_loader,
                                                extensions = [jinja2.ext.do, SkipBlockExtension],
                                                trim_blocks=False,
                                                lstrip_blocks=True)

        # Skip comment blocks as required
        if self._include_comments == False:
            self._template_env.skip_blocks.append('upper_comments')
            self._template_env.skip_blocks.append('lower_comments')
       
        # Skip introspection server blocks as required
        if self._include_introspection_server == False:
            self._template_env.skip_blocks.append('introspection_server')
            self._template_env.skip_blocks.append('spin')

        # Register custom tests with the environment
        self._template_env.tests['expression'] = expression
        self._template_env.tests['not_string'] = not_string
        
        # Register custom filters with the environment
        self._template_env.filters['exptostr'] = exptostr
        
        pass

    def list_templates(self):
        """
        List the available templates.

        INPUTS:
            None

        RETURNS:
            template_list: A list of names of templates.
        """
        # Compile regular expression to match all templates with '.tpl' extension,
        # while avoiding hidden files (files preceded with a full-stop, e.g. .MyTemplate.py.tpl.swp)
        regex = re.compile(r'(^[^\.].*)\.tpl(\.\w+)?$')
        
        # Find templates matching the regex
        template_list = self._template_env.list_templates(filter_func = lambda template_name: re.match(regex, template_name))

        return template_list

    def find_template_name(self, regex, template_env = None):
        """
        Find a template name in the available template list using a regular expression.

        INPUTS:
            regex: A regular expression used to search the list of templates (str).
            template_env: Optionally specify the template environment. Defaults to
                          current template_env member object. (jinja2.environment)

        RETURNS:
            template_name: The first template name found or an empty string if no template is found. (str)
        """
        # Select template_env
        if not template_env:
            template_env = self._template_env

        # Find templates matching the regex
        template_list = template_env.list_templates(filter_func = lambda template_name: re.match(regex, template_name))

        # Select the first match
        if template_list:
            return template_list[0]
        else:
            return ''

    def render(self, template_name, template_vars):
        """
        Render code template.

        INPUTS:
            template_name: The name of the template to be rendered (str).
            template_vars: The template variables (dict).

        RETURNS:
            code: The rendered template code (str).
        """
        # For reasons not entirely clear, a temporary environment must be created
        # to make this work.
        template_env = jinja2.Environment(loader=self._template_loader,
                                          extensions = [jinja2.ext.do, SkipBlockExtension],
                                          trim_blocks=False,
                                          lstrip_blocks=True)
        
        # Always skip the meta block
        template_env.skip_blocks.append('meta')

        # Skip comment blocks as required
        if self._include_comments == False:
            template_env.skip_blocks.append('upper_comments')
            template_env.skip_blocks.append('lower_comments')
        
        # Skip introspection server blocks as required
        if self._include_introspection_server == False:
            template_env.skip_blocks.append('introspection_server')
            template_env.skip_blocks.append('spin')

        # Register custom tests with the environment
        template_env.tests['expression'] = expression
        template_env.tests['not_string'] = not_string
        
        # Register custom filters with the environment
        template_env.filters['exptostr'] = exptostr

        # Read the state template file into a template object using the environment object
        found_template_name = self.find_template_name(template_name + '\.tpl(\.\w+)?$', template_env=template_env)
        template = template_env.select_template([template_name, found_template_name])
        
        # Render code
        code = template.render(**template_vars) 
        
        return code

    def render_block(self, template_name, template_vars, target_block):
        """
        Render specific block from code template.

        INPUTS:
            template_name: The name of the template (str).
            template_vars: The template variables (dict).
            target_block: The name of the block to be rendered (str).

        RETURNS:
            block_code: The rendered template block code (str).
        """
        # Read the state template file into a template object using the environment object
        found_template_name = self.find_template_name(template_name + '\.tpl(\.\w+)?$')
        template = self._template_env.select_template([template_name, found_template_name])

        # For reasons not entirely clear, a temporary environment must be created
        # to make this work.
        template_env = jinja2.Environment(loader=self._template_loader,
                                          extensions = [jinja2.ext.do, SkipBlockExtension],
                                          trim_blocks=False,
                                          lstrip_blocks=True)
        
        # Always skip the meta block unless it is the target block
        if target_block != 'meta':
            template_env.skip_blocks.append('meta')

        # Be sure to also skip comment blocks here as required
        if self._include_comments == False:
            template_env.skip_blocks.append('upper_comments')
            template_env.skip_blocks.append('lower_comments')
        
        # Be sure to also skip introspection server blocks here as required
        if self._include_introspection_server == False:
            template_env.skip_blocks.append('introspection_server')
            template_env.skip_blocks.append('spin')

        # Register custom tests with the environment
        template_env.tests['expression'] = expression
        template_env.tests['not_string'] = not_string
        
        # Register custom filters with the environment
        template_env.filters['exptostr'] = exptostr

        # Append non-target blocks to environment skip_blocks list
        for block_name, block in template.blocks.items():
            if block_name != target_block:
                template_env.skip_blocks.append(block_name)
       
        # Select the template from the temporary environment with
        # the appropriate skip_blocks list for non-target blocks
        found_template_name = self.find_template_name(template_name + '\.tpl(\.\w+)?$', template_env=template_env)
        target_block_template = template_env.select_template([template_name, found_template_name])
        
        # Render code for remaining block
        #
        # TODO: Raise exception here if there is no remaining block!
        #
        block_code = target_block_template.render(**template_vars)
        
        # Strip trailing whitespace from the block
        # block_code = block_code.rstrip()
        block_code = block_code.strip()
        
        if block_code == '':
            return block_code
        else:
            return block_code + '\n'

    def render_all_blocks(self, template_name, template_vars):
        """
        Render all blocks from code template.
        
        INPUTS:
            template_name: The name of the template (str).
            template_vars: The template variables (dict).

        RETURNS:
            template_block_code: The rendered code for each template block (dict).
        """
        # Read the state template file into a template object using the environment object
        found_template_name = self.find_template_name(template_name + '\.tpl(\.\w+)?$')
        template = self._template_env.select_template([template_name, found_template_name])

        # Render template code for each of the template blocks (all except the meta block)
        template_block_code = {block : self.render_block(template_name, template_vars, block) for block, _ in template.blocks.items() if block != 'meta'}

        return template_block_code
    
    def render_meta_block(self, template_name):
        """
        Render meta block from code template.
        
        INPUTS:
            template_name: The name of the template (str).

        RETURNS:
            meta_block_code: The rendered code for the template meta block (str).
        """
        # Read the state template file into a template object using the environment object
        found_template_name = self.find_template_name(template_name + '\.tpl(\.\w+)?$')
        template = self._template_env.select_template([template_name, found_template_name])

        # Generate a context placeholder
        context = template.new_context

        # Render template code for the template meta block 
        meta_block_code = template.blocks['meta'](context(vars={})).next()

        return meta_block_code

    def get_template_vars(self, template_name, context = None):
        """
        Get all variables defined in a template.
        
        INPUTS:
            template_name: The name of the template (str).
            context: The (optional) template context (dict).

        RETURNS:
            template_vars: The variables defined in the template (dict).
        """
        # Read the state template file into a template object using the environment object
        found_template_name = self.find_template_name(template_name + '\.tpl(\.\w+)?$')
        template = self._template_env.select_template([template_name, found_template_name])

        # Use Jinja2's module functionality to grab the template variables and create a dict comprehension
        if context is not None:
            template_module_vars = [template_var for template_var in dir(template.make_module(vars=context)) if not re.match('^_+.*', template_var)]
            template_vars = { template_var : getattr(template.make_module(vars=context), template_var) for template_var in template_module_vars }
        else:
            template_module_vars = [template_var for template_var in dir(template.module) if not re.match('^_+.*', template_var)]
            template_vars = { template_var : getattr(template.module, template_var) for template_var in template_module_vars }

        return template_vars
