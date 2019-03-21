import jinja2
from jinja2 import meta
from jinja2.ext import Extension
from jinja2.tests import test_number

import os
import re

__all__ = ['Templater']


class SkipBlockExtension(Extension):
    """This Jinja2 extension class allows specified blocks to be either skipped
    or rendered in a given template.
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

            if (token.value == 'endblock'):
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
    """Custom test function to check if a template variable is an expression
    wrapped in backticks (e.g. `rospkg.RosPack().get_path('smacha')`).

    :param value: A string.
    :type value: str
    :return: True if value is wrapped in backticks, False otherwise.
    :rtype: bool
    """
    if re.match('`.*`$', value):
        return True
    else:
        return False


def exptostr(value):
    """Custom filter to remove expression backtick wrappers from a template
    variable.

    :param value: A string wrapped in backticks.
    :type value: str
    :return: The string with backticks removed.
    :rtype: str
    """
    return re.sub(r'`(.*)`', r'\g<1>', value)


def not_string(value):
    """Custom test function to check if a template variable is not a string.

    If the value passed in is in fact a string, it first checks to see if it
    looks like a viable expression. Thus, if it looks like a reference to an
    object member (i.e. contains any dots) or if it looks like a function call
    or object instantiation (i.e. contains an expression followed by wrapping
    parentheses, it returns true, otherwise it returns false.

    To force a string, the expression may be wrapped in single and double
    quotes (e.g. "'expression'") in the SMACHA YAML script and the above
    behaviour will be overridden, i.e. this function will return False.

    :param value: A string.
    :type value: str
    :return: True if value is not a string, False otherwise.
    :rtype: bool
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
        # This one could, and should, be improved:
        elif re.match(r'\D+[.]\D+', value):
            return True
        elif re.match(r'.*\(.*\)', value):
            return True

    # Probably safest to fall back to a string assumption in most other cases:
    return False


class Templater():
    """Main SMACHA template processor class.

    The templater uses Jinja2 as its main engine for processing templates,
    but it also provides some additional features and functionality.
    """

    def __init__(self, template_dirs=[], include_comments=False,
                 include_introspection_server=False):
        """Constructor.

        Specifies roundtrip processing for ruamel.yaml by default so that
        comments and script structure can be retained.

        :param template_dirs:
            A list of directories in which to search for SMACHA templates.
        :type template_dirs: list of str
        :param include_comments:
            A flag that specifies whether blocks marked as either
            ``'upper_comments'`` or ``'lower_comments'`` should be included
            in rendered templates.
        :type include_comments: bool
        """
        # Flag to enable rendering of header and footer comments in templates
        self._include_comments = include_comments

        # Flag to enable inclusion of introspection server (for use with
        # smach_viewer)
        self._include_introspection_server = include_introspection_server

        # Create list of any custom user-defined template dirs + default
        # template dir
        self._template_dirs = (
            template_dirs +
            [os.path.join(os.path.dirname(os.path.realpath(__file__)),
                          'templates')])

        # Create template loader for the template directories
        template_loaders = [
            jinja2.FileSystemLoader(template_dir)
            for template_dir in self._template_dirs]
        self._template_loader = jinja2.ChoiceLoader(template_loaders)

        # Create an environment for reading and parsing templates, including
        # the SkipBlockExtension class to allow for skipping certain blocks.
        self._template_env = (
            jinja2.Environment(loader=self._template_loader,
                               extensions=[jinja2.ext.do,
                                           SkipBlockExtension],
                               trim_blocks=False,
                               lstrip_blocks=True))

        # Skip comment blocks as required
        if not self._include_comments:
            self._template_env.skip_blocks.append('upper_comments')
            self._template_env.skip_blocks.append('lower_comments')

        # Skip introspection server blocks as required
        if not self._include_introspection_server:
            self._template_env.skip_blocks.append('introspection_server')
            self._template_env.skip_blocks.append('spin')

        # Register custom tests with the environment
        self._template_env.tests['expression'] = expression
        self._template_env.tests['not_string'] = not_string

        # Register custom filters with the environment
        self._template_env.filters['exptostr'] = exptostr

        # Create a template references cache dictionary
        # to be indexed by template names.
        self._template_ref_names_cache = {}

        # Create a template block names cache dictionary
        # to be indexed by template names.
        self._template_block_names_cache = {}

        # Create a template block cache dictionary
        # to be indexed by tuples of the form (template_name, block_name)
        self._template_block_cache = {}

        pass

    def list_templates(self):
        """List the available templates.

        :return: A list of names of templates.
        :rtype: list of str
        """
        # Compile regular expression to match all templates with '.tpl'
        # extension, while avoiding hidden files (files preceded with a
        # full-stop, e.g. .MyTemplate.py.tpl.swp)
        regex = re.compile(r'(^[^\.].*)\.tpl(\.\w+)?$')

        # Find templates matching the regex
        template_list = self._template_env.list_templates(
            filter_func=lambda template_name: re.match(regex, template_name))

        return template_list

    def find_template_name(self, regex, template_env=None):
        """Find a template name in the available template list using a regular
        expression.

        :param regex:
            A regular expression used to search the list of templates.
        :type regex: str
        :param template_env:
            Optionally specify the template environment. Defaults to
            current template_env member object.
        :type template_env: :class:`jinja2.environment`
        :return:
            The first template name found or an empty string if no template is
            found.
        :rtype: str
        """
        # Select template_env
        if not template_env:
            template_env = self._template_env

        # Find templates matching the regex
        template_list = template_env.list_templates(
            filter_func=lambda template_name: re.match(regex, template_name))

        # Select the first match
        if template_list:
            return template_list[0]
        else:
            return ''

    def render(self, template_name, template_vars):
        """Render code template.

        :param template_name: The name of the template to be rendered.
        :type template_name: str
        :param template_vars: The template variables.
        :type template_vars: dict
        :return: The rendered template code.
        :rtype: str
        """
        # For reasons not entirely clear, a temporary environment must be
        # created to make this work.
        template_env = jinja2.Environment(loader=self._template_loader,
                                          extensions=[jinja2.ext.do,
                                                      SkipBlockExtension],
                                          trim_blocks=False,
                                          lstrip_blocks=True)

        # Always skip the meta block
        template_env.skip_blocks.append('meta')

        # Skip comment blocks as required
        if not self._include_comments:
            template_env.skip_blocks.append('upper_comments')
            template_env.skip_blocks.append('lower_comments')

        # Skip introspection server blocks as required
        if not self._include_introspection_server:
            template_env.skip_blocks.append('introspection_server')
            template_env.skip_blocks.append('spin')

        # Register custom tests with the environment
        template_env.tests['expression'] = expression
        template_env.tests['not_string'] = not_string

        # Register custom filters with the environment
        template_env.filters['exptostr'] = exptostr

        # Read the state template file into a template object using the
        # environment object
        found_template_name = self.find_template_name(
            template_name + '\.tpl(\.\w+)?$', template_env=template_env)
        template = template_env.select_template(
            [template_name, found_template_name])

        # Render code
        code = template.render(**template_vars)

        return code

    def render_block(self, template_name, template_vars, target_block):
        """Render specific block from code template.

        :param template_name: The name of the template.
        :type template_name: str
        :param template_vars: The template variables.
        :type template_vars: dict
        :param target_block: The name of the block to be rendered.
        :type target_block: str
        :return: The rendered template block code.
        :rtype: str
        """
        try:
            # Check if this template/block combo has been previously cached
            target_block_template = self._template_block_cache[(template_name, target_block)]
        except:
            # Read the state template file into a template object using the
            # environment object
            found_template_name = (
                self.find_template_name(template_name + '\.tpl(\.\w+)?$'))
            template = self._template_env.select_template(
                [template_name, found_template_name])

            # For reasons not entirely clear, a temporary environment must be
            # created to make this work.
            template_env = jinja2.Environment(loader=self._template_loader,
                                            extensions=[jinja2.ext.do,
                                                        SkipBlockExtension],
                                            trim_blocks=False,
                                            lstrip_blocks=True)

            # Always skip the meta block unless it is the target block
            if target_block != 'meta':
                template_env.skip_blocks.append('meta')

            # Be sure to also skip comment blocks here as required
            if not self._include_comments:
                template_env.skip_blocks.append('upper_comments')
                template_env.skip_blocks.append('lower_comments')

            # Be sure to also skip introspection server blocks here as required
            if not self._include_introspection_server:
                template_env.skip_blocks.append('introspection_server')
                template_env.skip_blocks.append('spin')

            # Register custom tests with the environment
            template_env.tests['expression'] = expression
            template_env.tests['not_string'] = not_string

            # Register custom filters with the environment
            template_env.filters['exptostr'] = exptostr

            # Append non-target blocks to environment skip_blocks list
            for block in self.get_template_blocks(template_name):
                if block != target_block:
                    template_env.skip_blocks.append(block)

            # Select the template from the temporary environment with
            # the appropriate skip_blocks list for non-target blocks
            found_template_name = self.find_template_name(
                template_name + '\.tpl(\.\w+)?$', template_env=template_env)
            target_block_template = (
                template_env.select_template([template_name, found_template_name]))

            # Cache the block template for later use
            self._template_block_cache[(template_name, target_block)] = target_block_template

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
        """Render all blocks from code template.

        :param template_name: The name of the template.
        :type template_name: str
        :param template_vars: The template variables.
        :type template_vars: dict
        :return: The rendered code for each template block.
        :rtype: dict
        """
        # Read the state template file into a template object using the
        # environment object
        found_template_name = (
            self.find_template_name(template_name + '\.tpl(\.\w+)?$'))
        template = self._template_env.select_template(
            [template_name, found_template_name])

        # Render template code for each of the template blocks (all except the
        # meta block)
        template_block_code = {
            block: self.render_block(template_name, template_vars, block)
            for block in
            self.get_template_blocks(template_name) if block != 'meta'}

        return template_block_code

    def render_meta_block(self, template_name):
        """Render meta block from code template.

        :param template_name: The name of the template.
        :type template_name: str
        :return: The rendered code for the template meta block.
        :rtype: str
        """
        # Read the state template file into a template object using the
        # environment object
        found_template_name = (
            self.find_template_name(template_name + '\.tpl(\.\w+)?$'))
        template = self._template_env.select_template(
            [template_name, found_template_name])

        # Generate a context placeholder
        context = template.new_context

        # Render template code for the template meta block
        meta_block_code = template.blocks['meta'](context(vars={})).next()

        return meta_block_code

    def get_template_vars(self, template_name, context=None):
        """Get all variables defined in a template.

        :param template_name: The name of the template.
        :type template_name: str
        :param context: The (optional) template context.
        :type context: dict or None
        :return: The variables defined in the template.
        :rtype: dict
        """
        # Read the state template file into a template object using the
        # environment object
        found_template_name = (
            self.find_template_name(template_name + '\.tpl(\.\w+)?$'))
        template = self._template_env.select_template(
                [template_name, found_template_name])

        # Use Jinja2's module functionality to grab the template variables and
        # create a dict comprehension
        if context is not None:
            template_module_vars = [
                template_var for template_var in
                dir(template.make_module(vars=context))
                if not re.match('^_+.*', template_var)]
            template_vars = {
                template_var:
                getattr(template.make_module(vars=context), template_var)
                for template_var in template_module_vars}
        else:
            template_module_vars = [
                template_var for template_var in
                dir(template.module) if not re.match('^_+.*', template_var)]
            template_vars = {
                template_var:
                getattr(template.module, template_var)
                for template_var in template_module_vars}

        return template_vars

    def get_template_refs(self, template_name):
        """Get all templates referenced in a template.

        Get a list of all templates referenced via {% include %} or
        {% extends %} tags in a given template.

        :param template_name: The name of the template.
        :type template_name: str
        :return: The names of the templates referenced by the template.
        :rtype: list of str
        """
        try:
            # Check if the template refs for this template have been previously
            # cached.
            template_refs = self._template_ref_names_cache[template_name]
        except:
            # Read the template source using the template environment
            try:
                template_source = self._template_env.loader.get_source(
                    self._template_env, template_name)[0]
            except:
                try:
                    found_template_name = (
                        self.find_template_name(template_name + '\.tpl(\.\w+)?$'))
                    template_source = self._template_env.loader.get_source(
                        self._template_env, found_template_name)[0]
                except Exception as e:
                    raise e

            # Parse the content of the source
            parsed_content = self._template_env.parse(template_source)

            # Get the list of referenced templates using the Meta API
            template_refs = list(
                jinja2.meta.find_referenced_templates(parsed_content))

            # Cache the block template for later use
            self._template_ref_names_cache[template_name] = template_refs

        return template_refs

    def get_template_blocks(self, template_name):
        """Get all template blocks.

        Get a list of all blocks in a given template, either those defined
        directly in the template itself, or those defined in templates
        referenced via {% include %} or {% extends %} tags.

        :param template_name: The name of the template.
        :type template_name: str
        :return: The names of the template blocks.
        :rtype: set of str
        """
        try:
            # Check if the block names for this template have been previously
            # cached.
            template_blocks = self._template_block_names_cache[template_name]
        except:
            # Read the state template file into a template object using the
            # environment object
            found_template_name = (
                self.find_template_name(template_name + '\.tpl(\.\w+)?$'))
            template = self._template_env.select_template(
                    [template_name, found_template_name])

            # Get a list of the blocks defined directly in the template
            template_blocks = set(template.blocks.keys())

            # Recurse through referenced templates while updating the set
            for template_ref in self.get_template_refs(template_name):
                template_blocks.update(self.get_template_blocks(template_ref))

            # Cache the template block names for later use
            self._template_block_names_cache[template_name] = template_blocks

        return template_blocks
