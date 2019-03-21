import smacha
import uuid

from smacha.util import bcolors
from smacha.exceptions import ParsingError

__all__ = ['Generator']


class Generator():
    """Main SMACHA code generator class.

    This class recursively processes the state machines described in SMACHA
    YAML scripts while generating executable code by rendering from templates
    that they reference.
    """
    def __init__(self, parser, templater, verbose=False,
                       base_vars =
                           ['name', 'manifest', 'function_name', 'node_name', 'outcomes', 'userdata'],
                       container_persistent_vars =
                           ['params'],
                       sub_script_persistent_vars =
                           ['userdata', 'remapping', 'transitions'],
                       buffer_names =
                           ['base_header', 'imports', 'defs', 'class_defs', 'cb_defs', 'main_def',
                            'header', 'body', 'footer', 'execute', 'base_footer', 'main'],
                       buffer_types =
                           ['list', 'list', 'list', 'list', 'list', 'list',
                            'dict', 'list', 'list', 'list', 'list', 'list'],
                       container_insertion_order = 
                           ['prepend', 'prepend', 'prepend', 'prepend', 'prepend', 'prepend',
                            'prepend', 'append', 'append', 'append', 'append', 'append'],
                       buffer_insertion_order =
                           ['append', 'append', 'append', 'append', 'append', 'append',
                            'append', 'append', 'prepend', 'prepend', 'prepend', 'prepend'],
                       local_var_lists = ['local_vars']):
        # TODO: Refactor this parameter mess!
        """Constructor.

        :param parser: SMACHA script parser object.
        :type parser: :class:`smacha.parser`
        :param templater: SMACHA templater object.
        :type templater: :class:`smacha.templater`
        :param verbose: A flag to enable verbose output to terminal.
        :type verbose: bool
        :param base_vars: Names of expected base template variables.
        :type base_vars: list of str
        :param container_persistent_vars:
            Names of variables that should persist from parent to child states.
        :type container_persistent_vars: list of str
        :param sub_script_persistent_vars:
            Names of variables that should persist from sub-script call to
            sub-script definition.
        :type sub_script_persistent_vars: list of str
        :param buffer_names:
            Names of code buffers (with respective template blocks) to be
            processed (n-dim list of strings).
        :param buffer_types:
            Types of code buffers to be processed ('list' or 'dict').
        :type buffer_types: list of str
        :param container_insertion_order:
            Container buffer insertion order rules ('append' or 'prepend').
        :type container_insertion_order: list of str
        :param buffer_insertion_order:
            Buffer insertion order rules ('append' or 'prepend').
        :type buffer_insertion_order: list of str
        :param local_var_lists:
            Local variable list names that are intended to contain variable
            names in templates that should be non-persistent between states
            (i.e. local).
        :type local_var_lists: list of str
        """
        # Flag to enable verbose output to terminal
        self._verbose = verbose

        # Handle to the YAML script parser
        self._parser = parser

        # Handle to the Jinja templater
        self._templater = templater

        # Initialise a list of names of expected base template variables
        self._base_vars = base_vars

        # Initialise a list of names of variables that should persist from
        # parent to child states
        self._container_persistent_vars = container_persistent_vars

        # Initialise a list of names of variables that should persist from
        # sub-script call to sub-script definition.
        self._sub_script_persistent_vars = sub_script_persistent_vars

        # Initialise a list of names of code buffers to be processed
        self._buffer_names = buffer_names

        # Initialise a list of types of code buffers to be processed
        self._buffer_types = buffer_types

        #
        # TODO: Review and appropriately fix all of this prepend/append
        # stuff. As of right now, they are hap-hazardly defined and
        # probably quite brittle. Hopefully the intention is clear though.
        #
        # TODO: Add exception handling below for input validation
        #
        # Initialise a dict of container buffer insertion order rules
        self._container_insertion_order = dict()
        for i_buffer, buffer_name in enumerate(self._buffer_names):
            self._container_insertion_order[buffer_name] = (
                container_insertion_order[i_buffer])

        # Initialise a dict of buffer insertion order rules
        self._buffer_insertion_order = dict()
        for i_buffer, buffer_name in enumerate(self._buffer_names):
            self._buffer_insertion_order[buffer_name] = (
                buffer_insertion_order[i_buffer])

        # Initialise a list of local variable list names (a singleton of
        # ['local_vars'] by default) that are intended to contain variable
        # names in templates that should be non-persistent between states (i.e.
        # local). Variables that are set in templates that are not contained in
        # these lists are assumed to be persistent between states (i.e. global)
        # by default.
        self._local_var_lists = local_var_lists

    def _process_script(self, script, script_vars):
        """Recursively process parsed SMACHA YAML script while updating script
        variables via template rendering.

        :param script: The parsed YAML script.
        :type script: dict or :class:`ruamel.yaml.comments.CommentedMap`
        :param script_vars: Script variables.
        :type script_vars: dict
        :return: Updated script variables.
        :rtype: dict
        """
        # Inspect script for list of states
        if isinstance(script, list):
            # Iterate through list of states
            for state_script in script:
                # Recursively process each state
                script_vars = self._process_script(state_script, script_vars)

        # Inspect script for state dict
        elif isinstance(script, dict):
            # Check if the state script dict is well-formed
            if len(list(script.items())) > 1:
                raise ParsingError(error='Badly formed state script!',
                                   line_number=script.lc.line)

            else:
                # Find the state name and variables in the state script
                state_name, state_vars = list(script.items())[0]

                # Add persistent state_vars from parent containers passed via
                # script_vars to state_vars.
                for persistent_var in self._container_persistent_vars:
                    if persistent_var in script_vars:
                        if persistent_var in state_vars:
                            state_vars[persistent_var].update(
                                {x: script_vars[persistent_var][x]
                                 for x in script_vars[persistent_var]
                                 if x not in state_vars[persistent_var]})
                        else:
                            state_vars[persistent_var] = (
                                script_vars[persistent_var])

                # Try to convert any state_vars lookups or string constructs
                # that might be present.
                for state_var, state_var_val in state_vars.items():
                    if isinstance(state_var_val, list):
                        try:
                            state_vars[state_var] = (
                                self._parser.lookup(state_vars, state_var_val))
                        except:
                            try:
                                state_vars[state_var] = (
                                    self._parser.construct_string(
                                        state_vars, state_var_val))
                            except:
                                # If no state_vars lookups or string constructs
                                # can be parsed from state_var, leave it as it
                                # is and continue.
                                continue
                    elif isinstance(state_var_val, dict):
                        for state_var_val_item, state_var_val_item_val in (
                                state_var_val.items()):
                            if isinstance(state_var_val_item_val, list):
                                try:
                                    state_vars[state_var][state_var_val_item] = (
                                        self._parser.lookup(state_vars,
                                                            state_var_val_item_val))
                                except:
                                    try:
                                        state_vars[state_var][state_var_val_item] = (
                                            self._parser.construct_string(state_vars,
                                                                          state_var_val_item_val))
                                    except:
                                        # If no state_vars lookups or string
                                        # constructs can be parsed from the
                                        # state_var dict item, leave it as it
                                        # is and continue.
                                        continue
                            else:
                                continue
                    else:
                        continue

                # If state_vars contains a 'states' key,
                # we're dealing with a nested SMACH container.
                if 'states' in state_vars:
                    # Process and render state code from nested container
                    # template
                    try:
                        if self._verbose:
                            print(bcolors.OKGREEN +
                                  'Processing nested container state \'' + state_name + '\'' + bcolors.ENDC)

                        # Create a new dictionary for the state template variables,
                        # initialized with the template name, state name and uuid.
                        template_vars = {'class_name': state_vars['template'], 'name': state_name, 'uuid': uuid.uuid4().hex}

                        # Add the other state variables to the template variables dictionary
                        for state_var, state_var_val in state_vars.items():
                            if state_var != 'states' and state_var != 'template':
                                template_vars[state_var] = state_var_val

                        # Initialise dict of container script vars
                        container_script_vars = dict()

                        # Add list buffers for smach code generated from nested container states
                        for buffer_name, buffer_type in zip(self._buffer_names, self._buffer_types):
                            if buffer_type == 'dict':
                                container_script_vars[buffer_name] = dict()
                            else:
                                container_script_vars[buffer_name] = list()

                        # Add appropriate script_vars to container_script_vars so that child templates
                        # have access to variables defined in base templates
                        container_script_vars.update({x: script_vars[x] for x in script_vars if x not in self._buffer_names})

                        # Add parent container template name, state machine name, template and type
                        # NOTE: For now, parent template will be the same as parent type
                        # unless we're dealing with the base template. This may have to be accounted
                        # for in a neater way later.
                        container_script_vars['parent_name'] = state_name
                        if 'sm_name' in state_vars:
                            container_script_vars['parent_sm_name'] = state_vars['sm_name']
                        else:
                            container_script_vars['parent_sm_name'] = 'sm_' + state_name.lower()
                        container_script_vars['parent_template'] = state_vars['template']
                        container_script_vars['parent_type'] = state_vars['template']

                        # Add persistent state_vars to container_script_vars.
                        # E.g. parameters that need to be passed between parent
                        # and child states.
                        container_script_vars.update({x: state_vars[x] for x in state_vars if x in self._container_persistent_vars})

                        # Recursively process nested child states
                        container_script_vars = (
                            self._process_script(state_vars['states'],
                                                 container_script_vars))

                        # Convert nested state body code to string
                        for buffer_name in self._buffer_names:
                            if isinstance(container_script_vars[buffer_name], dict):
                                template_vars[buffer_name] = dict()
                                for parent_key, buffer_val in container_script_vars[buffer_name].items():
                                    template_vars[buffer_name][parent_key] = self._gen_code_string(buffer_val)
                            else:
                                template_vars[buffer_name] = self._gen_code_string(container_script_vars[buffer_name])

                        # Add appropriate script_vars to template_vars so that
                        # child templates have access to variables defined in
                        # base templates
                        template_vars.update({x: script_vars[x] for x in script_vars if x not in self._buffer_names})

                        # Call the templater object to render all blocks in the container template
                        container_code = self._templater.render_all_blocks(state_vars['template'], template_vars)

                        # Update script_vars based on the container state
                        # template
                        template_vars_update = self._templater.get_template_vars(state_vars['template'], template_vars)
                        # Delete local variables
                        for local_var_list in self._local_var_lists:
                            if local_var_list in template_vars_update:
                                for local_var in template_vars_update[local_var_list]:
                                    if local_var in template_vars_update:
                                        del template_vars_update[local_var]
                        script_vars.update(template_vars_update)

                        # Add generated container code to respective container
                        # code buffers
                        for buffer_name, insertion_order in self._container_insertion_order.items():
                            if buffer_name in container_script_vars and buffer_name in container_code:
                                if isinstance(container_script_vars[buffer_name], dict):
                                    if state_name not in container_script_vars[buffer_name]:
                                        container_script_vars[buffer_name][state_name] = list()
                                    if insertion_order == 'prepend':
                                        container_script_vars[buffer_name][state_name].insert(0, container_code[buffer_name])
                                    else:
                                        container_script_vars[buffer_name][state_name].append(container_code[buffer_name])
                                else:
                                    if insertion_order == 'prepend':
                                        container_script_vars[buffer_name].insert(0, container_code[buffer_name])
                                    else:
                                        container_script_vars[buffer_name].append(container_code[buffer_name])

                        # Generate code strings from container code buffers and
                        # add to respective parent code buffers
                        for buffer_name, insertion_order in self._buffer_insertion_order.items():
                            if buffer_name in script_vars and buffer_name in container_code:
                                buffer_code = container_code[buffer_name]
                            elif buffer_name in script_vars and buffer_name in container_script_vars:
                                if isinstance(container_script_vars[buffer_name], dict):
                                    buffer_code = dict()
                                    for parent_key, buffer_val in container_script_vars[buffer_name].items():
                                        buffer_code[parent_key] = self._gen_code_string(buffer_val)
                                else:
                                    buffer_code = self._gen_code_string(container_script_vars[buffer_name])
                            else:
                                continue

                            if isinstance(script_vars[buffer_name], dict) and isinstance(buffer_code, dict):
                                for parent_key, buffer_val in buffer_code.items():
                                    if parent_key not in script_vars[buffer_name]:
                                        script_vars[buffer_name][parent_key] = list()
                                    if insertion_order == 'prepend':
                                        script_vars[buffer_name][parent_key].insert(0, buffer_val)
                                    else:
                                        script_vars[buffer_name][parent_key].append(buffer_val)
                            else:
                                if buffer_code != '':
                                    if insertion_order == 'prepend':
                                        script_vars[buffer_name].insert(0, buffer_code)
                                    else:
                                        script_vars[buffer_name].append(buffer_code)

                    except Exception as e:
                        print(bcolors.WARNING +
                              'WARNING: Error processing template for nested container state \'' + state_name + '\': ' + bcolors.ENDC +
                              str(e))
                        pass

                # If state_vars contains a 'script' key,
                # we're dealing with an included sub-script.
                #
                # TODO: Improve error-handling and input validation.
                elif 'script' in state_vars:
                    # Process included sub-script
                    try:
                        if self._verbose:
                            print(bcolors.OKBLUE + bcolors.BOLD + bcolors.UNDERLINE +
                                  'Processing state \'' + state_name + '\'' +
                                  ' with incluced sub-script \'' + state_vars['script'] + '\'' + bcolors.ENDC)

                        # Parse the included sub-script
                        sub_script = self._parser.parse(state_vars['script'])

                        # Ensure script is in longhand format
                        try:
                            sub_script = self._parser.sub_shorthand(sub_script)
                        except Exception as e:
                            raise ParsingError(error='Error when converting sub-script to longhand format: {}'.format(str(e)))

                        # Check its validity - it should only be a one-item list.
                        if not isinstance(sub_script, list):
                            raise ParsingError(error='Invalid script formatting- included script does not contain a list!')
                        elif len(sub_script) != 1:
                            raise ParsingError(error='Invalid script formatting- included script should only contain a single state!')
                        else:
                            sub_script = sub_script[0]
                            pass

                        # Find and replace the state name in the state sub-script
                        sub_script_state_name, sub_script_state_vars = list(sub_script.items())[0]
                        sub_script[state_name] = sub_script.pop(sub_script_state_name)

                        # Find and replace sub-script variables with current state variables
                        # (i.e. variables defined by the state that called the sub-script).
                        # If the variables have been defined in both the current state and the sub-script or
                        # if they have been defined in the current state and are marked as being persistent,
                        # then they should be replaced in the sub-script.
                        for state_var, state_var_val in state_vars.items():
                            if state_var in sub_script[state_name].keys() or state_var in self._sub_script_persistent_vars:
                                if state_var in sub_script[state_name].keys():
                                    sub_script[state_name][state_var].update(state_var_val)
                                else:
                                    sub_script[state_name][state_var] = state_var_val

                        # Add persistent state_vars to script_vars.
                        # E.g. parameters that need to be passed between parent and child states.
                        script_vars.update({x: state_vars[x] for x in state_vars if x in self._container_persistent_vars})

                        # Continue processing with the included sub-script now substituted in
                        # for the current state with potentially re-defined state variables
                        # appropriately remapped.
                        script_vars = (
                            self._process_script(sub_script,
                                                 script_vars))

                    except Exception as e:
                        print(bcolors.WARNING +
                              'WARNING: Error processing included script for state \'' + state_name + '\': ' + bcolors.ENDC +
                              str(e))
                        pass

                # Otherwise, assume we have hit a leaf state
                else:
                    # Process and render state code from leaf template
                    try:
                        if self._verbose:
                            print(bcolors.OKBLUE + 'Processing state \'' + state_name + '\'' + bcolors.ENDC)

                        # Create a new dictionary for the state template variables,
                        # initialized with the template name, state name and uuid.
                        template_vars = {'class_name': state_vars['template'], 'name': state_name, 'uuid': uuid.uuid4().hex}

                        # Add the other state variables to the template variables dictionary
                        for state_var, state_var_val in state_vars.items():
                            if state_var != 'template':
                                template_vars[state_var] = state_var_val

                        # Add appropriate script_vars to template_vars so that the current leaf state template
                        # has access to variables defined in base templates
                        template_vars.update({x: script_vars[x] for x in script_vars if x not in self._buffer_names})

                        # Call the templater object to render all blocks in the current leaf state template
                        state_code = self._templater.render_all_blocks(state_vars['template'], template_vars)

                        # Update script_vars based on the leaf state template
                        template_vars_update = self._templater.get_template_vars(state_vars['template'], template_vars)
                        # Delete local variables
                        for local_var_list in self._local_var_lists:
                            if local_var_list in template_vars_update:
                                for local_var in template_vars_update[local_var_list]:
                                    if local_var in template_vars_update:
                                        del template_vars_update[local_var]
                        script_vars.update(template_vars_update)

                        # Add generated code from leaf state code buffers to respective parent code buffers
                        for buffer_name, insertion_order in self._buffer_insertion_order.items():
                            if buffer_name in script_vars and buffer_name in state_code and state_code[buffer_name] != '':
                                if isinstance(script_vars[buffer_name], dict):
                                    if script_vars['parent_name'] not in script_vars[buffer_name]:
                                        script_vars[buffer_name][script_vars['parent_name']] = list()
                                    if insertion_order == 'prepend':
                                        script_vars[buffer_name][script_vars['parent_name']].insert(0, state_code[buffer_name])
                                    else:
                                        script_vars[buffer_name][script_vars['parent_name']].append(state_code[buffer_name])
                                else:
                                    if insertion_order == 'prepend':
                                        script_vars[buffer_name].insert(0, state_code[buffer_name])
                                    else:
                                        script_vars[buffer_name].append(state_code[buffer_name])

                    except Exception as e:
                      print(bcolors.WARNING +
                            'WARNING: Error processing template for state \'' + state_name + '\': ' + bcolors.ENDC +
                            str(e))
                      pass

        else:
            pass

        return script_vars

    def _gen_code_string(self, code_buffer):
        """Generate code string from code list buffer.

        :param code_buffer: A list buffer of code strings.
        :type code_buffer: list of str
        :return: A concatenation of the strings in the list buffer.
        :rtype: str
        """
        code_string = ''
        for code_snippet in code_buffer:
            code_string = code_string + code_snippet + '\n'
        return code_string

    def run(self, script):
        """Generate SMACH code from a parsed SMACHA yaml script.

        :param script: The parsed YAML script.
        :type script: dict or :class:`ruamel.yaml.comments.CommentedMap`
        :return: The generated code.
        :rtype: str
        """
        # Ensure script is in longhand format
        try:
            script = self._parser.sub_shorthand(script)
        except Exception as e:
            raise ParsingError(error='Error when converting script to longhand format: {}'.format(str(e)))

        # TODO: Clean up this logic
        if not isinstance(script, dict):
            raise ParsingError(error='Invalid script formatting!')
        elif 'states' not in script:
            raise ParsingError(error='Script does not contain states!')
        else:
            # Start processing states from the script
            if self._verbose:
                print(bcolors.HEADER + bcolors.BOLD + bcolors.UNDERLINE +
                      'Processing state machine' + bcolors.ENDC)

            # Initialise dict of variables needed for script processing
            script_vars = dict()

            # Add base parent template name, state machine name, template and
            # type NOTE: For now, we explicitly state that the base is a parent
            # of type 'StateMachine' here. This may have to be handled in a
            # neater way later.
            script_vars['parent_name'] = script['name']
            if 'sm_name' in script_vars:
                script_vars['parent_sm_name'] = script_vars['sm_name']
            else:
                script_vars['parent_sm_name'] = script['name'].lower()
            script_vars['parent_template'] = script['template']
            script_vars['parent_type'] = 'StateMachine'

            # Add list buffers in which to store generated smach code
            # script_vars.update({ x : list() for x in self._buffer_names })
            for buffer_name, buffer_type in (
                    zip(self._buffer_names, self._buffer_types)):
                if buffer_type == 'dict':
                    script_vars[buffer_name] = dict()
                else:
                    script_vars[buffer_name] = list()

            # Add any variables defined in the base template to script_vars
            #
            # TODO: Throw exception here if any of these clash with the
            # canononical variables.
            #
            # Create context
            context = dict()
            context['name'] = script['name']
            for buffer_name, buffer_type in (
                    zip(self._buffer_names, self._buffer_types)):
                if buffer_type == 'dict':
                    context[buffer_name] = dict()
                    context[buffer_name][script['name']] = ''
                else:
                    context[buffer_name] = ''
            # Get template vars using context
            template_vars_update = self._templater.get_template_vars(script['template'], context=context)
            # Delete local variables
            for local_var_list in self._local_var_lists:
                if local_var_list in template_vars_update:
                    for local_var in template_vars_update[local_var_list]:
                        if local_var in template_vars_update:
                            del template_vars_update[local_var]
            script_vars.update(template_vars_update)

            # Process base template states script
            script_vars = self._process_script(script['states'],
                                               script_vars)

            # Initialise a dict for the base template variables and code
            # buffers
            base_template_vars = dict()

            # Add base template variables from script
            #
            # TODO: Add exception handling for cases where certain template
            # vars are necessary.
            #
            base_template_vars.update({x : script[x] for x in script if x in self._base_vars})

            # Generate code strings from the code buffers and add them to
            # base_template_vars
            for script_var_name, script_var_val in script_vars.items():
                if script_var_name in self._buffer_names:
                    if isinstance(script_var_val, dict):
                        base_template_vars[script_var_name] = dict()
                        for parent_name, buffer_val in script_var_val.items():
                            base_template_vars[script_var_name][parent_name] = self._gen_code_string(buffer_val).strip()
                    else:
                        base_template_vars[script_var_name] = self._gen_code_string(script_var_val).strip()

            # Add updated script_vars to base_template_vars
            base_template_vars.update({x: script_vars[x] for x in script_vars if x not in self._buffer_names})

            # Render the base state machine template
            base_code = self._templater.render(script['template'],
                                               base_template_vars)

            # Strip whitespace
            base_code = base_code.strip()

            return base_code
