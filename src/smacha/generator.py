import smacha

from smacha.util import bcolors
from smacha.util import ParseException

__all__ = ['Generator']

class Generator():
    """SMACH code generator."""
    def __init__(self, templater, verbose=False,
                    base_vars =
                        ['name', 'manifest', 'function_name', 'node_name', 'outcomes', 'userdata'],
                    buffer_names =
                        ['base_header', 'imports', 'defs', 'class_defs', 'main_def',
                         'header', 'body', 'footer', 'execute', 'base_footer', 'main'],
                    container_insertion_order = 
                        ['prepend', 'prepend', 'prepend', 'prepend', 'prepend',
                         'prepend', 'append', 'append', 'append', 'append', 'append'],
                    buffer_insertion_order =
                        ['append', 'append', 'append', 'append', 'append',
                         'append', 'append', 'prepend', 'prepend', 'prepend', 'prepend']):
        # Flag to enable verbose output to terminal 
        self._verbose = verbose

        # Handle to the Jinja templater
        self._templater = templater

        # Initialise a list of names of expected base template variables
        self._base_vars = base_vars

        # Initialise a list of names of code buffers to be processed
        self._buffer_names = buffer_names

        #
        # TODO: Add exception handling below for input validation
        #
        # Initialise a dict of container buffer insertion order rules
        self._container_insertion_order = dict()
        for i_buffer, buffer_name in enumerate(self._buffer_names):
            self._container_insertion_order[buffer_name] = container_insertion_order[i_buffer]
        
        # Initialise a dict of buffer insertion order rules
        self._buffer_insertion_order = dict()
        for i_buffer, buffer_name in enumerate(self._buffer_names):
            self._buffer_insertion_order[buffer_name] = buffer_insertion_order[i_buffer]
    
    def _process_script(self, script, script_vars):
        """Recursively process parsed yaml SMACHA script."""
        # Inspect script for list of states
        if isinstance(script, list):
            # Iterate through list of states
            for state_script in script:
                # Recursively process each state
                script_vars = self._process_script(state_script, script_vars)

        # Inspect script for state dict
        elif isinstance(script, dict):
            # Check if the state script dict is well-formed
            if len(script.items()) > 2:
                raise ParseException(error='Badly formed state script!', line_number=script['__line__'])
            
            else:
                # Find the state name and variables in the state script
                for state_name, state_vars in script.items():
                    if state_name != '__line__':
                      break
                
                # If we have state_vars that contain a 'states' key,
                # we're dealing with a nested SMACH container.
                if 'states' in state_vars:
                    # Process and render state code from nested container template
                    try:
                        if self._verbose:
                            print(bcolors.OKGREEN +
                                  'Processing nested container state \'' + state_name + '\'' + bcolors.ENDC)
                        
                        # Create a new dictionary for the state template variables
                        template_vars = { 'name' : state_name } 
                        
                        # Add the other state variables to the template variables dictionary
                        for state_var, state_var_val in state_vars.items():
                            if state_var != 'states' and state_var != 'template' and state_var != '__line__':
                                template_vars[state_var] = self._strip_line_numbers(state_var_val)
                        
                        # Initialise dict of container script vars
                        container_script_vars = dict()
                        
                        # Add list buffers for smach code generated from nested container states
                        for buffer_name in self._buffer_names:
                            container_script_vars[buffer_name] = list()

                        # Add appropriate script_vars to container_script_vars so that child templates
                        # have access to variables defined in base templates
                        container_script_vars.update({x: script_vars[x] for x in script_vars if x not in self._buffer_names})
                        
                        # Add parent container template name, template and type
                        # NOTE: For now, parent template will be the same as parent type
                        # unless we're dealing with the base template. This may have to be accounted
                        # for in a neater way later.
                        container_script_vars['parent_name'] = state_name
                        container_script_vars['parent_template'] = state_vars['template']
                        container_script_vars['parent_type'] = state_vars['template']
                        
                        # Recursively process nested child states
                        container_script_vars = self._process_script(state_vars['states'], container_script_vars)
                        
                        # Convert nested state body code to string
                        template_vars['body'] = self._gen_code_string(container_script_vars['body'])
                        
                        # Add appropriate script_vars to template_vars so that child templates
                        # have access to variables defined in base templates
                        template_vars.update({x: script_vars[x] for x in script_vars if x not in self._buffer_names})
                        
                        # Call the templater object to render all blocks in the container template
                        container_code = self._templater.render_all_blocks(state_vars['template'], template_vars)

                        # Update script_vars based on the container state template
                        script_vars.update(self._templater.get_template_vars(state_vars['template'], template_vars))

                        # Add generated container code to respective container code buffers
                        for buffer_name, insertion_order in self._container_insertion_order.items():
                            if buffer_name != 'body':
                                if buffer_name in container_script_vars and buffer_name in container_code:
                                    if insertion_order == 'prepend':
                                        container_script_vars[buffer_name].insert(0, container_code[buffer_name])
                                    else:
                                        container_script_vars[buffer_name].append(container_code[buffer_name])
                        
                        # Generate code strings from container code buffers and add to respective parent code buffers
                        for buffer_name, insertion_order in self._buffer_insertion_order.items():
                            if buffer_name == 'body' and buffer_name in script_vars and buffer_name in container_code:
                                buffer_code = container_code[buffer_name]
                            elif buffer_name in script_vars and buffer_name in container_script_vars:
                                buffer_code = self._gen_code_string(container_script_vars[buffer_name])
                            else:
                                continue
                            if insertion_order == 'prepend':
                                script_vars[buffer_name].insert(0, buffer_code)
                            else:
                                script_vars[buffer_name].append(buffer_code)
                                        
                    
                    except Exception as e:
                        print(bcolors.WARNING +
                              'WARNING: Error processing template for nested container state \'' + state_name + '\': ' + bcolors.ENDC +
                              str(e))
                        pass
                
                # Otherwise, assume we have hit a leaf state
                else:
                    # Process and render state code from leaf template
                    try:
                        if self._verbose:
                            print(bcolors.OKBLUE + 'Processing state \'' + state_name + '\'' + bcolors.ENDC)
                        
                        # Create a new dictionary for the state template variables
                        template_vars = { 'name' : state_name } 
                        
                        # Add the other state variables to the template variables dictionary
                        for state_var, state_var_val in state_vars.items():
                            if state_var != 'template' and state_var != '__line__':
                                template_vars[state_var] = self._strip_line_numbers(state_var_val)
                        
                        # Add appropriate script_vars to template_vars so that the current leaf state template
                        # has access to variables defined in base templates
                        template_vars.update({x: script_vars[x] for x in script_vars if x not in self._buffer_names})
                        
                        # Call the templater object to render all blocks in the current leaf state template
                        state_code = self._templater.render_all_blocks(state_vars['template'], template_vars)

                        # Update script_vars based on the leaf state template
                        script_vars.update(self._templater.get_template_vars(state_vars['template'], template_vars))
                        
                        # Add generated code from leaf state code buffers to respective parent code buffers
                        for buffer_name, insertion_order in self._buffer_insertion_order.items():
                            if buffer_name in script_vars and buffer_name in state_code and state_code[buffer_name] != '':
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
    
    #
    # TODO: Refactor codebase to clean this up.
    #       I.e., the output of the parser should probably be
    #       a class unto itself that exposes methods such as this one.
    #
    def _strip_line_numbers(self, script):
        """Strip any line number keys from state_var_val."""
        try:
            script = dict(script)
            for script_key, script_val in script.items():
                if isinstance(script_val, dict):
                    script[script_key] = self._strip_line_numbers(script_val)
            del script['__line__']
            return script
        except:
            return script
    
    def _gen_code_string(self, code_buffer):
        """Generate code string from code list buffer."""
        code_string = ''
        for code_snippet in code_buffer:
            code_string = code_string + code_snippet + '\n'
        return code_string
    
    def run(self, script):
        """Generate SMACH code from a parsed SMACHA yaml script."""
        
        # TODO: Clean up this logic
        if not isinstance(script, dict):
            raise ParseException(error='Invalid script formatting!')
        elif 'states' not in script:
            raise ParseException(error='Script does not contain states!')
        else:
            # Start processing states from the script
            if self._verbose:
                print(bcolors.HEADER + 'Processing state machine' + bcolors.ENDC)

            # Initialise dict of variables needed for script processing
            script_vars = dict()
            
            # Add base parent template name, template and type
            # NOTE: For now, we explicitly state that the base is a parent of type
            # 'StateMachine' here.  This may have to be handled in a neater way later.
            script_vars['parent_name'] = script['name']
            script_vars['parent_template'] = script['template']
            script_vars['parent_type'] = 'StateMachine'
            
            # Add list buffers in which to store generated smach code
            script_vars.update({ x : list() for x in self._buffer_names })
            
            # Add any variables defined in the base template to script_vars
            #
            # TODO: Throw exception here if any of these clash with the canononical variables.
            # 
            script_vars.update(self._templater.get_template_vars(script['template'], context = { x : '' for x in self._buffer_names }))

            # Process base template states script
            script_vars = self._process_script(script['states'], script_vars)
            
            # Initialise a dict for the base template variables and code buffers
            base_template_vars = dict()

            # Add base template variables from script
            #
            # TODO: Add exception handling for cases where certain template vars are necessary.
            #
            base_template_vars.update({ x : self._strip_line_numbers(script[x]) for x in script if x in self._base_vars })

            # Generate code strings from the code buffers and add them to base_template_vars
            base_template_vars.update({ x : self._gen_code_string(script_vars[x]).strip() for x in script_vars if x in self._buffer_names })

            # Add updated script_vars to base_template_vars
            base_template_vars.update({ x : script_vars[x] for x in script_vars if x not in self._buffer_names })

            # Render the base state machine template
            base_code = self._templater.render(script['template'], base_template_vars)

            # Strip whitespace
            base_code = base_code.strip()
            
            return base_code

