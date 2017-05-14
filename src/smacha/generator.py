import smacha

from smacha.util import bcolors
from smacha.util import ParseException

__all__ = ['Generator']

class Generator():
    """SMACH code generator."""
    def __init__(self, templater, verbose=False):
        # Flag to enable verbose output to terminal 
        self._verbose = verbose

        # Handle to the Jinja templater
        self._templater = templater
    
    def _process_script(self, code_buffers, global_template_vars, parent_vars, script):
        """Recursively process parsed yaml SMACHA script."""
        # Inspect script for list of states
        if isinstance(script, list):
            # Iterate through list of states
            for state_script in script:
                # Recursively process each state
                code_buffers, global_template_vars = self._process_script(code_buffers,
                                                                          global_template_vars,
                                                                          parent_vars,
                                                                          state_script)
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
                        
                        # Initialise list buffers for smach code generated from nested container states
                        container_code_buffers = dict()
                        container_code_buffers['base_header'] = list()
                        container_code_buffers['header'] = list()
                        container_code_buffers['body'] = list()
                        container_code_buffers['footer'] = list()
                        container_code_buffers['base_footer'] = list()

                        # Add template name, template and type to child_vars
                        # NOTE: For now, child template will be the same as child type
                        # unless we're dealing with the base template. This may have to be accounted
                        # for in a neater way later.
                        child_vars = dict()
                        child_vars['name'] = state_name
                        child_vars['template'] = state_vars['template']
                        child_vars['type'] = state_vars['template']
                        
                        # Recursively process nested child states
                        container_code_buffers, global_template_vars = self._process_script(container_code_buffers,
                                                                                            global_template_vars,
                                                                                            child_vars,
                                                                                            state_vars['states'])
                        
                        # Convert nested state code to strings
                        template_vars['body'] = self._gen_code_string(container_code_buffers['body'])
                        
                        # Add the parent name, template and type to template_vars so that states can
                        # refer to their parent containers in their templates
                        template_vars['parent_name'] = parent_vars['name']
                        template_vars['parent_template'] = parent_vars['template']
                        template_vars['parent_type'] = parent_vars['type']

                        # Add the global_template_vars to template_vars so that child templates
                        # have access to variables defined in base templates
                        template_vars.update(global_template_vars)
                        
                        # Call the templater object to render the container templates with
                        # the generated nested state code
                        # container_code = self._templater.render_all(state_vars['template'], template_vars)
                        container_code = self._templater.render_all_blocks(state_vars['template'], template_vars)

                        # Update global_template_vars based on the child template
                        global_template_vars.update(self._templater.get_template_vars(state_vars['template'], template_vars))
                        
                        # Append relevant generated container code to respective 
                        if 'base_header' in container_code_buffers and 'base_header' in container_code:
                            container_code_buffers['base_header'].insert(0, container_code['base_header'])
                        if 'header' in container_code_buffers and 'header' in container_code:
                            container_code_buffers['header'].insert(0, container_code['header'])
                        if 'footer' in container_code_buffers and 'footer' in container_code:
                            container_code_buffers['footer'].append(container_code['footer'])
                        if 'base_footer' in container_code_buffers and 'base_footer' in container_code:
                            container_code_buffers['base_footer'].append(container_code['base_footer'])
                        
                        # Append the generated container code buffers to the respective base code sections
                        if 'base_header' in code_buffers and 'base_header' in container_code_buffers:
                            base_header_code = self._gen_code_string(container_code_buffers['base_header'])
                            if base_header_code != '':
                                code_buffers['base_header'].append(base_header_code)
                        if 'header' in code_buffers and 'header' in container_code_buffers:
                            header_code = self._gen_code_string(container_code_buffers['header'])
                            if header_code != '':
                                code_buffers['header'].append(header_code)
                        if 'body' in code_buffers and 'body' in container_code:
                            code_buffers['body'].append(container_code['body'])
                        if 'footer' in code_buffers and 'footer' in container_code_buffers:
                            footer_code = self._gen_code_string(container_code_buffers['footer'])
                            if footer_code != '':
                                code_buffers['footer'].insert(0, footer_code)
                        if 'base_footer' in code_buffers and 'base_footer' in container_code_buffers:
                            base_footer_code = self._gen_code_string(container_code_buffers['base_footer'])
                            if base_footer_code != '':
                                code_buffers['base_footer'].insert(0, base_footer_code)
                    
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
                        
                        # Add the parent name, template and type to template_vars so that states can
                        # refer to their parent containers in their templates
                        template_vars['parent_name'] = parent_vars['name']
                        template_vars['parent_template'] = parent_vars['template']
                        template_vars['parent_type'] = parent_vars['type']
                        
                        # Add the global_template_vars to template_vars so that child templates
                        # have access to variables defined in base templates
                        template_vars.update(global_template_vars)
                        
                        # Call the templater object to render the current state template
                        # state_code = self._templater.render_all(state_vars['template'], template_vars)
                        state_code = self._templater.render_all_blocks(state_vars['template'], template_vars)
                        
                        # Update global_template_vars based on the child template
                        global_template_vars.update(self._templater.get_template_vars(state_vars['template'], template_vars))
                        
                        # Append the template for current state to smach_code buffer list
                        if 'base_header' in code_buffers and 'base_header' in state_code and state_code['base_header'] != '':
                            code_buffers['base_header'].append(state_code['base_header'])
                        if 'header' in code_buffers and 'header' in state_code and state_code['header'] != '':
                            code_buffers['header'].append(state_code['header'])
                        if 'body' in code_buffers and 'body' in state_code and state_code['body'] != '':
                            code_buffers['body'].append(state_code['body'])
                        if 'footer' in code_buffers and 'footer' in state_code and state_code['footer'] != '':
                            code_buffers['footer'].insert(0, state_code['footer'])
                        if 'base_footer' in code_buffers and 'base_footer' in state_code and state_code['base_footer'] != '':
                            code_buffers['base_footer'].insert(0, state_code['base_footer'])
                    
                    except Exception as e:
                      print(bcolors.WARNING +
                            'WARNING: Error processing template for state \'' + state_name + '\': ' + bcolors.ENDC +
                            str(e))
                      pass
        
        else:
            pass
        
        return code_buffers, global_template_vars
    
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
            
            # Initialise list buffers in which to store generated smach code
            base_code_buffers = dict()
            base_code_buffers['base_header'] = list()
            base_code_buffers['body'] = list()
            base_code_buffers['base_footer'] = list()
            
            # Add any variables defined in the base template to global_template_vars
            global_template_vars = self._templater.get_template_vars(script['template'],
                                                                     context = {'base_header':'', 'body':'', 'base_footer':''})

            # Add base template name, template and type to parent_vars
            parent_vars = dict()
            parent_vars['name'] = script['name']
            parent_vars['template'] = script['template']
            parent_vars['type'] = 'StateMachine'
            
            # NOTE: For now, we explicitly state that the base is a parent of type
            # 'StateMachine' here.  This may have to be handled in a neater way later.
            base_code_buffers, global_template_vars = self._process_script(base_code_buffers,
                                                                           global_template_vars,
                                                                           parent_vars,
                                                                           script['states'])
            
            # Create and fill a dict for the base template variables
            #
            # TODO: Add exception handling for cases where certain template vars are necessary.
            #
            base_template_vars = dict()
            if 'name' in script:
                base_template_vars['name'] = script['name']
            if 'manifest' in script:
                base_template_vars['manifest'] = script['manifest']
            if 'node_name' in script:
                base_template_vars['node_name'] = script['node_name']
            if 'outcomes' in script:
                base_template_vars['outcomes'] = script['outcomes']
            if 'userdata' in script:
                base_template_vars['userdata'] = self._strip_line_numbers(script['userdata'])
            if 'base_header' in base_code_buffers:
                base_template_vars['base_header'] = self._gen_code_string(base_code_buffers['base_header']).strip()
            if 'header' in base_code_buffers:
                base_template_vars['header'] = self._gen_code_string(base_code_buffers['header']).strip()
            if 'body' in base_code_buffers:
                base_template_vars['body'] = self._gen_code_string(base_code_buffers['body']).strip()
            if 'header' in base_code_buffers:
                base_template_vars['footer'] = self._gen_code_string(base_code_buffers['footer']).strip()
            if 'base_footer' in base_code_buffers:
                base_template_vars['base_footer'] = self._gen_code_string(base_code_buffers['base_footer']).strip()

            # Add updated global_template_vars to base_template_vars
            base_template_vars.update(global_template_vars)

            # Render the base state machine template
            base_code = self._templater.render(script['template'], base_template_vars)
            
            return base_code

