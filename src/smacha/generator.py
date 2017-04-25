import smacha

from smacha.util import bcolors
from smacha.util import ParseException

class Generator():
    """SMACH code generator."""
    def __init__(self, templater, verbose=False):
        self._verbose = verbose
        # Handle to the Jinja templater
        self._templater = templater
    
    def _process_state_machine(self, code_buffers, container_name, state):
        # Inspect state
        if type(state) is list:
            # Iterate through list of states
            for i_sub_state, sub_state in enumerate(state):
                # Recursively process each state
                code_buffers = self._process_state_machine(code_buffers, container_name, sub_state)
        
        elif type(state) is dict:
        
            # Check if the state is well-formed
            if len(state.items()) > 2:
                raise ParseException(error='Badly formed state!', line_number=state['__line__'])
            
            else:
                # Find the state name and variables
                for state_name, state_vars in state.items():
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
                                template_vars[state_var] = state_var_val
                        
                        # Initialise list buffers for smach code generated from nested container states
                        container_code_buffers = dict()
                        container_code_buffers['base_header'] = list()
                        container_code_buffers['body'] = list()
                        container_code_buffers['base_footer'] = list()
                        
                        # Recursively process nested states
                        container_code_buffers = self._process_state_machine(container_code_buffers, state_name, state_vars['states'])
                        
                        # Convert nested state code to strings
                        template_vars['body'] = self._gen_code_string(container_code_buffers['body'])
                        
                        # Call the templater object to render the container templates with
                        # the generated nested state code
                        container_code = self._templater.render_all(state_vars['template'], template_vars)
                        
                        # Append relevant generated container code to respective 
                        if 'base_header' in container_code_buffers and 'base_header' in container_code:
                            container_code_buffers['base_header'].insert(0, container_code['base_header'])
                        if 'base_footer' in container_code_buffers and 'base_footer' in container_code:
                            container_code_buffers['base_footer'].append(container_code['base_footer'])
                        
                        # Append the generated container code buffers to the respective base code sections
                        if 'base_header' in code_buffers and 'base_header' in container_code_buffers:
                            code_buffers['base_header'].append(self._gen_code_string(container_code_buffers['base_header']))
                        if 'body' in code_buffers:
                            code_buffers['body'].append(container_code['body'])
                        if 'base_footer' in code_buffers and 'base_footer' in container_code_buffers:
                            code_buffers['base_footer'].insert(0, self._gen_code_string(container_code_buffers['base_footer']))
                    
                    except Exception as e:
                        print(bcolors.WARNING +
                              'WARNING: Error processing template for nested container state \'' + state_name + '\': ' + bcolors.ENDC +
                              str(e))
                        pass
                
                # Otherwise, assume we have hit a leaf.
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
                                template_vars[state_var] = state_var_val
                        
                        # Add the container name to template_vars so that states can
                        # refer to their parent containers in their templates
                        template_vars['container_name'] = container_name
                        
                        # Call the templater object to render the current state template
                        state_code = self._templater.render_all(state_vars['template'], template_vars)
                        
                        # Append the template for current state to smach_code buffer list
                        if 'base_header' in code_buffers and 'base_header' in state_code:
                            code_buffers['base_header'].append(state_code['base_header'])
                        if 'body' in code_buffers:
                            code_buffers['body'].append(state_code['body'])
                        if 'base_footer' in code_buffers and 'base_footer' in state_code:
                            code_buffers['base_footer'].insert(0, state_code['base_footer'])
                    
                    except Exception as e:
                      print(bcolors.WARNING +
                            'WARNING: Error processing template for state \'' + state_name + '\': ' + bcolors.ENDC +
                            str(e))
                      pass
        
        else:
            pass
        
        return code_buffers
    
    def _gen_code_string(self, code_buffer):
        """Generate code string from code list buffer."""
        code_string = ''
        for code_snippet in code_buffer:
            code_string = code_string + code_snippet + '\n\n'
        return code_string
    
    def run(self, script):
        """Generate SMACH code from a parsed SMACHA yaml script."""
        
        # TODO: Clean up this logic
        if type(script) is not dict:
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
            base_code_buffers = self._process_state_machine(base_code_buffers, script['name'], script['states'])
            
            # Create and fill a dict for the base template variables
            base_template_vars = dict()
            base_template_vars['name'] = script['name']
            base_template_vars['node_name'] = script['node_name']
            base_template_vars['header'] = self._gen_code_string(base_code_buffers['base_header'])
            base_template_vars['body'] = self._gen_code_string(base_code_buffers['body'])
            base_template_vars['footer'] = self._gen_code_string(base_code_buffers['base_footer'])
            
            # Render the base state machine template
            base_code = self._templater.render(script['template'], base_template_vars)
            
            return base_code

