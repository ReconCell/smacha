import os
from ruamel import yaml
from smacha.exceptions import ScriptNotFoundError

__all__ = ['Parser']

class Parser():
    """
    Main SMACHA YAML script parser class.

    The parser uses ruamel.yaml as its main engine for reading, parsing and
    writing YAML files, while also providing methods for interpreting
    SMACHA-specific script constructs.
    """

    def __init__(self, script_dirs=[],
                 container_persistent_vars =
                     ['params'],
                 sub_script_persistent_vars =
                     ['userdata', 'remapping', 'transitions']):
        """
        Constructor.

        Specifies roundtrip processing for ruamel.yaml by default so that
        comments and script structure can be retained.

        INPUTS:
            script_dirs: A list of directories in which to search for SMACHA scripts.
            container_persistent_vars: Names of variables that should persist from
                                       parent to child states (list).
            sub_script_persistent_vars: Names of variables that should persist from
                                        sub-script call to sub-script definition.

        RETURNS:
            N/A.
        """
        self._loader = yaml.RoundTripLoader
        self._dumper = yaml.RoundTripDumper
        self._script_dirs = script_dirs
        
        # Initialise a list of names of variables that should persist from parent to child states
        self._container_persistent_vars = container_persistent_vars
        
        # Initialise a list of names of variables that should persist from sub-script call
        # to sub-script definition.
        self._sub_script_persistent_vars = sub_script_persistent_vars

    def load(self, script):
        """
        Search for the specified YAML script file and load it.

        INPUTS:
            script: Either a file name (str) or a file handle to a SMACHA YAML script.

        RETURNS:
            contents: The file contents (str).
            filename: The file name (str).
        """
        def read_contents(filename):
            """Helper function for reading file content."""
            try:
                f = open(filename, 'rb')
            except IOError as e:
                f = None
                pass

            if f is None:
                return None
            try:
                contents = f.read()
            finally:
                f.close()
            
            return contents
    
        def select_script(names):
            """Helper function that loads the first loadable file in a list."""
            if not names:
                raise ScriptNotFoundError(names)

            for name in names:
                try:
                    return self.load(name)
                except ScriptNotFoundError:
                    pass
            raise ScriptNotFoundError(names)

        if isinstance(script, list):
            return select_script(script)

        elif os.path.isfile(script):
            filename = script
            contents = read_contents(filename)
            return contents, filename
        else:
            script_filename = script.split('/')[-1]
            for script_dir in self._script_dirs:
                filename = os.path.join(script_dir, script_filename)
                contents = read_contents(filename)
                if contents is None:
                    continue
                return contents, filename

        raise ScriptNotFoundError(script)

    def parse(self, script):
        """
        Parse YAML script.

        INPUTS:
            script: Either a file name (str) or script string (bytes).

        RETURNS:
            parsed_script: The parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap).
        """
        try:
            script_buffer, _ = self.load([script, script + '.yml'])
            parsed_script = yaml.load(script_buffer, Loader=self._loader)
        except Exception:
            try:
                parsed_script = yaml.load(script, Loader=self._loader)
            except Exception:
                raise ScriptNotFoundError(script)

        return parsed_script

    def dump(self, script, output_file=None):
        """
        Dump YAML script to string and (optionally) write to file.

        INPUTS:
            script: The parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap).
            output_file: Script filename (str).

        RETURNS:
            script_string: The rendered script (str).
        """
        if output_file:
            with open(output_file, 'w') as output_file_handle:
                script_string = yaml.dump(script, output_file_handle,
                                          Dumper = self._dumper, default_flow_style=False, default_style='')
        else:
            script_string = yaml.dump(script, None,
                                      Dumper = self._dumper, default_flow_style=False, default_style='')

        return script_string


    def contains_lookups(self, script_var, lookup_vars):
        """
        Check if a script variable contains variable lookups.

        INPUTS:
            script_var: An arbitrarily typed script variable.  E.g. it could be a string like 'robot' or
                        a dict like {'foo': 'bar'} or a list containing script variable lookups like
                        [['params', 'robot'], '/base'].
            lookup_vars: A list of names of possible lookup variables, e.g. ['params'].

        RETURNS:
            True: If script_var contains lookups.
            False: Otherwise.
        """
        try:
            if isinstance(script_var, list):
                try:
                    if (len(script_var) == 2 and isinstance(script_var[0], str) and isinstance(script_var[1], str) and
                        script_var[0] in lookup_vars):
                            return True
                    else:
                        raise Exception
                except:
                    for script_var_var in script_var:
                        if self.contains_lookups(script_var_var, lookup_vars):
                            return True
                        else:
                            continue
                    return False
        except:
            return False
    
    def lookup(self, script_vars, query):
        """
        Retrieve a variable from script_vars as specified by lookup query.
    
        Query should be either a 1 or 2-element list, e.g. ['params', 'robot'],
        in which case a robot name would be retrieved from the 'params'
        entry in 'script_vars'.

        INPUTS:
            script_vars: Script variables (dict).
            query: A 1-element or 2-element list of strings.
        
        RETURNS:
            script_vars[query[0]] if query is a 1-element list or
            script_vars[query[0]][query[1]] if querty is a 2-element list.
        """
        if not isinstance(query, list):
            raise ValueError('Query should be a list!')
        elif len(query) == 1:
            if not isinstance(query[0], str):
                raise ValueError('Query list should contain strings!')
            elif query[0] not in script_vars:
                raise ValueError('Query "{}" not found in script_vars!'.format(query[0]))
            else:
                return script_vars[query[0]]
        elif len(query) == 2:
            if not isinstance(query[0], str) or not isinstance(query[1], str):
                raise ValueError('script_vars query list should contain strings!')
            elif query[0] not in script_vars:
                raise ValueError('Query "{}" not found in script_vars!'.format(query[0]))
            elif query[1] not in script_vars[query[0]]:
                raise ValueError('Query "{}" not found in script_vars["{}"]!'.format(query[1], query[0]))
            else:
                return script_vars[query[0]][query[1]]
        else:
            raise ValueError('script_vars query should be a list of length 1 or 2!')
    
    
    def construct_string(self, script_vars, string_construct):
        """
        Construct a string given a list of script_vars lookups (specified as 1 or 2-element lists- see
        lookup() method) and/or strings.
        
        E.g. the string_construct ['/', ['params', 'robot'], '/joint_states']
        would return an output string '/robot_1/joint_states' if script_vars['params']['robot'] == 'robot_1'
        and '/robot_2/joint_states' if script_vars['params']['robot'] == 'robot_2'.
        
        INPUTS:
            script_vars: A dictionary of script/state variables.
            string_construct: A list of either script_vars lookups or strings describing how a string should
                              be constructed, as demonstrated in the above example.
        RETURNS:
            output_string: The constructed string is returned, as long as the string_construct can be parsed
                           successfully and as long as it contains at least one script_vars lookup.
            string_construct: If the string_construct can be parsed, but does not contain a lookup,
                              the original string_construct list is returned as-is. This is done
                              to avoid confusion with a genuine list.
        """
        output_string = ''
        contains_lookup = False
        if not isinstance(string_construct, list):
            raise ParsingError(error='String construct should be a list!')
        
        for string_part in string_construct:
            if isinstance(string_part, list):
                try:
                    output_string = output_string + str(self.lookup(script_vars, string_part))
                    contains_lookup = True
                except:
                    raise ParsingError(error='Could not parse script_vars lookup in string construct!')
            elif isinstance(string_part, str):
                output_string = output_string + string_part
            else:
                try:
                    output_string = output_string + str(string_part)
                except:
                    raise ParsingError(error='Could not convert string construct part to string!')

        if contains_lookup:    
            return output_string
        else:
            return string_construct
    
    def contain(self, script, container_name, container_type, states):
        """
        Convert a sequence of states in a script to a container state.

        NOTE: Currently only works for the 'StateMachine' container type!

        INPUTS:
            script: The parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap)
            container_name: A name for the container (str).
            container_type: The container type (str, e.g. 'StateMachine' or 'Concurrence') 
            states: The sequence of states to be contained (list of str's).

        RETURNS:
            script: Either a file name (str) or a file handle to a SMACHA YAML script.
        """
        #
        # Find the list of states to be contained
        #
        states_buffer = yaml.comments.CommentedSeq()
        i_state = 0
        i_script_container_state = 0
        for i_script_state, script_state in enumerate(script['states']):
            # Ensure we haven't run off a cliff
            if i_state >= len(states):
                break

            # Find the state name and variables in the current state
            script_state_name, script_state_vars = list(script_state.items())[0]

            # Record a script state buffer and the script state index of
            # the new container state.
            if script_state_name == states[i_state] and states_buffer:
                states_buffer.append(script_state)
                i_state = i_state + 1
            elif script_state_name != states[i_state] and states_buffer:
                raise ParsingError('State list does not match script state sequence!')
            elif script_state_name == states[i_state] and not states_buffer:
                states_buffer.append(script_state)
                i_script_container_state = i_script_state
                i_state = i_state + 1
            else:
                continue

        #
        # Construct skeleton of container state entry 
        #
        container_state_name = container_name
        container_state_vars = dict()
        container_state_vars['template'] = container_type
   
        # 
        # Generate new container state outcomes as appropriate and remap transitions
        #
        container_state_vars['outcomes'] = yaml.comments.CommentedSet()
        container_state_vars['transitions'] = yaml.comments.CommentedMap()
        outcome_map = yaml.comments.CommentedMap()
        for state in states_buffer:
            for outcome, transition in list(state.items())[0][1]['transitions'].items():
                if transition not in states:
                    # Add the transition to the outcome_map and container state outcomes if necessary
                    if transition not in outcome_map.keys():
                        new_container_outcome = container_state_name.lower() + '_outcome_' + str(len(outcome_map.keys()) + 1)
                        outcome_map[transition] = new_container_outcome
                        container_state_vars['outcomes'].add(new_container_outcome)
                        # Update the container transition
                        container_state_vars['transitions'][new_container_outcome] = transition
                    # Update the state transition
                    list(state.items())[0][1]['transitions'][outcome] = outcome_map[transition]
        container_state_vars['outcomes'] = yaml.comments.CommentedSeq(container_state_vars['outcomes'])

        #
        # Adjust transitions of all other states in script to point to container state as appropriate.
        #
        for state in script['states']:
            if list(state.items())[0][0] not in states:
                for outcome, transition in list(state.items())[0][1]['transitions'].items():
                    if transition in states:
                        list(state.items())[0][1]['transitions'][outcome] = container_state_name

        #
        # Handle container persistent variables by moving them outside of the container as appropriate.
        #
        for persistent_var in self._container_persistent_vars:
            # Create a dict for the persistent variable in container_state_vars if it doesn't exist yet.
            if persistent_var not in container_state_vars:
                container_state_vars[persistent_var] = yaml.comments.CommentedMap()

            for state in states_buffer:
                if persistent_var in list(state.items())[0][1]:
                    # We assume persistent_var is a dict of vars
                    for var, val in list(state.items())[0][1][persistent_var].items():
                        # If it doesn't yet exist in the container state, add it
                        if var not in container_state_vars[persistent_var].keys():
                            container_state_vars[persistent_var][var] = val
                        # If it does exist, but the var value is different, generate a new var key
                        elif container_state_vars[persistent_var][var] != val:
                            # Generate a new var
                            i_new_key = 1
                            while var + '_' + str(i_new_key) in container_state_vars[persistent_var][var]:
                                i_new_key = i_new_key + 1
                            container_state_vars[persistent_var][var + '_' + str(i_new_key)] = val
                        # If it exists, and the var value is the same, skip it
                        else:
                            continue
                    # Delete persistent variable from the state
                    list(state.items())[0][1].pop(persistent_var, 0)

            # Delete the persistent variable from container_state_vars if it's still empty
            if not container_state_vars[persistent_var]:
                container_state_vars.pop(persistent_var, 0)

        #
        # Handle userdata and remapping by moving certain userdata entries outside of the container as appropriate.
        #
        container_state_vars['remapping'] = yaml.comments.CommentedMap()

        # Create a dict for userdata in script_vars if it doesn't exist yet.
        if 'userdata' not in script:
            script['userdata'] = yaml.comments.CommentedMap()

        for state in states_buffer:
            if 'userdata' in list(state.items())[0][1]:
                for var, val in list(list(state.items())[0][1]['userdata'].items()):
                    # Userdata should only be lifted out of the container if it
                    # does not contain variable lookups.
                    if not self.contains_lookups(val, self._container_persistent_vars):
                        # If it doesn't yet exist in the script userdata, add it
                        if var not in script['userdata'].keys():
                            # Add it to script userdata
                            script['userdata'][var] = val

                            # Update the remapping
                            container_state_vars['remapping'][var] = var

                        # If it does exist, generate a new var key
                        else:
                            # Generate a new var
                            i_new_key = 1
                            while var + '_' + str(i_new_key) in script['userdata'][var]:
                                i_new_key = i_new_key + 1
                            script['userdata'][var + '_' + str(i_new_key)] = val

                            # Update the remapping
                            container_state_vars['remapping'][var] = var + '_' + str(i_new_key)

                        # Delete the variable from the userdata in the state
                        list(state.items())[0][1]['userdata'].pop(var, 0)

                # If the state userdata is now empty, delete it
                if not list(state.items())[0][1]['userdata']:
                    list(state.items())[0][1].pop('userdata', 0)
        
        # Delete the remapping variable from the container_state_vars if it's still empty
        if not container_state_vars['remapping']:
            container_state_vars.pop('remapping', 0)

        # Delete the userdata variable from the script if it's still empty
        if not script['userdata']:
            script.pop('userdata', 0)
        
        #
        # Handle input_keys and output_keys by cross-checking between userdata and remappings
        #
        preceding_userdata = dict()
        input_keys = yaml.comments.CommentedSet()
        output_keys = yaml.comments.CommentedSet()

        # Collate userdata from the parent script
        if 'userdata' in script:
            preceding_userdata.update(script['userdata'])

        # Collate userdata from preceding states
        for i_state in range(0,i_script_container_state):
            if 'userdata' in script['states'][i_state].items()[0][1]:
                preceding_userdata.update(script['states'][i_state].items()[0][1]['userdata'])

        for state in states_buffer:
            if 'remapping' in list(state.items())[0][1]:
                for var, val in list(state.items())[0][1]['remapping'].items():
                    # If val appears in the current state's userdata, it is neither an input nor an output key
                    if 'userdata' in list(state.items())[0][1] and val in list(state.items())[0][1]['userdata'].keys():
                        continue
                    # Otherwise, if val appears in the preceding userdata, we assume it must be an input key
                    elif val in preceding_userdata.keys():
                        input_keys.add(val)
                    # Otherwise, we assume it's an output key
                    else:
                        output_keys.add(val)

        # Add input_keys and output_keys to container_state_vars
        if input_keys:
            container_state_vars['input_keys'] = yaml.comments.CommentedSeq(input_keys)
        if output_keys:
            container_state_vars['output_keys'] = yaml.comments.CommentedSeq(output_keys)

        # Add states_buffer to container
        container_state_vars['states'] = states_buffer

        # Reorder the container_state_vars appropriately in a CommentedMap type
        ordered_container_state_vars = yaml.comments.CommentedMap()
        if 'template' in container_state_vars:
            ordered_container_state_vars['template'] = container_state_vars['template']
        if 'params' in container_state_vars:
            ordered_container_state_vars['params'] = container_state_vars['params']
        if 'userdata' in container_state_vars:
            ordered_container_state_vars['userdata'] = container_state_vars['userdata']
        if 'input_keys' in container_state_vars:
            ordered_container_state_vars['input_keys'] = container_state_vars['input_keys']
        if 'output_keys' in container_state_vars:
            ordered_container_state_vars['output_keys'] = container_state_vars['output_keys']
        if 'remapping' in container_state_vars:
            ordered_container_state_vars['remapping'] = container_state_vars['remapping']
        if 'outcomes' in container_state_vars:
            ordered_container_state_vars['outcomes'] = container_state_vars['outcomes']
        if 'transitions' in container_state_vars:
            ordered_container_state_vars['transitions'] = container_state_vars['transitions']
        if 'states' in container_state_vars:
            ordered_container_state_vars['states'] = container_state_vars['states']

        # Construct container state
        container_state = yaml.comments.CommentedMap()
        container_state[container_state_name] = ordered_container_state_vars

        # Remove old states from script and add container
        del script['states'][i_script_container_state+1:i_script_container_state+len(states_buffer)]
        script['states'][i_script_container_state] = container_state

        return script
