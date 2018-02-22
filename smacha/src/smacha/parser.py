import os
import copy
from ruamel import yaml
from smacha.exceptions import ScriptNotFoundError
from smacha.exceptions import ParsingError


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
                                          Dumper = self._dumper, default_flow_style=False, default_style='', width=1000)
        else:
            script_string = yaml.dump(script, None,
                                      Dumper = self._dumper, default_flow_style=False, default_style='', width=1000)

        return script_string


    def contains_lookups(self, script, lookup_vars):
        """
        Check if a script variable contains variable lookups.
        
        Given a script and a list of possible lookup variables, e.g. ['params'],
        recursively check to see if the script contains any lookups to those variables,
        e.g. regular lookups like ['params', 'robot'] or string constructs like [['params', 'robot'], '/base'].

        INPUTS:
            script: The parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap)
            lookup_vars: A list of names of possible lookup variables, e.g. ['params'].

        RETURNS:
            True: If script contains lookups.
            False: Otherwise.
        """
        try:
            if isinstance(script, list):
                try:
                    if (len(script) == 2 and isinstance(script[0], str) and isinstance(script[1], str) and
                        script[0] in lookup_vars):
                            return True
                    else:
                        raise Exception
                except:
                    for script_val in script:
                        if self.contains_lookups(script_val, lookup_vars):
                            return True
                        else:
                            continue
                    return False
            elif isinstance(script, dict):
                try:
                    for script_var, script_val in list(script.items()):
                        if self.contains_lookups(script_val, lookup_vars):
                            return True
                        else:
                            continue
                    return False
                except:
                    return False
            else:
                return False
        except:
            return False

    def sub_lookups(self, script, old_key, old_val, new_key, new_val):
        """
        Substitute lookup key/value names throughout a script.

        Given a script, recursively substitute [old_key, old_val] lookups
        with [new_key, new_val] lookups everywhere.

        NOTE: This method, in its current form, will modify the input script object!

        INPUTS:
            script: The parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap)
            old_key: The old_key in [old_key, old_val] lookups (str).
            old_val: The old_val in [old_key, old_val] lookups (str).
            new_key: The new_key in [new_key, new_val] lookups (str).
            new_val: The new_val in [new_key, new_val] lookups (str).

        RETURNS:
            script: The updated YAML script with substituted lookups.
        """
        try:
            if isinstance(script, list):
                try:
                    if (len(script) == 2 and isinstance(script[0], str) and isinstance(script[1], str) and
                        script[0] == old_key and script[1] == old_val):
                        # Perform the substitution
                        script[0] = new_key
                        script[1] = new_val
                    else:
                        raise Exception
                except:
                    try:
                        for script_val in script:
                            script_val = self.sub_lookups(script_val, old_key, old_val, new_key, new_val)
                    except:
                        raise
                return script
            elif isinstance(script, dict):
                try:
                    for script_var, script_val in list(script.items()):
                        script_val = self.sub_lookups(script_val, old_key, old_val, new_key, new_val)
                        script[script_var] = script_val
                except:
                    raise
                return script
            else:
                return script
        except:
            raise
    
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
    
    def contain(self, script, container_name, container_type, states, default_outcome_transition=None):
        """
        Convert a sequence of states in a script to a container state.

        INPUTS:
            script: The parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap)
            container_name: A name for the container (str).
            container_type: The container type (str, e.g. 'StateMachine' or 'Concurrence') 
            states: The sequence of states to be contained (list of str's).
            default_outcome_transition: The transition for the default_outcome associated with Concurrence containers.
                                        Set to container_name if None.

        RETURNS:
            script: Parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap) with
                    the specified states now contained in the container state.
        """
        #
        # Set defaults
        #
        default_outcome_transition = container_name 

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
        container_state_vars['outcomes'].fa.set_flow_style()
        container_state_vars['transitions'] = yaml.comments.CommentedMap()
        container_state_vars['transitions'].fa.set_flow_style()
        container_outcome_map = yaml.comments.CommentedMap()
        container_outcome_map.fa.set_flow_style()
        for state in states_buffer:
            for outcome, transition in list(state.items())[0][1]['transitions'].items():
                if transition not in states:
                    # Add the transition to the container_outcome_map and container state outcomes if necessary
                    if transition not in container_outcome_map.keys():
                        new_container_outcome = container_state_name.lower() + '_outcome_' + str(len(container_outcome_map.keys()) + 1)
                        container_outcome_map[transition] = new_container_outcome
                        container_state_vars['outcomes'].add(new_container_outcome)
                        # Update the container transition
                        container_state_vars['transitions'][new_container_outcome] = transition
                    # Update the state transition
                    list(state.items())[0][1]['transitions'][outcome] = container_outcome_map[transition]
        
        #
        # Generate an outcome_map for Concurrence containers
        #
        def find_state_outcome(container_outcome, state_outcome, state_transition):
            if state_transition not in states and state_transition == container_outcome:
                return state_outcome
            elif state_transition in states:
                state_found = False
                for state in states_buffer:
                    state_name = list(state.items())[0][0]
                    if state_name == state_transition:
                        state_found = True
                        try:
                            return find_state_outcome(container_outcome, state_outcome, list(state.items())[0][1]['transitions'][state_outcome])
                        except Exception as e:
                            raise ParsingError('Failed to successfully trace state transition to a state outcome when ' +
                                               'converting state sequence to Concurrence container: {}'.format(str(e)))
                        break
                if not state_found:
                    raise ParsingError('Failed to successfully trace state transition to a state outcome when ' +
                                       'converting state sequence to Concurrence container.')
            else:
                raise ParsingError('Failed to successfully trace state transition to a state outcome when ' +
                                   'converting state sequence to Concurrence container.')

        if container_type == 'Concurrence':
            container_state_vars['outcome_map'] = yaml.comments.CommentedMap()
            container_state_vars['outcome_map'].fa.set_flow_style()
            for outcome in container_state_vars['outcomes']:
                new_outcome_map = yaml.comments.CommentedMap()
                new_outcome_map.fa.set_flow_style()
                for state in states_buffer:
                    for state_outcome, state_transition in list(state.items())[0][1]['transitions'].items():
                        state_name = list(state.items())[0][0]
                        if state_transition not in states and state_transition == outcome:
                            new_outcome_map[state_name] = state_outcome
                        elif state_transition in states:
                            # Trace the state transition to a state outcome and use that
                            try:
                                new_outcome_map[state_name] = find_state_outcome(outcome, state_outcome, state_transition)
                            except:
                                continue
                            

                container_state_vars['outcome_map'][outcome] = new_outcome_map

        #
        # Remove transitions from all states in Concurrence containers
        #
        if container_type == 'Concurrence':
            for state in states_buffer:
                list(state.items())[0][1].pop('transitions', 0)


        #
        # Generate a default outcome for Concurrence containers
        #
        if container_type == 'Concurrence':
            # Update the outcomes list
            new_container_outcome = container_state_name.lower() + '_default_outcome'
            container_state_vars['outcomes'].add(new_container_outcome)
            # Create a default_outcome variable
            container_state_vars['default_outcome'] = new_container_outcome
            # Update the container transition
            container_state_vars['transitions'][new_container_outcome] = default_outcome_transition
        
        #
        # Convert outcomes from a set to a sequence
        #
        container_state_vars['outcomes'] = yaml.comments.CommentedSeq(container_state_vars['outcomes'])
        container_state_vars['outcomes'].fa.set_flow_style()

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
                container_state_vars[persistent_var].fa.set_flow_style()

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
                            while var + '_' + str(i_new_key) in container_state_vars[persistent_var]:
                                i_new_key = i_new_key + 1
                            container_state_vars[persistent_var][var + '_' + str(i_new_key)] = val

                            # Substitute every reference to the old key for the new key in the state
                            self.sub_lookups(state, persistent_var, var, persistent_var, var + '_' + str(i_new_key))

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
        container_state_vars['remapping'].fa.set_flow_style()

        # Create a dict for userdata in script_vars if it doesn't exist yet.
        # Insert it after outcomes.
        if 'userdata' not in script:
            script.insert(script.index('outcomes')+1, 'userdata', yaml.comments.CommentedMap())
            script['userdata'].fa.set_block_style()

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
                            while var + '_' + str(i_new_key) in script['userdata']:
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
        input_keys.fa.set_flow_style()
        output_keys = yaml.comments.CommentedSet()
        output_keys.fa.set_flow_style()

        # Collate userdata from the parent script
        if 'userdata' in script:
            preceding_userdata.update(script['userdata'])

        # Collate userdata from preceding states
        for i_state in range(0,i_script_container_state):
            if 'userdata' in list(script['states'][i_state].items())[0][1]:
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
            container_state_vars['input_keys'].fa.set_flow_style()
        if output_keys:
            container_state_vars['output_keys'] = yaml.comments.CommentedSeq(output_keys)
            container_state_vars['output_keys'].fa.set_flow_style()

        # Add states_buffer to container
        container_state_vars['states'] = states_buffer

        # Reorder the container_state_vars appropriately in a CommentedMap type
        ordered_container_state_vars = yaml.comments.CommentedMap()
        ordered_container_state_vars.fa.set_block_style()
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
        if 'default_outcome' in container_state_vars:
            ordered_container_state_vars['default_outcome'] = container_state_vars['default_outcome']
        if 'outcome_map' in container_state_vars:
            ordered_container_state_vars['outcome_map'] = container_state_vars['outcome_map']
        if 'transitions' in container_state_vars:
            ordered_container_state_vars['transitions'] = container_state_vars['transitions']
        if 'states' in container_state_vars:
            ordered_container_state_vars['states'] = container_state_vars['states']

        # Construct container state
        container_state = yaml.comments.CommentedMap()
        container_state.fa.set_block_style()
        container_state[container_state_name] = ordered_container_state_vars

        # Remove old states from script and add container
        del script['states'][i_script_container_state+1:i_script_container_state+len(states_buffer)]
        script['states'][i_script_container_state] = container_state

        return script

    def extract(self, script, container_state, sub_script_filename=None):
        """
        Extract a container state from a script and export to sub-script and super-script

        INPUTS:
            script: The parsed YAML script (dict or a ruamel type, e.g., ruamel.yaml.comments.CommentedMap)
            state: Name of container state to be extracted (str).
            sub_script_filename = Desired name for prospective sub-script file (str).

        RETURNS:
            sub_script: Generated SMACHA YAML sub-script for container state.
            super_script: Generated SMACHA YAML super-script that calls the sub-script.
        """
        # Find the container state
        for i_state, state in enumerate(script['states']):
            # Find the state name and variables in the current state
            state_name, state_vars = list(state.items())[0]

            # Check if it's the state we're looking for
            if state_name == container_state:
                break

        # Construct skeleton of super-script state entry 
        super_script_state_name = state_name
        super_script_state_vars = yaml.comments.CommentedMap()
        super_script_state_vars.fa.set_block_style()
        if sub_script_filename:
            super_script_state_vars['script'] = os.path.splitext(os.path.basename(sub_script_filename))[0]
        else:
            super_script_state_vars['script'] = state_name.lower()
        
        # Construct skeleton of sub-script
        sub_script_name = state_name
        sub_script_state_vars = state_vars

        # Move persistent variables from the container state to the super-script
        for state_var, state_var_val in list(state_vars.items()):
            if state_var in self._container_persistent_vars or state_var in self._sub_script_persistent_vars:
                # Add to super-script
                super_script_state_vars[state_var] = state_var_val
                # Remove from sub-script
                sub_script_state_vars.pop(state_var, 0)

        # Create the super-script state entry
        super_script_state = yaml.comments.CommentedMap()
        super_script_state.fa.set_block_style()
        super_script_state[super_script_state_name] = super_script_state_vars

        # Copy the script and transform to super-script
        super_script = copy.copy(script)
        super_script['states'][i_state] = super_script_state
        
        # Create the sub-script
        sub_script = yaml.comments.CommentedMap()
        sub_script.fa.set_block_style()
        sub_script[sub_script_name] = sub_script_state_vars

        return sub_script, super_script
