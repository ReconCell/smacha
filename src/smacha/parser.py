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

    def __init__(self, script_dirs=[]):
        """
        Constructor.

        Specifies roundtrip processing for ruamel.yaml by default so that
        comments and script structure can be retained.

        INPUTS:
            script_dirs: A list of directories in which to search for SMACHA scripts.

        RETURNS:
            N/A.
        """
        self._loader = yaml.RoundTripLoader
        self._dumper = yaml.RoundTripDumper
        self._script_dirs = script_dirs
        pass

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
            script: Either a file name (str) or script string (bytes)

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

    def dump(self, script_file, script):
        """
        Dump YAML script to file.

        INPUTS:
            script_file: Script filename (str).
            script: Script (dict).

        RETURNS:
            N/A.
        """
        with open(script_file, 'w') as script_file_handle:
            yaml.dump(script, script_file_handle,
                      Dumper = self._dumper, default_flow_style=False, default_style='')

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
    
