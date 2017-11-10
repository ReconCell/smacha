import os
from ruamel import yaml
from smacha.exceptions import ScriptNotFoundError

__all__ = ['Parser']

class Parser():
    """YAML script parser."""

    def __init__(self, script_dirs=[]):
        self._loader = yaml.RoundTripLoader
        self._dumper = yaml.RoundTripDumper
        self._script_dirs = script_dirs
        pass

    def load_script(self, script):
        """Search for the specified YAML script file and load it."""
        def read_contents(filename):
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

        if os.path.isfile(script):
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

    def select_script(self, names):
        if not names:
            raise ScriptNotFoundError(names)

        for name in names:
            try:
                return self.load_script(name)
            except ScriptNotFoundError:
                pass

        raise ScriptNotFoundError(names)
    
    def parse(self, script):
        """Parse YAML script."""
        try:
            script_buffer, _ = self.select_script([script, script + '.yml'])
            parsed_script = yaml.load(script_buffer, Loader=self._loader)
        except Exception:
            try:
                parsed_script = yaml.load(script, Loader=self._loader)
            except Exception:
                raise ScriptNotFoundError(script)

        return parsed_script

    def dump(self, script_file, script):
        """Dump YAML script to file."""
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
