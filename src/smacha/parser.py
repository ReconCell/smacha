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
    
    def parse(self, script, include_line_numbers=True):
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
