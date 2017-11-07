import os
import yaml
from yaml.composer import Composer
from yaml.constructor import Constructor
from smacha.exceptions import ScriptNotFoundError

__all__ = ['Parser']

class Parser():
    """YAML script parser."""

    def __init__(self, script_dirs=[]):
        self._loader = None
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
    
    
    # Credit to puzzlet@stackoverflow for the following two functions that
    # allow for line numbers to be added to the parsed yaml.
    #
    # http://stackoverflow.com/questions/13319067/parsing-yaml-return-with-line-number
    #
    def _compose_node(self, parent, index):
        # the line number where the previous token has ended (plus empty lines)
        line = self._loader.line
        node = Composer.compose_node(self._loader, parent, index)
        node.__line__ = line + 1
        return node
    
    def _construct_mapping(self, node, deep=False):
        mapping = Constructor.construct_mapping(self._loader, node, deep=deep)
        mapping['__line__'] = node.__line__
        return mapping
    
    def parse(self, script, include_line_numbers=True):
        """Parse YAML script."""
        try:
            script_buffer, _ = self.select_script([script, script + '.yml'])
            self._loader = yaml.Loader(script_buffer)
        except Exception:
            try:
                self._loader = yaml.Loader(script)
            except Exception:
                raise ScriptNotFoundError(script)

        if include_line_numbers:
            self._loader.compose_node = self._compose_node
            self._loader.construct_mapping = self._construct_mapping
        
        parsed_script = self._loader.get_single_data()
        
        return parsed_script

    def strip_line_numbers(self, script):
        """Strip any line number keys from script."""
        try:
            script = dict(script)
            for script_key, script_val in script.items():
                if isinstance(script_val, dict):
                    script[script_key] = self.strip_line_numbers(script_val)
            del script['__line__']
            return script
        except:
            return script
