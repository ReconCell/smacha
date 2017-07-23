# Yaml for script parsing
import os
import yaml
import errno
from yaml.composer import Composer
from yaml.constructor import Constructor
from smacha.exceptions import ScriptNotFoundError

__all__ = ['Parser']

class Parser():
    """SMACHA script parser."""

    def __init__(self, script_dirs=[]):
        self._loader = None
        self._script_dirs = script_dirs
        pass

    def load_script(self, script):
        """Search for the specified SMACHA YAML script file and load it."""
        script_filename = script.split('/')[-1]
        for script_dir in self._script_dirs:
            filename = os.path.join(script_dir, script_filename)
            try:
                f = open(filename, 'rb')
            except IOError as e:
                f = None
                pass

            if f is None:
                continue
            try:
                contents = f.read()
            finally:
                f.close()

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
    
    def parse(self, script):
        """Parse SMACHA YAML script."""
        if isinstance(script, file):
            script_buffer = script
        elif isinstance(script, str):
            script_buffer, _ = self.select_script([script, script + '.yml'])
        else:
            raise ScriptNotFoundError(script)

        self._loader = yaml.Loader(script_buffer)
        
        self._loader.compose_node = self._compose_node
        self._loader.construct_mapping = self._construct_mapping
        
        parsed_script = self._loader.get_single_data()
        
        return parsed_script
