# Yaml for script parsing
import yaml
from yaml.composer import Composer
from yaml.constructor import Constructor

__all__ = ['Parser']

class Parser():
    """SMACHA script parser."""
    def __init__(self):
        self._loader = None
        pass
    
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
    
    def parse(self, script_buffer):
        """Parse SMACHA yaml script."""
        
        self._loader = yaml.Loader(script_buffer)
        
        self._loader.compose_node = self._compose_node
        self._loader.construct_mapping = self._construct_mapping
        
        parsed_script = self._loader.get_single_data()
        
        return parsed_script
