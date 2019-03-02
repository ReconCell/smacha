from smacha.util import bcolors


class ScriptNotFoundError(Exception):
    """The requested SMACHA YAML script was not found."""


class ParsingError(BaseException):
    """Parse exception class."""
    def __init__(self, error=None, line_number=None, line=None):
        if error is not None and line_number is not None and line is not None:
            message = bcolors.FAIL + 'Parsing error at line {0}:\n>>> {1}: {2}'.format(line_number, line, error) + bcolors.ENDC
        elif error is not None and line_number is not None:
            message = bcolors.FAIL + 'Parsing error at line {0}: {1}'.format(line_number, error) + bcolors.ENDC
        elif line_number is not None and line is not None:
            message = bcolors.FAIL +'Parsing error at line {0}:\n>>> {1}'.format(line_number, line) + bcolors.ENDC
        elif error is not None:
            message = bcolors.FAIL + 'Parsing error: {0}'.format(error) + bcolors.ENDC
        else:
            message = bcolors.FAIL + 'Parsing error!' + bcolors.ENDC

        super(ParsingError, self).__init__(message)
