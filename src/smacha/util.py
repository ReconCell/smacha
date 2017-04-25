import smacha

class bcolors:
    """Colour terminal print strings."""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class ParseException(BaseException):
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
        
        super(ParseException, self).__init__(message)
