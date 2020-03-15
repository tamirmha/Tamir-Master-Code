import sys
from pathlib import Path


class PrintLogger(object):
    """ PrintLogger prints to a file and to std_out simultaneously. """
    def __init__(self, log_path):
        self.terminal = sys.stdout
        self.log_path = log_path / 'print_log.txt'
        try:
            self.log.close()
        except AttributeError:
            pass
        self.log = open(str(self.log_path), 'a')

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)

    def end_log(self):
        self.log.close()
        sys.stdout = self.terminal

    def flush(self):
        pass

# import sys
# while True:
#     data = sys.stdin.read()
#     print 'Data from stdin -', data