# import sys
# from pathlib import Path
#
#
# class PrintLogger(object):
#     """ PrintLogger prints to a file and to std_out simultaneously. """
#     def __init__(self, log_path):
#         self.terminal = sys.stdout
#         self.log_path = log_path / 'print_log.txt'
#         try:
#             self.log.close()
#         except AttributeError:
#             pass
#         self.log = open(str(self.log_path), 'a')
#
#     def write(self, message):
#         self.terminal.write(message)
#         self.log.write(message)
#
#     def end_log(self):
#         self.log.close()
#         sys.stdout = self.terminal
#
#     def flush(self):
#         pass
#
# # import sys
# # while True:
# #     data = sys.stdin.read()
# #     print 'Data from stdin -', data

from multiprocessing import Process, Queue
from time import sleep
import subprocess
import shlex
from simulator import simulate
from ros import Ros
import os


def my_function():
    x = 0
    for i in range(100):
        x += 1
        sleep(0.061)
        print x
    # q.put(x)


def try_os(cmd):
    # os.system(cmd)
    cmd = shlex.split(cmd)
    subprocess.Popen(cmd, stdout=subprocess.PIPE, preexec_fn=os.setsid)
    # a = Ros.checkroscorerun()
    # print a


if __name__ == '__main__':

    # cmd = 'gnome-terminal -e roscore'
    # cmd = 'gnome-terminal -- python simulator.py 6 1'
    # p = Process(target=Ros.ter_command, args=(cmd,))
    # p = Process(target=try_os, args=(cmd,))
    p = Process(target=simulate, args=(1,))
    p.start()
    p.join()  # this blocks until the process terminates
    # result = queue.get()
    print "finish"


