from __future__ import print_function
import os
import subprocess
import sys


def run(cmd):
    os.environ['PYTHONUNBUFFERED'] = "1"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            universal_newlines=True, )
    stdout = []
    stderr = []
    while proc.poll() is None:
        line = proc.stdout.readline()
        if line != "":
            stdout.append(line)
            # print(line, end='')
        line = proc.stderr.readline()
        if line != "":
            stderr.append(line)
            print(line, end='')
    return proc.returncode, stdout, stderr

if __name__ == '__main__':
    dofe = '5'  # number degrees of freedom of the manipulator
    link_max = '0.41'  # max link length to check
    start_arm = '0'
    args = sys.argv
    if len(args) > 1:
        dofe = args[1]
        if len(args) > 2:
            link_max = args[2]
            if len(args) > 3:
                start_arm = args[3]
    code, out, err = run([sys.executable, 'simulator.py', dofe, link_max, start_arm ])
    # print("out: '{}'".format(out))
    # print("err: '{}'".format(err))
    print("exit: {}".format(code))