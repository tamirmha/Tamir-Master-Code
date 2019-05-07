from ros import Ros
from move_manipulator import MoveGroupPythonInteface
import time
import csv
import os


def save_data(data,file_name):
    """Save to csv format"""
    with open(file_name + ".csv", 'w') as file:
        writer = csv.writer(file,delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerows(data)


ros = Ros()
launchs_path = "man_gazebo"
main = ros.start_launch("main", launchs_path, ["gazebo_gui:=false","rviz:=false"])  # main launch file
manipulator_move = MoveGroupPythonInteface()
time.sleep(0.2)
manipulator_move.add_obstacles(height=0.8, radius=0.1, pose=[0.7, 0.7])

poses = [[0.1, 0.2, 0.6], [0.3, 0.1, 0.1+0.3]]  # desired positions of the EE in world frame
oriens = [[0, -3.14, 0], [0, -3.14, 0]]  # desired orientaions of the EE in world frame

first_run = True
to_replace = False
data = []

arb = False

cx = True
x = 1
while cx:
    time.sleep(1)
    for i in range(len(poses)):
        pose = poses[i]
        orien = oriens[i]
        data.append(str(manipulator_move.go_to_pose_goal(pose, orien)))
        to_replace = True
    time.sleep(1)

    if arb:
        arg = ["man:=manipulatorr"]
    else:
        arg = ["man:=manipulator"]
    arb = not arb

    if to_replace:  # to replace a manipulator
        replace = ros.start_launch("replace_model", launchs_path,arg)
        if not first_run:
            ros.stop_launch(arm_control)  # doesnt run it first launch
        arm_control = ros.start_launch("arm_controller", launchs_path)
        first_run = False

    if x <= 0:
        cx = False
    x = x-1
    print x
print data
save_data(data,'Test_file')
ros.stop_launch(replace)
ros.stop_launch(arm_control)
ros.stop_launch(main)
