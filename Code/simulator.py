from ros import Ros
from move_manipulator import MoveGroupPythonInteface
import time
import datetime
import csv
import os


def save_data(data, file_name):
    """Save to csv format"""
    with open(file_name + ".csv", 'ab') as name:
        writer = csv.writer(name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerows(data)

def create_arm(dof, joints_types, links, joint_axis, link_axis):
    """create the desired arm
        dof- number degrees of freedom
        joints_types - Revolute or prismatic
        links - length of links
        joint_axis - the axe which each joint use
        link_axis - the axe (from parent frame) which the cylinder height is going through
    """
    a=b


ros = Ros()
launchs_path = "man_gazebo"
main = ros.start_launch("main", launchs_path, ["gazebo_gui:=false", "rviz:=false"])  # main launch file
manipulator_move = MoveGroupPythonInteface()
time.sleep(0.2)
manipulator_move.add_obstacles(height=0.8, radius=0.1, pose=[0.7, 0.7])

poses = [[0.1, 0.2, 0.6], [0.3, 0.1, 0.1+0.3]]  # desired positions of the EE in world frame
oriens = [[0, -3.14, 0], [0, -3.14, 0]]  # desired orientaions of the EE in world frame

save_name = 'results_file' + datetime.datetime.now().strftime("%d_%m_%y")
arms=['option1','option2','option3']  # arms to check

first_run = True
to_replace = False

all_data = [["date ", "Time ", "Arm ", "Results "]]
arb = False

# cx = True
# x = 1
#while cx:
for arm in range(0,len(arms)):
    if arm % 100 == 0:  # save every 100 iterations
        save_data(all_data, save_name)
        all_data = []

    data = []
    # send the manipulator to the selected points
    for i in range(len(poses)):
        pose = poses[i]
        orien = oriens[i]
        data.append(str(manipulator_move.go_to_pose_goal(pose, orien)))
        to_replace = True
    # inserting the data into array
    all_data.append([datetime.datetime.now().strftime("%d/%m/%y, %H:%M"),arms[arm], ",".join(data)])

    time.sleep(0.1)

    # need to be changed
    if arb:
        arg = ["man:=manipulatorr"]
    else:
        arg = ["man:=manipulator"]
    arb = not arb
    #

    if to_replace:  # to replace a manipulator
        replace = ros.start_launch("replace_model", launchs_path,arg) # this launch file replace the manipulator
        if not first_run:  # doesnt run it first launch
            ros.stop_launch(arm_control)  # this launch file must be stopped, otherwise it wont work
        arm_control = ros.start_launch("arm_controller", launchs_path)
        first_run = False



save_data(all_data, save_name)
ros.stop_launch(replace)
ros.stop_launch(arm_control)
ros.stop_launch(main)
