from ros import Ros, MoveGroupPythonInterface, UrdfClass
import time
import datetime
import csv
import os
import numpy as np
import itertools
import rospy
import signal


# start - handle with csv
def save_data(data, file_name):
    """Save to csv format"""
    with open(file_name + ".csv", 'ab') as name:
        writer = csv.writer(name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerows(data)


def read_data(file_name):
    with open(file_name + ".csv", 'r') as _filehandler:
        csv_file_reader = csv.reader(_filehandler)
        data = []
        manip = []
        empty = True
        for row in csv_file_reader:
            while "" in row:
                row.remove("")
            if len(row) > 0:
                data.append(row)
                empty = False
            else:
                if not empty:
                    manip.append(read_data_action(data))
                    data = []
                empty = True
        manip.append(read_data_action(data))  # append the last session
        return manip


def read_data_action(data):
    manip = map(list, zip(*data[1:]))
    manip_array_of_dict = []
    for i in range(0, len(manip)-1, 2):
        manip_array_of_dict.append({"joint": manip[i], "axe": manip[i+1]})
    return manip_array_of_dict
# end - hande with csv


def create_arm(interface_joints, joint_parent_axis, links, folder):
    """create the desired arm
        interface_joints- roll,pitch,yaw or prismatic
                         roll - revolute around z (child frame)
                         pitch - revolute around y (child frame)
                         taw - revolute around x (child frame)
                         pris - prismatic along z (child frame)
        links - length of links
        joint_parent_axis - the axe, in the parent frame, which each joint use
    """
    joints = []
    joint_axis = []
    rpy = []
    file_name = ""
    for i in range(len(joint_parent_axis)):
        file_name += interface_joints[i] + "_" + joint_parent_axis[i] + "_" + links[i].replace(".", "_")
        if interface_joints[i].strip() == "roll":
            joints.append("revolute")
            joint_axis.append('z')
            if joint_parent_axis[i].strip() == "y":
                rpy.append(['${-pi/2} ', '0 ', '0 '])
            elif joint_parent_axis[i].strip() == "x":
                rpy.append(['0 ', '${pi/2} ', '0 '])
            elif joint_parent_axis[i].strip() == "z":
                rpy.append(['0 ', '0 ', '0 '])
        elif interface_joints[i].strip() == "pitch":
            joints.append("revolute")
            joint_axis.append('y')
            if joint_parent_axis[i].strip() == "y":
                rpy.append(['0 ', '0 ', '0 '])
            elif joint_parent_axis[i].strip() == "x":
                rpy.append(['0 ', '0 ', '${-pi/2} '])
            elif joint_parent_axis[i].strip() == "z":
                rpy.append(['${pi/2} ', '0 ', '0 '])
        elif interface_joints[i].strip() == "yaw":
            joints.append("revolute")
            joint_axis.append('x')
            if joint_parent_axis[i].strip() == "y":
                rpy.append(['0 ', '0 ', '0 '])
            elif joint_parent_axis[i].strip() == "x":
                rpy.append(['0 ', '0 ', '${-pi/2} '])
            elif joint_parent_axis[i].strip() == "z":
                rpy.append(['0 ', '${-pi/2} ', '0 '])
        elif interface_joints[i].strip() == "pris":
            joints.append("prismatic")
            joint_axis.append('z')
            if joint_parent_axis[i].strip() == "y":
                rpy.append(['${pi/2} ', '0 ', '0 '])
            elif joint_parent_axis[i].strip() == "x":
                rpy.append(['0 ', '${-pi/2} ', '0 '])
            elif joint_parent_axis[i].strip() == "z":
                rpy.append(['0 ', '0 ', '0 '])
    arm = UrdfClass(links, joints, joint_axis, rpy)
    return {"arm": arm, "name": file_name, "folder": folder}


def set_links_length(number=6, min_length=1):
    link_min = 0.1
    link_interval = 0.3
    link_max = 0.41
    links = []
    lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
    links_length = [[0.1] + list(tup) for tup in
                    list(itertools.product(lengths_2_check, repeat=(number - 1)))]
    for link in links_length:
        if sum(link) > min_length:
            links.append([str(x) for x in link])
    return links


def create_urdf_from_csv(csv_name="manips", number=6):
    # read from csv file with all the possible configuration for manipulators
    configs = read_data(csv_name)

    # Create the urdf files
    arms = []
    data = []
    folder = ""
    c = 0
    base_path = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/"
    links = set_links_length(number)
    #links = [['0.1', '0.4', '0.4', '0.1', '0.1', '0.1'], ['0.1', '0.4', '0.4', '0.1', '0.1', '0.1'],
    #        ['0.1', '0.4', '0.4', '0.1', '0.1', '0.1'], ['0.1', '0.4', '0.4', '0.1', '0.1', '0.1']]
    for config in configs:
        for arm in config:
            for i in range(len(arm["joint"])):
                folder = folder + arm["joint"][i] + "_" + arm["axe"][i] + "_"
            create_folder(base_path + str(len(arm["axe"])) + "dof/" + folder)
            for link in links:
                arms.append(create_arm(arm["joint"], arm["axe"], link, folder))
                path = base_path + str(len(arm["axe"])) + "dof/" + folder + "/"
                index = config.index(arm)+links.index(link)+c
                arms[index]["arm"].urdf_write(arms[index]["arm"].urdf_data(), path + arms[index]["name"])
                data.append([arms[index]["name"], folder, datetime.datetime.now().strftime("%d_%m_%y")])
            folder = ""
        c = c + len(config)+len(links)
    save_data(data, "created files")
    return arms


def arms_exist(folder="6dof/roll_z_pitch_y_pitch_y_pitch_y_pitch_z_roll_z_/"):
    path = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/" + folder
    # files = []
    arms =[]
    for file in os.listdir(path):
        # files.append(file.replace(".urdf.xacro", ""))
        fol = folder.split("/")

        arms.append({ "name": file.replace(".urdf.xacro", ""), "folder": fol[1]})
    return arms


def create_folder(name):
    if not os.path.exists(name):
        os.mkdir(name)
    return name


def run_simulation(dof=6):
    # initiliaze
    #arms = create_urdf_from_csv(str(dof) + "dof_configs", dof)  # all the configuration of the arms
    arms = arms_exist()

    ros = Ros()  # for work with Ros
    save_name = 'results_file' + datetime.datetime.now().strftime("%d_%m_%y")  # file to save the results

    # desired positions of the EE in world frame
    poses = [[0.5, 0.15, 0.86], [0.5, 0.0, 0.89]] #, [0.5, -0.15, 0.86], [0.5, -0.15, 0.45], [0.5, 0.15, 0.45]]
    # desired orientaions of the EE in world frame
    oriens = [[1.98, -0.83, 0], [-3.14, 0, 0], [-1.98, -0.83, 0], [-0.81, 0.52, 0], [0.9, 0.02, 0]]

    first_run = True
    to_replace = False
    all_data = [["date ", "Time ", "Arm ", "Results "]]
    moveit_arg = "dof:=" + str(dof) + "dof"
    for arm in range(0, len(arms)):
        print "arm " + str(arm+1) + " of " + str(len(arms)) + " arms"
        if first_run:
            # run all the needed launch files
            # launchs_path = "man_gazebo"
            # main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:=" + str(dof) + "dof",
            #                    "man:=" + arms[arm + 1]["folder"] + "/" + arms[arm + 1]["name"]]
            # main = ros.start_launch("main", launchs_path, main_launch_arg)  # main launch file
            #
            man_launch_arg = ["gui:=false", "dof:=" + str(dof) + "dof",
                              "man:=" + arms[arm + 1]["folder"] + "/" + arms[arm + 1]["name"]]
            man = ros.start_launch("manipulator", "man_gazebo", man_launch_arg)
            # time.sleep(5)  # need time to upload
            # man_command="x-terminal-emulator -e roslaunch man_gazebo manipulator.launch" \
            #     " gui:=false " + "dof:=" + str(dof) + "dof man:=" + arms[arm]["folder"] + "/" + arms[arm]["name"]
            # man = ros.ter_command(man_command)
            # moveit_command = "x-terminal-emulator -e roslaunch man_moveit man_moveit_planning_execution.launch " + moveit_arg + "  __name:=test"
            # moveit = ros.ter_command(moveit_command)
            rospy.sleep(3)
            moveit = ros.start_launch("man_moveit_planning_execution", "man_moveit", moveit_arg)
            rospy.sleep(3)
            manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
            rospy.sleep(3)  # need time to upload
            # add floor and plant to the planning model
            manipulator_move.add_obstacles(height=0.75, radius=0.1, pose=[0.5, 0])
            rospy.sleep(3)

        if arm % 100 == 0:  # save every 100 iterations
            save_data(all_data, save_name)
            all_data = []

        data = []
        # send the manipulator to the selected points
        rospy.sleep(3)
        for i in range(len(poses)):
            pose = poses[i]
            orien = oriens[i]
            time.sleep(0.65)
            data.append(str(manipulator_move.go_to_pose_goal(pose, orien)))
            to_replace = True
        # inserting the data into array
        all_data.append([datetime.datetime.now().strftime("%d/%m/%y, %H:%M"), arms[arm]["name"], ",".join(data)])

        if arm == len(arms)-1:
            break

        launch_arg = ["man:=" + arms[arm + 1]["folder"] + "/" + arms[arm + 1]["name"], "dof:="+str(dof) + "dof"]
        file = "man:=" + arms[arm + 1]["folder"] + "/" + arms[arm + 1]["name"] + " dof:=" + str(dof) + "dof"


        if to_replace:  # to replace a manipulator
            # replace_command = "x-terminal-emulator -e roslaunch man_gazebo replace_model.launch " + file
            # ros.ter_command(replace_command)
            ros.start_launch("replace_model", "man_gazebo", launch_arg)
            if not first_run:  # doesnt run it first launch
                # ros.stop_launch(arm_control)  # this launch file must be stopped, otherwise it wont work
                # arm_control.kill()
                # os.killpg(os.getpgid(arm_control.pid), signal.SIGTERM)
                a =6
            #arm_control = ros.start_launch("arm_controller", launchs_path, ["dof:="+str(dof) + "dof"])
            # arm_control = ros.ter_command("x-terminal-emulator -e roslaunch man_gazebo arm_controller.launch ")
            rospy.sleep(0.06)
            first_run = False

    save_data(all_data, save_name)
    man.kill()
    # ros.stop_launch(replace)
    # ros.stop_launch(arm_control)
    # ros.stop_launch(man)
    # ros.stop_launch(main)


tic = datetime.datetime.now()
run_simulation()
#arms=arms_exist()


toc = datetime.datetime.now()
delta = (toc - tic).seconds
print('Time of Run (seconds): ' + str(delta))
