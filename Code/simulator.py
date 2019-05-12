from ros import Ros, MoveGroupPythonInterface, UrdfClass
import time
import datetime
import csv
import os
import numpy as np
import itertools

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
            if len(row)>0:
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
    manip_array_of_dict=[]
    for i in range(0, len(manip)-1, 2):
        manip_array_of_dict.append({"joint":manip[i], "axe":manip[i+1]})
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
    rpy =[]
    file_name = ""
    for i in range(len(joint_parent_axis)):
        file_name += interface_joints[i] + "_" + joint_parent_axis[i] + "_" + links[i].replace(".","_")
        if interface_joints[i] == "roll":
            joints.append("revolute")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                rpy.append(['${-pi/2} ', '0 ', '0 '])
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '${pi/2} ', '0 '])
            elif joint_parent_axis[i] == "z":
                    rpy.append(['0 ', '0 ', '0 '])
        elif interface_joints[i] == "pitch":
            joints.append("revolute")
            joint_axis.append('y')
            if joint_parent_axis[i] == "y":
                rpy.append(['0 ', '0 ', '0 '])
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '0 ', '${-pi/2} '])
            elif joint_parent_axis[i] == "z":
                rpy.append(['${pi/2} ', '0 ', '0 '])
        elif interface_joints[i] == "yaw":
            joints.append("revolute")
            joint_axis.append('x')
            if joint_parent_axis[i] == "y":
                rpy.append(['0 ', '0 ', '0 '])
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '0 ', '${-pi/2} '])
            elif joint_parent_axis[i] == "z":
                    rpy.append(['0 ', '${-pi/2} ', '0 '])
        elif interface_joints[i] == "pris":
            joints.append("prismatic")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                rpy.append(['${pi/2} ', '0 ', '0 '])
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '${-pi/2} ', '0 '])
            elif joint_parent_axis[i] == "z":
                    rpy.append(['0 ', '0 ', '0 '])
    arm = UrdfClass(links, joints, joint_axis, rpy)
    return {"arm":arm, "name":file_name, "folder":folder}


def set_links_length(number=6, min_length=1.3):
    link_min = 0.1
    link_interval = 0.3
    link_max = 1.1
    links = []
    lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
    links_length = [[0.1] + list(tup) for tup in
                    list(itertools.product(lengths_2_check, repeat=(number - 1)))]
    for link in links_length:
        if sum(link) > min_length:
            links.append([str(x) for x in link])

    return links


def create_urdf_from_csv(csv_name="manips"):
    # read from csv file with all the possible configuration for manipulators
    configs = read_data(csv_name)

    # Create the urdf files
    arms = []
    folder = ""
    c = 0
    base_path = os.environ['HOME'] + "/catkin_ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/"
    links = set_links_length()
    for config in configs:
        # Todo just for start- need to be replace with real lengths
        # if len(config[0]["joint"]) == 6:
        #     links = ["0.1", "0.4", "0.7", "0.1", "0.1", "0.1"]
        # if len(config[0]["joint"]) == 5:
        #     links = ["0.1", "0.4", "0.7", "0.1", "0.1"]
        # if len(config[0]["joint"]) == 4:
        #     links = ["0.1", "0.4", "0.7", "0.1"]
        # if len(config[0]["joint"]) == 3:
        #     links = ["0.1", "0.4", "0.7"]
        for arm in config:
            for i in range(len(arm["joint"])):
                folder = folder + arm["joint"][i] + "_" +  arm["axe"][i] + "_"
            create_folder(base_path  + str(len(arm["axe"])) + "dof/"+ folder)
            for link in links:
                arms.append(create_arm(arm["joint"], arm["axe"], link, folder))
                path = base_path + str(len(arm["axe"])) + "dof/" + folder + "/"
                index = config.index(arm)+links.index(link)+c
                arms[index]["arm"].urdf_write(arms[index]["arm"].urdf_data(), path + arms[index]["name"])
            folder = ""
        c = c + len(config)+len(links)
    return arms


def create_folder(name):
    if not os.path.exists(name):
        os.mkdir(name)
    return name

tic = datetime.datetime.now()
# initiliaze
dof = 6  # number of degrees of freedom
arms = create_urdf_from_csv()  # all the configuration of the arms
ros = Ros()  # for work with Ros
save_name = 'results_file' + datetime.datetime.now().strftime("%d_%m_%y") # file to save the results

# desired points to reach  Todo decide points
poses = [[0.1, 0.2, 0.6], [0.3, 0.1, 0.1+0.3]]  # desired positions of the EE in world frame
oriens = [[0, -3.14, 0], [0, -3.14, 0]]  # desired orientaions of the EE in world frame

first_run = True
to_replace = False
all_data = [["date ", "Time ", "Arm ", "Results "]]
arb = False
for arm in range(0, 250):# len(arms)):
    if first_run:
        # run all the needed launch files
        launchs_path = "man_gazebo"
        main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:="+str(dof)+"dof", "man:=" + arms[arm + 1]["folder"] + "/" + arms[arm + 1]["name"]]
        main = ros.start_launch("main", launchs_path,main_launch_arg

                                )  # main launch file
        manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        time.sleep(1)  # need time to upload
        manipulator_move.add_obstacles(height=0.8, radius=0.1,
                                       pose=[0.7, 0.7])  # add floor and plant to the planning model

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
    all_data.append([datetime.datetime.now().strftime("%d/%m/%y, %H:%M"), arms[arm]["name"], ",".join(data)])

    launch_arg = ["man:="+arms[arm+1]["folder"]+"/"+arms[arm+1]["name"]]
    if to_replace:  # to replace a manipulator
        replace = ros.start_launch("replace_model", launchs_path, launch_arg)  # this launch file replace the manipulator
        if not first_run:  # doesnt run it first launch
            ros.stop_launch(arm_control)  # this launch file must be stopped, otherwise it wont work
        arm_control = ros.start_launch("arm_controller", launchs_path)
        first_run = False

save_data(all_data, save_name)
ros.stop_launch(replace)
ros.stop_launch(arm_control)
ros.stop_launch(main)
toc = datetime.datetime.now()
delta = (toc - tic).seconds
print('Time of Run (seconds): ' + str(delta))