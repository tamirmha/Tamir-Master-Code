from ros import Ros, MoveGroupPythonInterface, UrdfClass, HandleCSV
from datetime import datetime
from os import environ, listdir, path, mkdir, rename
import numpy as np
from itertools import product
from time import sleep
from rospy import init_node
import getpass, sys
import json


class Simulator(object):

    def __init__(self, dof, folder, create=False, arms=None, wait1=1.7, wait2=1.2, link_max=0.41):
        # if arms is None:
        #   arms = []
        self.dof = dof
        self.folder = folder
        self.ros = Ros()  # for work with Ros
        self.arm_control = 0
        self.arms = []
        self.wait1 = wait1
        self.wait2 = wait2
        if create:  # all the configuration of the arms
            self.create_urdf_from_csv(str(self.dof) + "dof_configs",  link_max)
            # sleep(10)
        else:
            if not arms:
                self.arms_exist()
            else:
                self.arms = arms
        # desired positions and orientaions of the EE in world frame
        z = 3  # height from ground
        # self.poses = [[0.5, 0.15, z + 0.86], [0.5, 0.0, z + 0.89], [0.5, -0.15, z + 0.86],
        #               [0.5, -0.15, z + 0.45], [0.5, 0.15, z + 0.45]]
        # self.oriens = [[1.98, -0.83, 0], [-3.14, 0, 0], [-1.98, -0.83, 0], [-0.81, 0.52, 0], [0.9, 0.02, 0]]
        self.poses = [[0.5, 0, z + 0.9], [0.2, 0, z + 0.9], [0.2, 0.0, z + 0.65], [0.2, 0, z + 0.4]]
        self.oriens = [[-3.14, 0, 0], [0, 3.1459*0.75, 0], [0, 3.1459*0.5, 0], [0, 3.1459*0.25, 0]]
        self.save_name = 'results_file' + datetime.now().strftime("%d_%m_") + str(dof) + "dof_" \
                        + str(len(self.poses)) + "d_"
        # for some reason the 1st manipulator must succeed reach to point otherwise the other manipulators will failed
        main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:=" + str(self.dof) + "dof"]
        self.main = self.ros.start_launch("main", "man_gazebo", main_launch_arg)  # main launch file
        # set the obstacles and initiliaze the manipulator
        self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        # add floor and plant to the planning model
        sleep(0.14)
        self.manipulator_move.add_obstacles(height=z + 0.75, radius=0.1, pose=[0.5, 0])
        pos = self.manipulator_move.get_current_position()
        orien = self.manipulator_move.get_current_orientain()
        self.manipulator_move.go_to_pose_goal([pos.x, pos.y, pos.z], [orien[0], orien[1], orien[2]])
        # self.manipulator_move.go_to_pose_goal(self.poses[0], self.oriens[0])
        self.replace_model(0)  # set the first arm

    @staticmethod
    def save_json(name="data_file", data=None):
        with open(name + ".json", "a") as write_file:
            json.dump(data, write_file, indent=2)

    @staticmethod
    def load_json(name="data_file"):
        with open(name + ".json", "r") as read_file:
            return json.load(read_file)

    @staticmethod
    def create_arm(interface_joints, joint_parent_axis, links, folder):
        """create the desired arm
            interface_joints- roll,pitch,yaw or prismatic
                             roll - revolute around own Z axe
                             pitch - revolute that not roll
                             pris - prismatic along
            links - length of links
            joint_parent_axis - the axe, in the parent frame, which each joint use
        """
        joints = []
        joint_axis = []
        rpy = []
        file_name = ""
        rolly_number = -1
        rolly_originy = []
        pitchz_number = 1
        prisy_number = 1
        for i in range(len(joint_parent_axis)):
            rolly_originy.append(1)
            file_name += interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + links[i].replace(".", "_")
            if interface_joints[i].replace(" ", "") == "roll":
                joints.append("revolute")
                joint_axis.append('z')
                if joint_parent_axis[i].replace(" ", "") == "y":
                    # rpy.append(['${-pi/2} ', '0 ', '0 '])
                    # todo -- to check!!
                    rolly_rot = '${' + str(rolly_number) + '/2*pi} '
                    rpy.append([rolly_rot, '0 ', '0 '])
                    rolly_number = rolly_number * -1
                    rolly_originy[i] = rolly_number
                elif joint_parent_axis[i].replace(" ", "") == "x":
                    rpy.append(['0 ', '${pi/2} ', '0 '])
                elif joint_parent_axis[i].replace(" ", "") == "z":
                    rpy.append(['0 ', '0 ', '0 '])
            elif interface_joints[i].replace(" ", "") == "pitch":
                joints.append("revolute")
                joint_axis.append('y')
                if joint_parent_axis[i].strip() == "y":
                    rpy.append(['0 ', '0 ', '0 '])
                elif joint_parent_axis[i].strip() == "x":
                    rpy.append(['0 ', '0 ', '${-pi/2} '])
                elif joint_parent_axis[i].strip() == "z":
                    # rpy.append(['${pi/2} ', '0 ', '0 '])
                    # todo -- to check!!
                    pitchz = '${' + str(pitchz_number) + '/2*pi} '
                    rpy.append([pitchz, '0 ', '0 '])
                    pitchz_number = pitchz_number * -1
            elif interface_joints[i].replace(" ", "") == "pris":
                joints.append("prismatic")
                joint_axis.append('z')
                if joint_parent_axis[i].strip() == "y":
                    # rpy.append(['${pi/2} ', '0 ', '0 '])
                    prisy = '${' + str(prisy_number) + '/2*pi} '
                    rpy.append([prisy, '0 ', '0 '])
                    prisy_number = prisy_number * -1
                elif joint_parent_axis[i].strip() == "x":
                    rpy.append(['0 ', '${-pi/2} ', '0 '])
                elif joint_parent_axis[i].strip() == "z":
                    rpy.append(['0 ', '0 ', '0 '])
        arm = UrdfClass(links, joints, joint_axis, rpy, rolly_originy)
        return {"arm": arm, "name": file_name, "folder": folder}

    def set_links_length(self, min_length=1, link_min=0.1, link_interval=0.3, link_max=0.71):
        """
        set all the possible links lengths in the defind interval
        :param min_length: the minimum length of all the links
        :param link_min: minimum length of a link
        :param link_interval: interval between joints lengths
        :param link_max: maximun length of a link
        :return: links: all the lengths combinations in the minimum length - list of strings
        """

        links = []
        lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
        links_length = [[0.1] + list(tup) for tup in
                        list(product(lengths_2_check, repeat=(self.dof - 1)))]
        for link in links_length:
            if sum(link) > min_length:
                links.append([str(x) for x in link])
        return links

    def create_urdf_from_csv(self, csv_name="manips", link_max=0.41):
        # read from csv file with all the possible configuration for manipulators
        configs = HandleCSV().read_data(csv_name)
        # Create the urdf files
        data = []
        base_path = environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/"
        self.create_folder(base_path + str(self.dof) + "dof/" + self.folder)
        links = self.set_links_length(link_max=link_max)
        index = 0
        for config in configs:
            for arm in config:
                for link in links:
                    self.arms.append(self.create_arm(arm["joint"], arm["axe"], link, self.folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + self.folder + "/"
                    self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                                                       path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"].replace(" ", ""), self.folder, datetime.now().strftime("%d_%m_%y")])
                    index = index+1
        # self.save_json("created arms", data)

    def arms_exist(self):
        path = environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/" + str(self.dof) \
               + "dof/" + self.folder
        for fil in listdir(path):
            fol = self.folder.split("/")
            self.arms.append({"name": fil.replace(".urdf.xacro", ""), "folder": fol[0]})

    @staticmethod
    def create_folder(name):
        if not path.exists(name):
            mkdir(name)
        return name

    def assign_data(self, data, arm):
        """
        Calculate the manipulaot indices(Manipulability, Local Conditioning Index, Joint Mid-Range Proximity)
        if the manipulator succed and the time that take
        :param data: array of the result of the configuration about each detection point
        :param arm: which configuration
        :return: array of the results
        """
        data_res = []
        jacobian = []
        curr_pos =[]
        mu = []   # Manipulability index
        lci = []  # Local Conditioning Index
        z = []    # Joint Mid-Range Proximity
        ri = []   # Relative Manipulability Index
        for j in data:
            data_res.append(j[0])
            if j[0]:
                mu.append(j[2][0])
                lci.append(j[2][1])
                z.append(j[2][2].min())
                ri.append(j[2][3])
                jacobian.append(j[2][4].tolist())
                curr_pos.append(j[2][5].tolist())
            else:
                mu.append(-1)
                lci.append(-1)
                z.append(-1)
                ri.append(-1)
                jacobian.append(-1)
                curr_pos.append(-1)
        self.json_data.append({self.arms[arm]["name"]: [jacobian, curr_pos]})
        suc_res = "False"
        mu_min = -1
        lci_min = -1
        ri_min = -1
        z_max = -1
        data_time = [-1, -1, -1, -1]
        avg_time = -1
        if data_res.count(True) >= 3 and data_res[3] and (data_res[0] or data_res[1]):
            # if the arm arrived to 3 or more point and get to the lower point or one of the two
            # top points --> it success and calc indices
            suc_res = "True"
            data_time = [d[1] for d in data]
            avg_time = np.mean(data_time).round(2)
            mu = np.asarray(mu)
            lci = np.asarray(lci)
            z = np.asarray(z)
            ri = np.asarray(ri)
            # print(str(mu), str(lci), str(z))
            # choose only the min values because those are the "worst grade"
            try:
                mu_min = mu[mu >= 0.0].min()
            except:
                self.save_json("mu_err", mu.tolist())
                mu_min = -16
            try:
                lci_min = lci[lci >= 0.0].min()
            except:
                self.save_json("lci_err", lci.tolist())
                lci_min = -16
            try:
                ri_min = ri[ri >= 0.0].min()
            except:
                self.save_json("ri_err", ri.tolist())
                ri_min = -16
                # choose only the max value because this is the "worst grade"
            try:
                z_max = z[z > 0.0].max()
            except:
                self.save_json("z_err", z.tolist())
                z_max = -16
        return [datetime.now().strftime("%d/%m/%y, %H:%M"), self.arms[arm]["name"], data_res,
                str(data_time), suc_res,  str(avg_time), str(mu_min), str(z_max), str(lci_min), str(ri_min)]

    def replace_model(self, arm):
        """
        replace configuration in the simulation
        :param arm: the new configuration tho simulate
        :return:
        """
        fil = "man:=" + self.arms[arm + 1]["folder"] + "/" + self.arms[arm + 1]["name"] + \
              " dof:=" + str(self.dof) + "dof"
        if self.arm_control != 0:
            self.ros.stop_launch(self.arm_control)  # this launch file must be stopped, otherwise it wont work
        replace_command = "roslaunch man_gazebo replace_model.launch " + fil
        self.ros.ter_command(replace_command)
        sleep(self.wait1)
        self.arm_control = self.ros.start_launch("arm_controller", "man_gazebo", ["dof:=" + str(self.dof) + "dof"])
        sleep(self.wait2)

    def run_simulation(self,  k=0, len_arm=1638):
        save_name = self.save_name  # 'results_file' + datetime.now().strftime("%d_%m_%y")  # file to save the results
        all_data = []
        self.json_data = []
        for arm in range(0, len(self.arms)):
            print self.arms[arm]["name"] + " " + str(arm + 1 + k) + " of " + str(len_arm) + " arms"
            data = []
            try:
                joints = self.arms[arm]["arm"].joint_data
                links = self.arms[arm]["arm"].links
            except:
                joints = ["revolute"] * self.dof
                links = [0.4] * self.dof
            for p in range(len(self.poses)):  # send the manipulator to the selected points
                # inserting the data into array
                data.append(self.manipulator_move.go_to_pose_goal(self.poses[p], self.oriens[p], joints, links))
            # calculate relavent data from data array
            all_data.append(self.assign_data(data, arm))

            if arm == len(self.arms) - 1:
                break
            self.replace_model(arm)
        # save the remaining data and close all the launch files
        self.save_json(save_name, self.json_data)
        HandleCSV().save_data(all_data, save_name)
        self.ros.stop_launch(self.arm_control)
        self.ros.stop_launch(self.main)


if __name__ == '__main__':
    # get pc name for specific configuration
    username = getpass.getuser()
    if username == "tamir":  # tamir laptop
        nums = 35  # how many arms to send to simulator each time
        wait1_replace = 2
        wait2_replace = 2
    elif username == "arl_main":  # lab
        nums = 35  # how many arms to send to simulator each time
        wait1_replace = 2
        wait2_replace = 2
    elif username == "tamirm":  # VM
        nums = 25  # how many arms to send to simulator each time
        wait1_replace = 2
        wait2_replace = 2
    else:
        nums = 30  # how many arms to send to simulator each time
        wait1_replace = 1.7
        wait2_replace = 1.2
    # set parametrs from terminal
    args = sys.argv
    dofe = 5  # number degrees of freedom of the manipulator
    link_max = 0.41  # max link length to check
    start_arm = 0  # from which set of arms to start
    if len(args) > 1:
        dofe = int(args[1])
        if len(args) > 2:
            link_max = float(args[2]) + 0.1
            if len(args) > 3:
                start_arm = int(args[3])/nums
    tic_main = datetime.now()
    ros = Ros()
    ros.ter_command("rosclean purge -y")
    # check if there is roscore running if there is stop it
    roscore = ros.checkroscorerun()
    if roscore:
        ros.ter_command("kill -9 " + str(roscore))
    # start roscore
    ros.ros_core_start()
    init_node('arl_python', anonymous=True)
    # folder to save the file to
    foldere = "combined"
    sim = Simulator(dofe, foldere, True, wait1=wait1_replace,  wait2=wait2_replace, link_max=link_max)
    if start_arm > 0:
        ros.stop_launch(sim.arm_control)
        ros.stop_launch(sim.main)
    arms = sorted(sim.arms, reverse=True)
    # sim.arms = arms[:nums]
    # sim.run_simulation(nums*0, len(arms))
    for t in range(start_arm, int(np.ceil(1.0*len(arms) / nums))):
        if t == len(arms) / nums:
            sim = Simulator(dofe, foldere, False, arms[t * nums:], wait1=wait1_replace, wait2=wait2_replace)
            sim.run_simulation(nums*t, len(arms))
        elif t != 0:
            sim = Simulator(dofe, foldere, False, arms[t * nums:(t + 1) * nums], wait1=wait1_replace, wait2=wait2_replace)
            sim.run_simulation(nums*t, len(arms))
        else:  # first run
            sim.arms = arms[:nums]
            sim.run_simulation(nums*t, len(arms))
    ros.ros_core_stop()
    toc_main = datetime.now()
    print('Time of Run (seconds): ' + str((toc_main - tic_main).seconds))
    rename(sim.save_name + ".csv", sim.save_name + str((toc_main - tic_main).seconds) + ".csv")


# import multiprocessing as mp
# def map_simulator(t, nums=35):
#     sim = Simulator(dofe, foldere, False, arms[t * nums:(t + 1) * nums], wait1=wait1_replace, wait2=wait2_replace)
#     sim.run_simulation(nums * t, len(arms))
#
# n_cpu = 8
# if __name__ == '__main__':
#     mp.Pool(n_cpu).imap(map_simulator, range(start_arm, int(np.ceil(1.0*len(arms) / nums))))

# todo - change rpy!!!!!!!
# todO - get errors from terminal
# todo the file name wont change when date change
# todo add to rename the total of success
# Done - set for first joint the current location as target.
# done  calculate indicies
# todO change links weight according length
# done delete base link visuality and change link0 to box
# todo change link0 to the platform -
# done -add roll for each manipulator in last joint
# done add floor at 3 meter
# done change detection points
# todo change the base_link to 0 meters - check difference
# todo check planner parameters
# TODo how accuracy in go to pose effect
# Todo fix obstacle parameters
# done save to JSON file  - disabled
# done change defination of success
# done delete created files
# done change length from terminal
# done get pc name for specific configuration
# done set parametrs from terminal
