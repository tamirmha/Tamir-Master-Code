from ros import Ros, MoveGroupPythonInterface, UrdfClass, HandleCSV
from datetime import datetime
from os import environ, listdir, path, mkdir
import numpy as np
from itertools import product
from time import sleep
from rospy import init_node


class Simulator(object):

    def __init__(self, dof, folder, create=False, arms=None):
        # if arms is None:
        #   arms = []
        self.dof = dof
        self.folder = folder
        self.ros = Ros()  # for work with Ros
        self.arm_control = 0
        self.arms = []
        if create:  # all the configuration of the arms
            self.create_urdf_from_csv(str(self.dof) + "dof_configs")
        else:
            if not arms:
                self.arms_exist()
            else:
                self.arms = arms
        # desired positions and orientaions of the EE in world frame
        z = 3  # height from ground
        self.poses = [[0.5, 0.15, z + 0.86], [0.5, 0.0, z + 0.89], [0.5, -0.15, z + 0.86],
                      [0.5, -0.15, z + 0.45], [0.5, 0.15, z + 0.45]]
        self.oriens = [[1.98, -0.83, 0], [-3.14, 0, 0], [-1.98, -0.83, 0], [-0.81, 0.52, 0], [0.9, 0.02, 0]]
        # for some reason the 1st manipulator must succeed reach to point otherwise the other manipulators will failed
        main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:=" + str(self.dof) + "dof"]
        self.main = self.ros.start_launch("main", "man_gazebo", main_launch_arg)  # main launch file
        # set the obstacles and initiliaze the manipulator
        self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        # add floor and plant to the planning model
        sleep(0.13)
        self.manipulator_move.add_obstacles(height=z + 0.75, radius=0.1, pose=[0.5, 0])
        pos = self.manipulator_move.get_current_position()
        orien = self.manipulator_move.get_current_orientain()
        self.manipulator_move.go_to_pose_goal([pos.x, pos.y, pos.z], [orien[0], orien[1], orien[2]])

        # self.manipulator_move.go_to_pose_goal(self.poses[0], self.oriens[0])
        self.replace_model(0)  # set the first arm

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
            # elif interface_joints[i].strip() == "yaw":
            #     joints.append("revolute")
            #     joint_axis.append('x')
            #     if joint_parent_axis[i].strip() == "y":
            #         rpy.append(['0 ', '0 ', '0 '])
            #     elif joint_parent_axis[i].strip() == "x":
            #         rpy.append(['0 ', '0 ', '${-pi/2} '])
            #     elif joint_parent_axis[i].strip() == "z":
            #         rpy.append(['0 ', '${-pi/2} ', '0 '])
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

    def set_links_length(self, min_length=1, link_min=0.1, link_interval=0.3, link_max=0.41):
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

    def create_urdf_from_csv(self, csv_name="manips"):
        # read from csv file with all the possible configuration for manipulators
        configs = HandleCSV().read_data(csv_name)
        # Create the urdf files
        data = []
        base_path = environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/"
        self.create_folder(base_path + str(self.dof) + "dof/" + self.folder)
        links = self.set_links_length()
        index = 0
        for config in configs:
            for arm in config:
                for link in links:
                    self.arms.append(self.create_arm(arm["joint"], arm["axe"], link, self.folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + self.folder + "/"
                    self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                                                       path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"], self.folder, datetime.now().strftime("%d_%m_%y")])
                    index = index+1
        HandleCSV().save_data(data, "created files")

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
        sleep(1.6)
        self.arm_control = self.ros.start_launch("arm_controller", "man_gazebo", ["dof:=" + str(self.dof) + "dof"])
        sleep(1.2)

    def assign_data(self, data, arm):
        """
        Calculate the manipulaot indices(Manipulability, Local Conditioning Index, Joint Mid-Range Proximity)
        if the manipulator succed and the time that take
        :param data: array of the result of the configuration about each detection point
        :param arm: which configuration
        :return: array of the results
        """
        data_res = []
        mu = []  # Manipulability index
        lci = []  # Local Conditioning Index
        z = []  # Joint Mid-Range Proximity
        for j in data:
            data_res.append(j[0])
            if j[0]:
                mu.append(j[2][0])
                lci.append(j[2][1])
                z.append(j[2][2].min())
            else:
                mu.append(-1)
                lci.append(-1)
                z.append(-1)
        suc_res = "False"
        mu_min = -1
        lci_min = -1
        z_max = -1
        data_time = [-1, -1, -1, -1, -1]
        avg_time = -1
        if data_res.count(True) >= 3:
            # if the arm arrived to 3 or more point it success and calc indices
            suc_res = "True"
            # data_time = []
            data_time = [d[1] for d in data]
            avg_time = np.mean(data_time).round(2)
            mu = np.asarray(mu)
            lci = np.asarray(lci)
            z = np.asarray(z)
            # choose only the min values because those are the "worst grade"
            mu_min = mu[mu > 0].min()
            lci_min = lci[lci > 0].min()
            # choose only the max value because this is the "worst grade"
            z_max = z[z > 0].max()
        # return  data_res, data_time, suc_res, avg_time, mu_min, z_min, lci_min
        return [datetime.now().strftime("%d/%m/%y, %H:%M"), self.arms[arm]["name"],
                data_res, str(data_time), suc_res,  str(avg_time), str(mu_min), str(z_max), str(lci_min)]

    def run_simulation(self,  k=0, len_arm=1638):
        save_name = 'results_file' + datetime.now().strftime("%d_%m_%y")  # file to save the results
        all_data = []
        for arm in range(0, len(self.arms)):
            print "arm " + str(arm + 1 + k) + " of " + str(len_arm) + " arms"
            data = []
            joints = self.arms[arm]["arm"].joint_data
            links = self.arms[arm]["arm"].links
            for p in range(len(self.poses)):  # send the manipulator to the selected points
                # inserting the data into array
                data.append(self.manipulator_move.go_to_pose_goal(self.poses[p], self.oriens[p], joints, links))
            # calculate relavent data from data array
            # data_res, data_time, suc_res, avg_time, mu_min, z_min, LCI_min = self.assign_data(data)
            # all_data.append([datetime.now().strftime("%d/%m/%y, %H:%M"), self.arms[arm]["name"],
            #     data_res, str(data_time), suc_res,  str(avg_time), str(mu_min), str(z_min), str(LCI_min)])
            all_data.append(self.assign_data(data, arm))
            if arm == len(self.arms) - 1:
                break
            self.replace_model(arm)
        # save the remaining data and close all the launch files
        HandleCSV().save_data(all_data, save_name)
        self.ros.stop_launch(self.arm_control)
        self.ros.stop_launch(self.main)


if __name__ == '__main__':

    tic_main = datetime.now()
    dofe = 6
    ros = Ros()
    ros.ter_command("rosclean purge -y")
    roscore = ros.checkroscorerun()
    if roscore:
        ros.ter_command("kill -9 " + str(roscore))
    ros.ros_core_start()
    init_node('arl_python', anonymous=True)
    foldere = "combined"
    sim = Simulator(dofe, foldere, True)
    arms = sim.arms
    nums = 30  # how many arms to send to simulator each time
    for t in range(len(arms) / nums + 1):
        if t == len(arms) / nums:
            sim = Simulator(dofe, foldere, False, arms[t * nums:])
            sim.run_simulation(nums*t, len(arms))
        elif t != 0:
            sim = Simulator(dofe, foldere, False, arms[t * nums:(t + 1) * nums])
            sim.run_simulation(nums*t, len(arms))
        else:
            sim.arms = arms[:nums]
            sim.run_simulation(nums*t, len(arms))
    ros.ros_core_stop()
    toc_main = datetime.now()
    print('Time of Run (seconds): ' + str((toc_main - tic_main).seconds))

# Done - set for first joint the current location as target.
# done  calculate indicies
# todO change links weight according length
# done delete base link visuality and change link0 to box
# todo change link0 to the platform -
# done -add roll for each manipulator in last joint
# TodO add floor at 3 meter --disabled
