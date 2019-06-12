from ros import Ros, UrdfClass, HandleCSV
import datetime
import os
import numpy as np
import itertools
import time


class Simulator(object):

    def __init__(self, dof, folder, create=False):
        self.dof = dof
        self.folder = folder
        self.ros = Ros()  # for work with Ros
        self.ros.ros_core_start()
        self.arm_control = 0
        self.arms = []
        if create:  # all the configuration of the arms
            self.create_urdf_from_csv(str(self.dof) + "dof_configs")
        else:
            self.arms_exist()
        # x-terminal-emulator -e
        self.main_launch = self.ros.ter_command("roslaunch man_gazebo main.launch gazebo_gui:=false rviz:=false")
        self.ros.pub_sub_init(pub_name="Moveit_Sub", sub_name="Moveit_Pub")
        time.sleep(8)
        moveit_ready = self.handle_msg("Moveit Ready")
        if moveit_ready != "Moveit Ready":
            print "moveit doesn't ready"
        self.replace_model(0)  # set the first arm

    @staticmethod
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

    def set_links_length(self, min_length=1):
        link_min = 0.1
        link_interval = 0.3
        link_max = 0.41
        links = []
        lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
        links_length = [[0.1] + list(tup) for tup in
                        list(itertools.product(lengths_2_check, repeat=(self.dof - 1)))]
        for link in links_length:
            if sum(link) > min_length:
                links.append([str(x) for x in link])
        return links

    def create_urdf_from_csv(self, csv_name="manips"):
        # read from csv file with all the possible configuration for manipulators
        configs = HandleCSV().read_data(csv_name)
        # Create the urdf files
        data = []
        # folder = ""
        # c = 0
        base_path = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/"
        folder = "combined3"
        self.create_folder(base_path + "6dof/" + folder)
        links = self.set_links_length()
        index = 0
        for config in configs:
            for arm in config:
                # for i in range(len(arm["joint"])):
                #     folder = folder + arm["joint"][i] + "_" + arm["axe"][i] + "_"
                # self.create_folder(base_path + str(len(arm["axe"])) + "dof/" + folder)
                for link in links:
                    self.arms.append(self.create_arm(arm["joint"], arm["axe"], link, folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + folder + "/"
                    # index = config.index(arm) + links.index(link) + c
                    self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                                                       path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"], folder, datetime.datetime.now().strftime("%d_%m_%y")])
                    index = index+1
                # folder = ""
            # c = c + len(config) * len(links)
        HandleCSV().save_data(data, "created files")

    def arms_exist(self):
        path = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/" + self.folder

        for fil in os.listdir(path):
            fol = self.folder.split("/")
            self.arms.append({"name": fil.replace(".urdf.xacro", ""), "folder": fol[1]})
        # self.arms.reverse()

    @staticmethod
    def create_folder(name):
        if not os.path.exists(name):
            os.mkdir(name)
        return name

    def replace_model(self, arm):
        fil = "man:=" + self.arms[arm + 1]["folder"] + "/" + self.arms[arm + 1]["name"] + " dof:=" + str(self.dof) + "dof"
        if self.arm_control != 0:
            self.ros.stop_launch(self.arm_control)  # this launch file must be stopped, otherwise it wont work
        replace_command = "x-terminal-emulator -e roslaunch man_gazebo replace_model.launch " + fil
        self.ros.ter_command(replace_command)
        time.sleep(2)
        self.arm_control = self.ros.start_launch("arm_controller", "man_gazebo", ["dof:=" + str(self.dof) + "dof"])
        time.sleep(2)

    def handle_msg(self, check_msg=""):
        i = 0
        msg = ""
        for i in range(100):
            msg = self.ros.data_back
            if msg != check_msg:
                # self.ros.talker("Massage Recieved @ Gazebo")
                break
            time.sleep(0.05)
        if i == 100:  # timeout
            self.ros.talker("No Massage Recieved @ Gazebo")
        self.ros.data_back = ""
        return msg

    def run_simulation(self):
        flag = "Start"
        arm = 0
        while flag != "Stop":
            # for arm in range(0, len(self.arms)):
            # if arm % 2 == 0 and arm != 0:
            #     # self.ros.ter_command("rosnode kill /robot_state_publisher")
            #     rosnode.kill_nodes(rosnode.get_node_names())
            #     self.manipulator_move.stop_moveit()
            #     self.manipulator_move = None
            #     self.ros.stop_launch(self.arm_control)
            #     self.ros.ter_command("kill -9 " + str(self.ros.checkroscorerun()))
            #     time.sleep(3)
            #     self.main_launch = self.ros.ter_command("x-terminal-emulator -e roslaunch man_gazebo main.launch gazebo_gui:=false rviz:=false")
            #     time.sleep(3)  # need time to upload
            #
            #     self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
            #     time.sleep(0.5)  # need time to upload
            #     # add floor and plant to the planning model
            #     self.manipulator_move.add_obstacles(height=6.75, radius=0.1, pose=[0.5, 0])
            #     time.sleep(0.5)
            #     self.manipulator_move.go_to_pose_goal(self.poses[0], self.oriens[0])
            #     self.replace_model(arm-1)  # set the first arm
            #
            print "arm " + str(arm + 1) + " of " + str(len(self.arms)) + " arms"
            self.ros.talker(self.arms[arm]["name"])
            if arm == len(self.arms) - 1:
                break
            moveit_finish = self.handle_msg()
            if moveit_finish == "Massage Recieved":
                self.replace_model(arm)
                arm = arm + 1

        self.ros.talker("Stop")
        self.ros.stop_launch(self.arm_control)
        self.ros.ter_command("kill -9 " + str(self.ros.checkroscorerun()))


tic = datetime.datetime.now()
dofe = 6
foldere = "6dof/combined"
sim = Simulator(dofe, foldere, True)
sim.run_simulation()
toc = datetime.datetime.now()
print('Time of Run (seconds): ' + str((toc - tic).seconds))
