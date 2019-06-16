from ros import Ros, MoveGroupPythonInterface, UrdfClass, HandleCSV
import datetime
import os
import numpy as np
import itertools
import time
import rospy


class Simulator(object):

    def __init__(self, dof, folder, create=False, arms=[]):
        self.dof = dof
        self.folder = folder
        self.ros = Ros()  # for work with Ros
        # self.ros.ros_core_start()
        self.arm_control = 0
        self.arms = []
        if create:  # all the configuration of the arms
            self.create_urdf_from_csv(str(self.dof) + "dof_configs")
        else:
            if arms:
                self.arms_exist()
            else:
                self.arms = arms
        # desired positions and orientaions of the EE in world frame
        self.poses = [[0.5, 0.15, 6.86], [0.5, 0.0, 6.89], [0.5, -0.15, 6.86], [0.5, -0.15, 6.45], [0.5, 0.15, 6.45]]
        self.oriens = [[1.98, -0.83, 0], [-3.14, 0, 0], [-1.98, -0.83, 0], [-0.81, 0.52, 0], [0.9, 0.02, 0]]
        # set the obstacles and initiliaze the manipulator
        # for some reason the 1st manipulator must succeed reach to point otherwise the other manipulators will failed

        main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:=" + str(self.dof) + "dof"]
        self.main = self.ros.start_launch("main", "man_gazebo", main_launch_arg)  # main launch file
        time.sleep(1)  # need time to upload
        self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        time.sleep(1)  # need time to upload
        # add floor and plant to the planning model
        self.manipulator_move.add_obstacles(height=6.75, radius=0.1, pose=[0.5, 0])
        time.sleep(1)
        self.manipulator_move.go_to_pose_goal(self.poses[0], self.oriens[0])
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
        # folder = "combined3"
        self.create_folder(base_path + "6dof/" + self.folder)
        links = self.set_links_length()
        index = 0
        for config in configs:
            for arm in config:
                # for i in range(len(arm["joint"])):
                #     folder = folder + arm["joint"][i] + "_" + arm["axe"][i] + "_"
                # self.create_folder(base_path + str(len(arm["axe"])) + "dof/" + folder)
                for link in links:
                    self.arms.append(self.create_arm(arm["joint"], arm["axe"], link, self.folder))
                    path = base_path + str(len(arm["axe"])) + "dof/" + self.folder + "/"
                    # index = config.index(arm) + links.index(link) + c
                    self.arms[index]["arm"].urdf_write(self.arms[index]["arm"].urdf_data(),
                                                       path + self.arms[index]["name"])
                    data.append([self.arms[index]["name"], self.folder, datetime.datetime.now().strftime("%d_%m_%y")])
                    index = index+1
                # folder = ""
            # c = c + len(config) * len(links)
        HandleCSV().save_data(data, "created files")

    def arms_exist(self):
        path = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/" + str(self.dof) \
               + "dof/" + self.folder

        for fil in os.listdir(path):
            fol = self.folder.split("/")
            self.arms.append({"name": fil.replace(".urdf.xacro", ""), "folder": fol[0]})

    @staticmethod
    def create_folder(name):
        if not os.path.exists(name):
            os.mkdir(name)
        return name

    def replace_model(self, arm):
        fil = "man:=" + self.arms[arm + 1]["folder"] + "/" + self.arms[arm + 1]["name"] + " dof:=" + str(self.dof) + "dof"
        if self.arm_control != 0:
            self.ros.stop_launch(self.arm_control)  # this launch file must be stopped, otherwise it wont work
        # self.ros.start_launch("replace_model", "man_gazebo", fil)
        replace_command = "x-terminal-emulator -e roslaunch man_gazebo replace_model.launch " + fil
        self.ros.ter_command(replace_command)
        time.sleep(1.5)
        self.arm_control = self.ros.start_launch("arm_controller", "man_gazebo", ["dof:=" + str(self.dof) + "dof"])
        time.sleep(1)

    def run_simulation(self,  k=0, len_arm=1638):
        # if len(arms) > 0:
        #     self.arms = arms
        save_name = 'results_file' + datetime.datetime.now().strftime("%d_%m_%y")  # file to save the results
        all_data = [["Date", "Time ", "Arm ", "Results "]]
        for arm in range(0, len(self.arms)):
            print "arm " + str((arm + 1)*(k+1)) + " of " + str(len_arm) + " arms"
            data = []
            for i in range(len(self.poses)):  # send the manipulator to the selected points
                data.append(str(self.manipulator_move.go_to_pose_goal(self.poses[i], self.oriens[i])))
            # inserting the data into array
            all_data.append([datetime.datetime.now().strftime("%d/%m/%y, %H:%M"),
                             self.arms[arm]["name"], ",".join(data)])
            if arm == len(self.arms) - 1:
                break
            self.replace_model(arm)
        # save the remaining data and close all the launch files
        HandleCSV().save_data(all_data, save_name)
        self.ros.stop_launch(self.arm_control)
        self.ros.stop_launch(self.main)


if __name__ == '__main__':

    tic = datetime.datetime.now()
    dofe = 6
    ros = Ros()
    roscore = ros.checkroscorerun()
    if roscore:
        ros.ter_command("kill -9 " + str(roscore))
    ros.ros_core_start()
    rospy.init_node('arl_python', anonymous=True)
    foldere = "combined"
    sim = Simulator(dofe, foldere, True)
    arms = sim.arms

    nums = 40  # how many arms to send to simulator each time
    for i in range(len(arms) / nums + 1):
        if i == len(arms) / nums:
            sim = Simulator(dofe, foldere, False, arms[i * nums:])
            sim.run_simulation(i, len(arms))
        elif i != 0:
            sim = Simulator(dofe, foldere, False, arms[i * nums:(i + 1) * nums])
            sim.run_simulation(i, len(arms))
        else:
            sim.arms = arms[:nums]
            sim.run_simulation(i, len(arms))
        time.sleep(1.5)
    ros.ros_core_stop()
    toc = datetime.datetime.now()
    print('Time of Run (seconds): ' + str((toc - tic).seconds))
