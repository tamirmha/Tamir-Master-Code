from ros import Ros, MoveGroupPythonInterface, UrdfClass, HandleCSV
import datetime
import os
import numpy as np
import itertools
import time
import rospy
import tf


class Simulator(object):

    def __init__(self, dof, folder, create=False, arms=[]):
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
        self.poses = [[0.5, 0.15, z + 0.86], [0.5, 0.0, z + 0.89], [0.5, -0.15, z + 0.86], [0.5, -0.15, z + 0.45], [0.5, 0.15, z + 0.45]]
        self.oriens = [[1.98, -0.83, 0], [-3.14, 0, 0], [-1.98, -0.83, 0], [-0.81, 0.52, 0], [0.9, 0.02, 0]]
        # for some reason the 1st manipulator must succeed reach to point otherwise the other manipulators will failed
        main_launch_arg = ["gazebo_gui:=false", "rviz:=false", "dof:=" + str(self.dof) + "dof"]
        self.main = self.ros.start_launch("main", "man_gazebo", main_launch_arg)  # main launch file
        # set the obstacles and initiliaze the manipulator
        self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        # add floor and plant to the planning model
        time.sleep(0.13)
        self.manipulator_move.add_obstacles(height=0.75, radius=0.1, pose=[0.5, 0])
        # pos = self.manipulator_move.move_group.get_current_pose().pose.position
        # a = self.manipulator_move.move_group.get_current_pose().pose.orientation
        # orien = (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])) - 2 * np.pi) % (2 * np.pi)
        # self.manipulator_move.go_to_pose_goal([pos.x, pos.y, pos.z], orien)
        pos = self.manipulator_move.get_current_position()
        orien = self.manipulator_move.get_current_orientain()
        self.manipulator_move.go_to_pose_goal([pos.x, pos.y, pos.z], [orien[0], orien[1], orien[2]])

        # self.manipulator_move.go_to_pose_goal(self.poses[0], self.oriens[0])
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
        base_path = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/"
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
                    data.append([self.arms[index]["name"], self.folder, datetime.datetime.now().strftime("%d_%m_%y")])
                    index = index+1
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
        replace_command = "roslaunch man_gazebo replace_model.launch " + fil
        self.ros.ter_command(replace_command)
        time.sleep(1.4)
        self.arm_control = self.ros.start_launch("arm_controller", "man_gazebo", ["dof:=" + str(self.dof) + "dof"])
        time.sleep(1)

    def run_simulation(self,  k=0, len_arm=1638):
        save_name = 'results_file' + datetime.datetime.now().strftime("%d_%m_%y")  # file to save the results
        all_data = []
        for arm in range(0, len(self.arms)):
            print "arm " + str(arm + 1 + k) + " of " + str(len_arm) + " arms"
            data = []
            for p in range(len(self.poses)):  # send the manipulator to the selected points
                data.append(self.manipulator_move.go_to_pose_goal(self.poses[p], self.oriens[p]))
            # inserting the data into array
            data_res = str([i[0] for i in data])
            suc_res = "False"
            if data_res.count("True") >= 3:  # if the arm arrived to 3 or more point than success
                suc_res = "True"
            data_time = [i[1] for i in data]
            avg_time = np.mean(data_time)
            all_data.append([datetime.datetime.now().strftime("%d/%m/%y, %H:%M"), self.arms[arm]["name"],#",".join(data),
                            data_res, str(data_time), suc_res,  str(avg_time)])
            if arm == len(self.arms) - 1:
                break
            self.replace_model(arm)
        # save the remaining data and close all the launch files
        HandleCSV().save_data(all_data, save_name)
        self.ros.stop_launch(self.arm_control)
        self.ros.stop_launch(self.main)


if __name__ == '__main__':

    tic_main = datetime.datetime.now()
    dofe = 6
    ros = Ros()
    ros.ter_command("rosclean purge -y")
    roscore = ros.checkroscorerun()
    if roscore:
        ros.ter_command("kill -9 " + str(roscore))
    ros.ros_core_start()
    rospy.init_node('arl_python', anonymous=True)
    foldere = "combined"
    sim = Simulator(dofe, foldere, True)
    arms = sim.arms
    nums = 40  # how many arms to send to simulator each time
    for i in range(2):  # len(arms) / nums + 1):

        if i == len(arms) / nums:
            sim = Simulator(dofe, foldere, False, arms[i * nums:])
            sim.run_simulation(nums*i, len(arms))
        elif i != 0:
            sim = Simulator(dofe, foldere, False, arms[i * nums:(i + 1) * nums])
            sim.run_simulation(nums*i, len(arms))
        else:
            sim.arms = arms[:nums]
            sim.run_simulation(nums*i, len(arms))
    ros.ros_core_stop()
    toc_main = datetime.datetime.now()
    print('Time of Run (seconds): ' + str((toc_main - tic_main).seconds))

# Done - set for first joint the current location as target.
# Todo check to change tolerance for detection point z-orientaion
# todo change planner
# toDo calculate indicies
# tOdo finish workspace
# todO change links weight according length
