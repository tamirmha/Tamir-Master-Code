# coding=utf-8
""" Variables: x1 : Joints type - array [Roll, Pitch, Pris]
            x2: Previous axe - array [X, Y, Z]
            x3 : Link Length - array [0.1, 0.4, 0.7]
            x4 : DOF - Int [3, 4, 5, 6]
Objectives:
1)	Min  Degree of Redundancy     [-3:0]
2)	Max  manipulability [0-1]
3)	Min  z (Mid-Range Proximity)
Constrains:
•	Sum (X3) > 1
•	X1[0] = Roll
•	X2[0] = Z
•	X3[0]=0.1
•	No more than 3 Pris in X1
•	If (X1[i]==Roll and X2[i]==Z) than (X1[i+1]!=Roll and X2[i+1]!=Z)
•	Arrival points : reach to one from two upper points and to the middle and bottom points
"""

from ros import UrdfClass
import Other
# import datetime
# import pymoo
# import csv
# import os
from scipy.spatial import distance
import numpy as np


# class Configs:
#     def __init__(self):
#         configs = self.read_data("all_configs")
#         possible_configs = {"3": [], "4": [], "5": [], "6": []}
#         for config in configs:
#             if config[0]["dof"] == 3:
#                 possible_configs["3"].append(config[0])
#             elif config[0]["dof"] == 4:
#                 possible_configs["4"].append(config[0])
#             elif config[0]["dof"] == 5:
#                 possible_configs["5"].append(config[0])
#             elif config[0]["dof"] == 6:
#                 possible_configs["6"].append(config[0])
#         self.possible_configs = possible_configs
#
#     def read_data(self, file_name):
#         with open(file_name + ".csv", 'r') as _filehandler:
#             csv_file_reader = csv.reader(_filehandler)
#             data = []
#             manip = []
#             empty = True
#             for row in csv_file_reader:
#                 while "" in row:
#                     row.remove("")
#                 if len(row) > 0:
#                     data.append([row[0].strip("\"").strip(" "), row[1]])
#                     empty = False
#                 else:
#                     if not empty:
#                         manip.append(self.read_data_action(data))
#                         data = []
#                     empty = True
#             # manip.append(self.read_data_action(data))  # append the last session
#             return manip
#
#     @staticmethod
#     def read_data_action(data):
#         manip = list(map(list, zip(*data[0:])))
#         manip_array_of_dict = []
#         for i in range(0, len(manip) - 1, 2):
#             manip_array_of_dict.append({"joint": manip[i], "axe": manip[i + 1], "dof": len(manip[i])})
#         return manip_array_of_dict


def to_urdf(interface_joints, joint_parent_axis, links, folder):
    """Create the desired confiuration
            interface_joints- roll,pitch or prismatic
                             roll - revolute around own Z axe
                             pitch - revolute that not roll
                             pris - prismatic along z
            links - length of links
            joint_parent_axis - the axe, in the parent frame, which each joint use
        """
    joints = []
    joint_axis = []
    rpy = []
    # file_name = os.environ['HOME'] + "\Tamir_Ws\src\manipulator_ros\Manipulator\man_gazebo\urdf\"
    # + str(dof) + "dof\combined\"
    file_name = ""
    rolly_number = -1
    pitchz_number = 1
    prisy_number = -1
    for i in range(len(joint_parent_axis)):
        # file_name += interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + \
        #              links[i].replace(".", "_")
        file_name += interface_joints[i] + "_" + joint_parent_axis[i] + "_" + str(links[i]).replace(".", "_")
        if interface_joints[i] == "roll":
            joints.append("revolute")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                rolly_rot = '${' + str(rolly_number) + '/2*pi} '
                rpy.append([rolly_rot, '0 ', '0 '])
                rolly_number = rolly_number * -1
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
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                pitchz = '${' + str(pitchz_number) + '/2*pi} '
                rpy.append([pitchz, '0 ', '0 '])
                pitchz_number = pitchz_number * -1
        elif interface_joints[i] == "pris":
            joints.append("prismatic")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                prisy = '${' + str(prisy_number) + '/2*pi} '
                rpy.append([prisy, '0 ', '0 '])
                prisy_number = prisy_number * -1
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '${-pi/2} ', '0 '])
            elif joint_parent_axis[i] == "z":
                rpy.append(['0 ', '0 ', '0 '])
    arm = UrdfClass(links, joints, joint_axis, rpy)
    # arm.urdf_write(arm.urdf_data(), file_name)
    return {"arm": arm, "name": file_name, "folder": folder}


class Problem:
    def __init__(self, concept_name, confs_of_concepts, number_of_arms=100, number_of_objects=3, large_concept=200):
        #          x1_options=None, x2_options=None, x3_options=None, x4_options=None):
        # if x1_options is None:
        #     x1_options = ["roll", "pitch", "pris"]
        # if x2_options is None:
        #     x2_options = ["x", "y", "z"]
        # if x3_options is None:
        #     x3_options = [0.1, 0.4, 0.7]
        # if x4_options is None:
        #     x4_options = [4, 5, 6]
        # self.x1_options = x1_options
        # self.x2_options = x2_options
        # self.x3_options = x3_options
        # self.x4_options = x4_options
        # self.possibles_configs = Configs().possible_configs  # get all the possible configurations
        self.number_of_arms = number_of_arms  # how many arms to create
        self.confs_of_concepts = confs_of_concepts  # all possible configuration of the concept
        self.number_of_objects = number_of_objects
        self.large_concept = len(confs_of_concepts) > large_concept  # True if large False otherwise
        self.concept_name = concept_name
        self.confs_archive = []  # Archive of all the selected configurations
        self.dof = int(str(concept_name).split(" ")[5].split(",")[0])

    def rand_pop(self, pop_size=25):
        """ select random configurations that belongs to the concept"""
        confs_of_concepts = self.get_configs()
        confs_archive = self.get_prev_confs()
        remain_confs = len(confs_of_concepts) - len(confs_archive)
        # check that the number of selected configuration wont be bigger than the number of remain configurations
        if remain_confs < pop_size:
            pop_size = remain_confs
        num_of_confs = len(confs_of_concepts)
        # select random indices
        indices = np.random.randint(0, num_of_confs, pop_size)
        # get the configurations of the random indices
        random_confs = np.ndarray.tolist(np.asarray(confs_of_concepts)[indices])
        confs = []
        while len(random_confs) != 0:
            for conf in random_confs:
                # if the random configuration in the archive
                if conf in confs_archive:
                    # delete the configuration and select new one
                    random_confs.remove(conf)
                    new_ind = np.random.randint(0, num_of_confs, 1)
                    while new_ind in indices:
                        new_ind = np.random.randint(0, num_of_confs, 1)
                    random_confs.append(confs_of_concepts[new_ind[0]])
                else:
                    confs.append(conf)
                    random_confs.remove(conf)
        a = 3
        return confs

    def evalute(self, pop):
        # simulator
        f3 = np.around(np.random.randint(4, 7, pop.shape[0]), 3)  # dof
        f2 = np.around(np.random.uniform(0, 1, pop.shape[0]), 3)  # manipulability
        f1 = np.around(np.random.uniform(0, 0.5, pop.shape[0]), 3)   # (Mid-Range Proximity)
        return [f1, f2, f3, pop, self.concept_name]

    def stop_condition(self):
        """Stop condition of the concept (not of all the problem)
        condition 1 - if all the configuration been checked
        """
        stop = False
        if self.get_prev_confs().shape[0] == self.get_configs().shape[0]:
            stop = True

        return False

    @staticmethod
    def assign_fitness(points, dwoi):
        """ calculate the distance from each point to the DWOI"""
        dwoi_loc = np.asarray(dwoi[:3]).T   # np.asarray([[i[1], i[0], i[2]] for i in dwoi])
        dist = distance.cdist(dwoi_loc,  np.asarray(points[:3]).T, 'euclidean')
        return np.around(dist, 3)

    @staticmethod
    def selection(distanc, num_of_returns=2):
        """Make selection by using RWS return the value
        :param distanc: array of distances
        :param num_of_returns: how many elements to return
        :return selected: the selected values that won the RWS
        """
        selected = []
        distanc = 1 - distanc
        fp = np.asarray([i / sum(distanc) for i in distanc])
        for i in range(num_of_returns):
            wheel = np.random.uniform(0, 1)
            selected.append(round(distanc[(np.abs(fp - wheel).argmin())], 3))
        return [1-x for x in selected]

    def mating(self, parents):
        """ """
        parent_1 = np.asarray(Other.Concepts.arm2parts(str(parents[0]).split("_")))
        parent_2 = np.asarray(Other.Concepts.arm2parts(str(parents[1]).split("_")))
        offspring = []
        offspring.append(self.crossover([parent_1, parent_2]))
        offspring.append(self.mutation([parent_1, parent_2]))
        return offspring

    def crossover(self, parents):
        """ select a random number (link) and all the links&joints&axes until this point
        taken from parent 1 and all the links&joints&axes from this point are taken from parent 2
        """
        dof = self.dof
        for t in range(dof):
            point_of_split = np.random.randint(1, dof)
            child = np.ndarray.tolist(parents[0][:, :point_of_split].copy())
            [child[i].extend(np.ndarray.tolist(parents[1][i, point_of_split:])) for i in range(3)]
            child = to_urdf(child[0], child[1], child[2], "")
            if self.check_conf(child["name"]):
                break
        if t == dof - 1:
            child = np.ndarray.tolist(parents[0])
            child = to_urdf(child[0], child[1], child[2], "")
        return child["name"]

    def mutation(self, parents):
        """ switch randomlly 2 links and joints """
        dof = self.dof
        ind = np.random.randint(0, len(parents))
        parent = parents[ind]
        inds = np.arange(1, dof)
        np.random.shuffle(inds)
        inds = np.insert(inds, 0, 0)
        child = parent[:, inds]
        child = to_urdf(child[0], child[1], child[2], "")

    @staticmethod
    def confs_by_indices(select, fit):
        """ get the selected distances and return the indices of them
         if distance appear more than once it than take the second item
         """
        indices = []
        for x in select:
            ind = np.argwhere(fit == round(x, 3))
            print(ind)
            if ind.shape[0] > 1:
                fit[ind[0][0]] = 100
            indices.append(ind[0][0])
        return indices

    def get_conifgs_by_indices(self, conf_indices):
        """get configuration by indices"""
        get_configs = np.asarray(self.get_configs())
        return np.ndarray.tolist(np.unique(get_configs[conf_indices]))

    def check_conf(self, confs):
        """Ceck if the configurations are belongs to the concept
        :param confs: list of configurations to check        """
        concepts = self.get_configs()
        return confs in concepts
        # ind = [i for i in range(confs.shape) if confs[i] in concepts]
        # return confs[ind]

    @staticmethod
    def clost_point(distances, nums_mins=4):
        """Find the closest point to the DWOI
        :param distances: NxM array-N number of points in the DWOI, M number of points to calc distance
        :param nums_mins: number of minimum points to find
        return: the index of the closest point
        """
        indices = []
        print(distances.shape[0], distances.shape[1])
        for i in range(nums_mins):
            ind = distances.argmin() % distances.shape[1]
            indices.append(ind)
            distances[:, ind] = [100] * distances.shape[0]
            # distances = distances[:, range(ind) + range(ind + 1, distances.shape[1])]
        return indices

    def get_configs(self):
        return self.confs_of_concepts

    def domination_check(self, conf, front):
        """ check if any of the configuration dominated the DWOI and update the DWOI"""
        for i, j, k, l in zip(conf[0], conf[1], conf[2], conf[3]):  # z, mu, dof, configuration
            added = False
            for i_front, j_front, k_front, l_front in zip(front[0], front[1], front[2], front[3]):
                # check if the point is dominate the front
                if i <= i_front and j <= j_front and k <= k_front:
                    ind = front[3].index(l_front)
                    del front[0][ind]
                    del front[1][ind]
                    del front[2][ind]
                    del front[3][ind]
                    del front[4][ind]
                    if not added:
                        front[0].append(i)
                        front[1].append(j)
                        front[2].append(k)
                        front[3].append(l)
                        front[4].append(self.concept_name)
                        added = True
            if not added:
                front[0].append(i)
                front[1].append(j)
                front[2].append(k)
                front[3].append(l)
                front[4].append(self.concept_name)
        return front

    def set_prev_confs(self, confs):
        """Add configurtions to the archive """
        self.confs_archive.append(confs)

    def get_prev_confs(self):
        """ Return the configuration in the archive"""
        return self.confs_archive

    # def get_random_values(self):
    #     # set random
    #     x4 = random.randrange(self.x4_options[0], self.x4_options[-1]+1, 1)  # number degrees of freedom
    #     # the first joint is roll
    #     x1 = ["roll"]
    #     x2 = ["z"]
    #     x3 = [0.1]
    #     x = [[], [], [], x4]
    #     k = 0
    #     for i in range(1, self.number_of_arms * x4 - self.number_of_arms + 1):
    #         x1.append(random.choice(self.x1_options))
    #         x2.append(random.choice(self.x2_options))
    #         x3.append(random.choice(self.x3_options))
    #         if i % (x4-1) == 0 and i != 0:
    #             k += 1
    #             x[0].append(x1)
    #             x[1].append(x2)
    #             x[2].append(x3)
    #             x1 = ["roll"]
    #             x2 = ["z"]
    #             x3 = [0.1]
    #     return x
    #
    # def constrains(self, x):
    #     # filter the combinations according to constrains
    #     to_sim = []
    #     for i in range(self.number_of_arms):
    #         if sum(x[2][i]) > 1:
    #             for conf in self.possibles_configs[str(x[3])]:
    #                 if x[0][i] == conf["joint"] and x[1][i] == conf["axe"]:
    #                     to_sim.append(to_urdf(x[0][i], x[1][i], x[2][i], x[3]))
    #                     # print(conf["joint"])
    #                     break
    #     return to_sim


class DWOI:

    def __init__(self):
        self.gen = 0
        self.dwoi = self.dwoi2conf(Other.load_json("front_concept"))  # , self.gen])
        # self.dwoi_archive = [[], self.gen]

    def set_dwoi(self, dwoi):
        self.dwoi.append(dwoi)

    def get_all_dwoi(self):
        return self.dwoi

    def get_last_dwoi(self):
        return self.dwoi[-1]

    @staticmethod
    def dwoi2conf(dwoi):
        """ conf from the following format: 4D list:
        0 - list of z
        1 - list of mu
        2 - list of dof
        3 - list of configurations names
        4 - concept
        5 - number of gen
        """
        conf = [[], [], [], [], []]  # , dwoi[1]]
        for w in dwoi:
            conf[0].append(w["z"])
            conf[1].append(w["mu"])
            conf[2].append(w["dof"])
            conf[3].append(w["configuration"])
            conf[4].append(w["concept"])
        return [conf]

    def set_gen(self, gen):
        self.gen = gen

    def get_gen(self):
        return self.gen


test_points = np.asarray([[0.05, 0.57, 4], [0.09, 0.94, 5], [0.5, 0.8, 6], [0.5, 0.1, 4], [0.9, 0.94, 4], [0.5, 0.8, 4]])
concepts_with_conf = Other.load_json("concepts")
# dists = np.asarray([1, 6, 8, 9, 10], dtype=float)

woi = DWOI()

name_of_concept = list(concepts_with_conf)[0]
prob = Problem(name_of_concept, confs_of_concepts=concepts_with_conf[name_of_concept], large_concept=200)
# initiliaze population
population = prob.rand_pop(25)
# how many gens to run
num_gens = 10

for n in range(num_gens):
    # Evaluation
    confs_results = prob.evalute(np.asarray(population))
    # Update DWOI if necessary
    woi.set_dwoi(prob.domination_check(confs_results, woi.get_last_dwoi()))
    # Assign fitness
    dis = prob.assign_fitness(confs_results, woi.get_last_dwoi())  # the distance of all the configurations
    fitness = np.amin(dis, axis=0)  # calc the minimum distance for each configuration
    # fitness1 = np.asarray(confs_results[:4])[:, prob.clost_point(fitness)]

    # Stop Condition
    # todo - add Condition stop
    # check if large concept
    if prob.large_concept:  # if large concept
        # Selection (RWS)
        selection = prob.selection(fitness)
        selected_confs_ind = prob.confs_by_indices(selection, fitness)
        selected_confs = prob.get_conifgs_by_indices(selected_confs_ind)
        # todo - add faunction of mating
        population = prob.mating(selected_confs)

    else:  # if small concept
        # Random Selection
        population = prob.rand_pop(25)

    # pop_to_mating = prob.check_conf(selected_confs)
    # update generation
    woi.set_gen(n+1)

# TodO - if configuration allrady selected?
# todo - save dwoi to json every iteration?
# todo - how to evalute?  run simulation for all ?
# todo - how to get the results from the simulator
# todo - problematic urdf to make list and pre create
# todo - to check from main results fail if allready simulated
# tqdm
