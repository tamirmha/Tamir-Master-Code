from os import environ, listdir, mkdir, path
import shutil
import csv
import tkFileDialog
from Tkinter import *
import json
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from itertools import product
from simulator import Simulator
from matplotlib.tri import Triangulation
from tqdm import tqdm
import copy
import pickle
from multiprocessing import Pool


class MyCsv(object):

    @staticmethod
    def load_csv(file_name):
        with open(file_name + ".csv", "r") as data_file:
            csv_file_reader = csv.reader(data_file)
            result = []
            dof = data_file.name[data_file.name.find("dof")-1]
            for row in csv_file_reader:
                while "" in row:
                    row.remove("")
                if len(row) > 0:
                    if len(row) == 1:
                        row = row[0].split(",")
                    name = row[2]
                    joints = ["roll"]
                    prev_axe = ["z"]
                    link_length = ["0.1"]
                    arm = name.split("_")
                    for a in range(3, len(arm)-1):
                        if a % 3 == 0:
                            joints.append(arm[a][1:] + "_")
                        elif a % 3 == 1:
                            prev_axe.append(arm[a] + "_")
                        elif a % 3 == 2:
                            link_length.append(arm[a] + "." + arm[a+1][:1] + "_")
                    result.append({"name": row[2], "time": float(row[6]), "mu": float(row[7]), "Z": float(row[8]),
                         "dof": dof, "joints": "_".join(joints), "prev_axe": "_".join(prev_axe),
                                   "link_length": "_".join(link_length)})
            result.sort()
            return result

    @staticmethod
    def save_csv(data, file_name, csv_type="list"):
        """Save to csv format"""
        with open(file_name + ".csv", 'ab') as name:
            if csv_type == "list":
                writer = csv.writer(name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerows(data)
            elif csv_type == "dict":  # for dictionary
                headers = ["name", "joints", "link_length", "prev_axe", "dof", "mu", "time", "Z", "LCI", "Passed"]
                writer = csv.DictWriter(name, fieldnames=headers, delimiter=',')
                writer.writeheader()
                writer.writerows(data)

    @staticmethod
    def read_csv(file_name, csv_type="list"):
        csv.field_size_limit(1172750)
        with open(file_name + ".csv", 'r') as _filehandler:
            data = []
            all_data = []
            if csv_type == "list":
                csv_file_reader = csv.reader(_filehandler)
                empty = True
                for row in csv_file_reader:
                    while "" in row:
                        row.remove("")
                    if len(row) > 0:
                        if len(row) == 1:
                            row = row[0].split(",")
                        data.append(row)
                        empty = False
                    else:
                        if not empty:
                            all_data.append(data)
                            data = []
                        empty = True
            elif csv_type == "dict":
                csv_file_reader = csv.DictReader(_filehandler)
                for row in csv_file_reader:
                    data.append(row)
        if len(all_data) == 0:
            all_data = data
        else:
            all_data.append(data)
        return all_data


class MergeData(object):

    def __init__(self):
        root = Tk()
        root.update()
        self.files = tkFileDialog.askopenfilenames(filetypes=(("csv files", "*.csv"), ("all files", "*.*")))
        root.destroy()
        self.new_file_name = "results_all"  # + self.files[0][-18:-11]

    def merge(self):
        files = self.files
        new_file_name = self.new_file_name
        for i in range(len(files)):
            MyCsv.save_csv(MyCsv.read_csv(files[i][:-4]), new_file_name)
            try:
                self.fix_json(files[i][:-4])
                save_json(data=load_json(files[i][:-4] + "_fixed"), name=new_file_name)
            except:
                print("There are no Json files")

    # @staticmethod
    # def save_json(name="data_file", data=None):
    #     with open(name + ".json", "a") as write_file:
    #         json.dump(data, write_file, indent=2)
    #
    # @staticmethod
    # def load_json(name="data_file"):
    #     with open(name + ".json", "r") as read_file:
    #         return json.load(read_file)

    # @staticmethod
    # def fix_json(file_name):
    #     with open(file_name + ".json", 'r') as filehandler:
    #         file_reader = filehandler.readlines()
    #         data = []
    #         empty = True
    #         for row in file_reader:
    #             if len(row) > 0:
    #                 if '][' in row:
    #                     row = ',\n'
    #                 data.append(row)
    #                 empty = False
    #             else:
    #                 if not empty:
    #                     data = []
    #                 empty = True
    #     with open(file_name + ".json", 'w') as name:
    #         name.writelines(data)


class FixFromJson(object):

    def __init__(self, all_files=False):
        root = Tk()
        root.update()
        files = tkFileDialog.askopenfilenames(filetypes=(("csv files", "*.csv"), ("all files", "*.*")))
        root.destroy()
        self.all = all_files
        for fil in files:
            self.fix(fil[:-4])

    def fix(self, file2fix="/home/tamir/Tamir/Master/Code/test/results_4dof_4d_toMuch"):
        file_fixed = file2fix + "_fixed"
        to_fix = MyCsv.read_csv(file2fix)
        original_data = load_json(file2fix)
        for row in to_fix:
            if row[5] == "True":
                if row[8] == "-16" and not self.all:
                    to_fix = self.calc(row, to_fix, original_data)
                elif self.all:
                    to_fix = self.calc(row, to_fix, original_data)
        MyCsv.save_csv(to_fix, file_fixed)

    def calc(self, row, to_fix, original_data):
        for dat in original_data:
            if dat.keys()[0] == row[2] and dat[row[2]][1] != [-1, -1, -1, -1]:
                jacobians, curr_poss = dat[row[2]]
                mu = 1.1
                z = 0
                # lci = 1.1
                for jacobian, curr_pos in zip(jacobians, curr_poss):
                    if jacobian != -1:
                        mu_new, z_new = self.indices_calc(row[2], jacobian, curr_pos)
                        if mu > mu_new:
                            mu = mu_new
                        if z < z_new:
                            z = z_new
                        # if lci > lci_new:
                        #     lci = lci_new
                to_fix[to_fix.index(row)][7] = mu
                to_fix[to_fix.index(row)][8] = z
                # to_fix[to_fix.index(row)][9] = lci
        return to_fix

    def indices_calc(self, names, jacobian, cur_pos):
        cur_pos = np.asarray(cur_pos)
        jacobian = np.asarray(jacobian)
        j_ev = np.linalg.svd(jacobian, compute_uv=False)  # [1]  # singular (eighen) values
        # Manipulability index
        mu = round(np.product(j_ev), 3)  # self.manipulability_index(jacobian)
        # Local Conditioning Index
        # lci = round(j_ev[-1] / j_ev[0], 3)  # self.local_conditioning_index(jacobian)
        # Joint Mid-Range Proximity
        z = self.mid_joint_proximity(cur_pos, names)
        return mu, np.diag(z).max()

    @staticmethod
    def mid_joint_proximity(cur_pos, names):
        joints = ["roll"]
        link_length = ["0.1"]
        name = names.split("_")
        for a in range(3, len(name) - 1):
            if a % 3 == 0:
                joints.append(name[a][1:] + "_")
            elif a % 3 == 2:
                link_length.append(name[a] + "." + name[a + 1][:1])
        theta_mean = [0.75]
        to_norm = [1.5]
        for joint in joints:
            if "pris" not in joint:
                theta_mean.append(0)
                to_norm.append(2*np.pi)
            else:
                theta_mean.append(float(link_length[joints.index(joint)])/2)
                to_norm.append(float(link_length[joints.index(joint)]))
        dis = (cur_pos[:-1]-theta_mean)
        nor_dis = np.asarray(np.abs(dis))/np.asarray(to_norm)
        while np.isin(nor_dis > 1, True).argmax():
            # some simulations calculated with pris limit 2*length instead of length
            ind_wrong = np.isin(nor_dis > 1, True).argmax()
            to_norm[ind_wrong] = to_norm[ind_wrong] * 2
            nor_dis = np.asarray(np.abs(dis)) / np.asarray(to_norm)
        w = np.identity(len(joints)+1)*nor_dis  # weighted diagonal matrix
        z = np.around(0.5*np.transpose(nor_dis)*w, 3)
        return z


class Concepts:
    def __init__(self, file_name="/home/tamir/Tamir/Master/Code/all_configs"):
        self.concepts = []
        self.joints_combs = []
        self.links = []
        self.configurations = []
        self.set_possible_links_length(0.1, 0.71, 0.3, 1)
        self.set_possible_joints_order(file_name)
        self.concepts_with_configuration = []

    ###
    # for stage A - select random configurations from each concept and to simulate them
    @staticmethod
    def confs2simulate(concepts_with_values, min_configurations=25, percent_from_concept=3.0):
        """ Choose random configurations from each concept. the number of configurations is
        max(min_configurations, percent_from_concept * configurations in concept)
        :param concepts_with_values: list of all the concepts. All concept contains all its configurations
        :param min_configurations: minimum number of configurations to choose
        :param percent_from_concept: percent from each concept configurations to choose
        return: confs2sim- names of configurations to simulate
        """
        percent_from_concept = percent_from_concept/100.0
        confs2sim = []
        # cons_of_sim = []
        for k in concepts_with_values:
            if len(k[1]) < min_configurations:  # 1352 configurations  & 129 concepts
                confs2sim.append(k[1])
            else:
                number_of_confs2sim = int(len(k[1])*percent_from_concept)
                # check if the minimum desired percent is enough
                if number_of_confs2sim < min_configurations:  # 9875 confiurations & 395 concepts
                    number_of_confs2sim = min_configurations
                indices = np.arange(len(k[1]))                # 47970 confiurations & 270 concepts
                np.random.shuffle(indices)
                confs2sim.append(np.ndarray.tolist(np.take(k[1], indices[:number_of_confs2sim])))
            # cons_of_sim.append(k[0])
        return confs2sim  # , cons_of_sim

    @staticmethod
    def filter_confs(to_sim, results_all="results_all"):
        """ check if any of the selected configurations have been simulated before
        :param to_sim: all the selected configuration thats need to be simulated
        :param results_all: the of with all results
        return: sorted array without the configurations that allready simulated
                results: the results from the simulated configuration
        """
        simmed_results = MyCsv.read_csv(results_all, "dict")
        sim_allready = []
        results = []
        for i in simmed_results:
            for j in to_sim:
                if i["name"] in j:
                    sim_allready.append(i["name"])
                    results.append(i)
                    to_sim[to_sim.index(j)].remove(i["name"])
                    break
        to_sim.sort(key=len)
        return [x for x in to_sim if x], results

    ###
    # get configs names and create urdf files
    @staticmethod
    def arm2parts(arm):
        """ get arm name and disassemble it to (joints, axes, link length)
        :param arm: arm name (like in the urdf file) (string)
        return: joints: array with the joints type (array of string)
                prev_axe: array of the axes (array of string)
                link_length: array of the links lengths (array of string)
        """
        joints = ["roll"]
        prev_axe = ["z"]
        link_length = ["0.1"]
        for a in range(3, len(arm) - 1):
            if a % 3 == 0:
                joints.append(arm[a][1:])
            elif a % 3 == 1:
                prev_axe.append(arm[a])
            elif a % 3 == 2:
                link_length.append(arm[a] + "." + arm[a + 1][:1])
        return joints, prev_axe, link_length

    def create_files2sim(self, to_sim, confs_file5="urdf/5dof", confs_file6="urdf/6dof"):
        """ get configurations and save urdf files
        :param to_sim: list of configurations to create urdfs
        :param confs_file5: name of csv file with all 5dof configurations to create
        :param confs_file6: name of csv file with all 6dof configurations to create
        """
        conf_5dof = [[] for i in range(20)]  # 54, 50
        conf_6dof = [[] for i in range(20)]  # 60, 65
        i6 = 0
        i5 = 0
        i5_length = 0
        i6_length = 0
        for conf in to_sim:
            if 50 <= len(conf[0]) <= 54:
                conf_5dof[i5].append(conf)
                i5_length, i5 = self.create_urdfs(i5_length, i5, 5, conf)
            elif 60 <= len(conf[0]) <= 65:
                conf_6dof[i6].append(conf)
                i6_length, i6 = self.create_urdfs(i6_length, i6, 6, conf)
        conf_6dof = [x for x in conf_6dof if x]
        conf_5dof = [x for x in conf_5dof if x]
        for k in range(len(conf_5dof)):
            MyCsv.save_csv(conf_5dof[k], confs_file5)
        for k in range(len(conf_6dof)):
            MyCsv.save_csv(conf_6dof[k], confs_file6)

    def create_urdfs(self, i_length, i, dof, conf, confs_in_folder=6000, path="urdf/"):
        i_length += len(conf)
        path += str(dof) + "dof/"
        for c in conf:
            joints, prev_axe, link_length = self.arm2parts(c.split("_"))  # con.arm2parts(c.split("_"))
            arm = Simulator.create_arm(joints, prev_axe, link_length, "")
            # path = "tests/" + str(dof) + "dof/" + str(i) + "/"
            Simulator.create_folder(path)
            arm["arm"].urdf_write(arm["arm"].urdf_data(), path + arm["name"])
        if i_length > confs_in_folder:
            i += 1
            i_length = 0
        return i_length, i
    ###

    def calc(self):
        self.determine_combinations()
        self.set_configurations()
        self.assign_configuration2concept()
        data = self.get_concepts_with_configuration()
        concepts_with_values = [[k, v] for k, v in data.items() if v != []]
        combs_in_concept = []
        for c in concepts_with_values:
            combs_in_concept.append([c[0], len(c[1])])
        # MyCsv.save_csv(combs_in_concept, "concepts_sum")
        # save_json("concepts", data)
        return concepts_with_values
        # concepts_without_values = [[k, 0] for k, v in data.items() if v == []]
        # MyCsv.save_csv(concepts_without_values, "concepts_without_values")

    def determine_combinations(self):
        concepts_general = self.assign_concepts_vars_values()
        concept = self.concepts_combination(concepts_general)
        possible_concepts = self.filter_combinations(concept, concepts_general)
        self.set_concepts(self.concepts_to_dict(possible_concepts))

    @staticmethod
    def assign_concepts_vars_values():
        """ set all the concepts variables and each variable range (unfiltered)"""
        acc_length = [1.5, 2, 2.6, 3.1, 3.6]
        concept_general = {"dof": range(4, 7), "pitch_joint": [], "#long_link": [], "par_axes_y": [], "acc_length": [],
           "long_link": [0.4, 0.7], "p/r_ratio": [[0.0, 0.33, 1.0, 3.0], [0.0, 0.25, 0.67, 1.5], [0.0, 0.2, 0.5, 1.0]]}
        for c in concept_general["dof"]:
            concept_general["pitch_joint"].append(range(0, c))
            concept_general["#long_link"].append(range(0, c))
            concept_general["par_axes_y"].append([0]+range(2, c))
            concept_general["acc_length"].append(acc_length[:c - 1])
        return concept_general

    @staticmethod
    def concepts_combination(concept_general):
        """set the specific range for each variable (unfiltered)"""
        concepts = []  # all possible concepts
        for c in concept_general["dof"]:
            ind = concept_general["dof"].index(c)
            concepts.append(list(product(concept_general["#long_link"][ind], concept_general["pitch_joint"][ind],
                                concept_general["p/r_ratio"][ind], concept_general["par_axes_y"][ind],
                                concept_general["acc_length"][ind], concept_general["long_link"], repeat=1)))
        return concepts

    @staticmethod
    def filter_combinations(concepts, concept_general):
        # filter to only possible concepts
        possible_concepts = [[], [], []]
        k = 0
        for cs in concepts:
            for c in cs:
                # the sum of the links cant be more than the accumalated length
                if c[0] * 0.7 + 0.4 * (k+3-c[0]) < c[4] - 0.6 or c[0]*0.7+0.1*(k+4-c[0]) > c[4]:
                    continue
                # if there is longest link (c[0]) isn't possible to have longest link(c[5]) equal to 0.4
                if c[5] == 0.4 and c[0] > 0:
                    continue
                # doesnt posisble
                if c[5] == 0.7 and c[0] == 0:
                    continue
                # cant be more parallel in y than number of pitch joints
                if c[3] > c[1]:
                    continue
                # there is a realtion between the number of 'pitch_joint' and the 'p/r_ration'
                pr_ratio = concept_general["p/r_ratio"][k]
                if (c[1] == k + 1 and c[2] == pr_ratio[3]) or (
                        c[1] == k + 2 and (c[2] == pr_ratio[3] or c[2] == pr_ratio[2])) \
                        or (c[1] == k + 3 and c[2] != pr_ratio[0]):
                    continue
                possible_concepts[k].append(c)
            k += 1
        return possible_concepts

    @staticmethod
    def concepts_to_dict(concepts):
        concepts_dict = []
        dof = 4
        for conc in concepts:
            cons_dict = []
            for c in conc:
                con_dict = {"dof": dof, "#long_link": c[0], "pitch_joint": c[1], "p/r_ratio": c[2],
                            "par_axes_y": c[3], "acc_length": c[4], "long_link": c[5]}
                cons_dict.append(con_dict)
            concepts_dict.append(cons_dict)
            dof += 1
        return concepts_dict

    def set_possible_links_length(self, link_min, link_max, link_interval, min_length):
        links = [[], [], []]
        dofs = [4, 5, 6]
        lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
        for dof in dofs:
            links_length = [[0.1] + list(tup) for tup in list(product(lengths_2_check, repeat=(dof - 1)))]
            for link in links_length:
                if sum(link) > min_length:
                    links[dof-4].append([str(x) for x in link])
        self.links = links

    def set_configurations(self):
        links = self.get_possible_links_length()
        joints = self.get_possible_joints_order()
        configs = [[], [], []]
        k = 0
        for link in links:
            for length in link:
                for joint in joints:
                    if len(joint) == len(length):
                        conf = []
                        for i, j in zip(joint, length):
                            conf.append([" ".join(i), j])
                        configs[k].append(conf)
            k += 1
        self.configurations = configs

    def assign_configuration2concept(self):
        concepts_with_configuration = {}
        confs = self.get_configurations()
        concepts = self.get_concepts()
        for conf, concept in zip(confs, concepts):
            concepts_with_configuration.update(dict((str(el), []) for el in concept))
            for cf in conf:
                par_axes_y = 0
                par_axes_y_temp = 0
                conf_name = ""
                dof = len(cf)
                pitch = 0
                long_links = 0
                pris = 0
                acc_length = 0
                longest = 0.4
                for c in cf:
                    if "pitch" in "".join(c[0]) and "y" in "".join(c[0]):
                        par_axes_y_temp += 1
                    elif "pris" in "".join(c[0]) and "z" in "".join(c[0]):
                        par_axes_y_temp = par_axes_y_temp
                    else:
                        if par_axes_y_temp > par_axes_y:
                            par_axes_y = par_axes_y_temp
                        par_axes_y_temp = 0
                    conf_name += c[0][:-2].strip() + "_".join(c[0][-2:]).strip() + "_" + c[1].replace(".", "_")
                    acc_length += float(c[1])
                    if "0.7" in c[1]:
                        long_links += 1
                        longest = 0.7
                    if "pitch" in c[0]:
                        pitch += 1
                    if "pris" in c[0]:
                        pris += 1.0
                if par_axes_y_temp > par_axes_y:
                    par_axes_y = par_axes_y_temp
                if par_axes_y < 2:
                    par_axes_y = 0
                if acc_length <= 1.5:
                    acc_length = 1.5
                elif acc_length <= 2:
                    acc_length = 2
                elif acc_length <= 2.6:
                    acc_length = 2.6
                elif acc_length <= 3.1:
                    acc_length = 3.1
                else:
                    acc_length = 3.6
                conf_concept = {"dof": dof, 'par_axes_y': par_axes_y, '#long_link': long_links, 'long_link': longest,
                             'pitch_joint': pitch, 'p/r_ratio': round(pris/(dof-pris), 2), 'acc_length': acc_length}
                concepts_with_configuration[str(conf_concept)].append(conf_name)
        self.set_concepts_with_configuration(concepts_with_configuration)

    def set_possible_joints_order(self, file_name="/home/tamir/Tamir/Code/all_configs"):
        self.joints_combs = MyCsv.read_csv(file_name)

    def set_concepts(self, concepts):
        self.concepts = concepts

    def get_configurations(self):
        return self.configurations

    def get_concepts(self):
        return self.concepts

    def get_possible_joints_order(self):
        return self.joints_combs

    def get_possible_links_length(self):
        return self.links

    def set_concepts_with_configuration(self, concepts_with_configuration):
        self.concepts_with_configuration = concepts_with_configuration

    def get_concepts_with_configuration(self):
        return self.concepts_with_configuration


def split_files_to_several_folders(files_in_folder=5000):
    name = environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/5dof/to_run/"
    if not path.exists(name):
        mkdir(name)
    full_path = environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/5dof/combined/"
    files = listdir(full_path)
    for j in range(len(files)/files_in_folder):
        if not path.exists(name + str(j)):
            mkdir(name + str(j))
        for i in range(files_in_folder):
            # shutil.copy(full_path + files[j*files_in_folder+i], name + str(j) + files[j*files_in_folder+i])
            shutil.move(full_path + files[j*files_in_folder+i], name + str(j) + "/" + files[j*files_in_folder+i])


def sum_data():
    root = Tk()
    root.update()
    res_files = tkFileDialog.askopenfilenames(filetypes=(("csv files", "*.csv"), ("all files", "*.*")))
    root.destroy()
    new_file_name = "/".join(res_files[0].split("/")[:8]) + "/" + "/".join(res_files[0].split("/")[8:10])
    mu_penalty = -70
    time_penalty = 20
    z_penalty = 70
    data = []
    data_no_success = []
    in_list = []
    for res_file in res_files:
        try:
            csv_file = MyCsv.load_csv(res_file[:-4])
        except:
            csv_file = MyCsv.read_csv(res_file[:-4], "dict")
        for v in tqdm(csv_file):
            in_data = False
            if v["Z"] == -1 or v["Z"] == '70' and v["name"] not in in_list:
                v["mu"] = mu_penalty
                v["Z"] = z_penalty
                # v["time"] = time_penalty
                v["Passed"] = 0
                data_no_success.append(v)
                in_list.append((v["name"]))
                continue
            if v["name"] in in_list:
                for dat in data:
                    if v["name"] == dat["name"]:
                        dat_index = data[data.index(dat)]
                        if dat_index["Z"] > v["Z"]:
                            dat_index["Z"] = v["Z"]
                        if dat_index["mu"] < v["mu"]:
                            dat_index["mu"] = v["mu"]
                        # if dat_index["time"] > v["time"]:
                        #     dat_index["time"] = v["time"]
                        in_data = True
                        break
            elif not in_data:
                v["Passed"] = 1
                data.append(v)
                in_list.append((v["name"]))
    data_no_success = [dict(t) for t in {tuple(d.items()) for d in data_no_success}]
    MyCsv.save_csv(data, new_file_name, "dict")
    MyCsv.save_csv(data + data_no_success, new_file_name + "_with_failed", "dict")
    return data


def plot_data(result_file="/home/tamir/Tamir/Master/Code/sim_results/results_all"):
    all_data = MyCsv.read_csv(result_file, "dict")
    mu = []
    time = []
    z = []
    lci = []
    dof = []
    for dat in all_data:
        if dat["mu"] != "-1.0" and dat["mu"] != "mu":
            mu.append(float(dat["mu"]))
            time .append(float(dat["time"]))
            z.append(float(dat["Z"]))
            lci.append(float(dat["LCI"]))
            dof.append(float(dat["dof"]))
    plt.subplot(551)
    plt.title("Manipulability")
    plt.xlim(0, 1)
    plt.ylabel("Manipulability")
    plt.subplot(556)
    plt.scatter(mu, lci, color="g", s=4)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.ylabel("Local condion number")
    plt.subplot(5, 5, 11)
    plt.xlim(0, 1)
    plt.ylim(0, 7)
    plt.scatter(mu, time, color="black", s=4)
    plt.ylabel("Time (sec)")
    plt.subplot(5, 5, 16)
    plt.xlim(0, 1)
    plt.ylim(0, 0.51)
    plt.scatter(mu, z, color="cyan", s=4)
    plt.ylabel("Mid-joint state")
    plt.subplot(5, 5, 21)
    plt.xlim(0, 1)
    plt.scatter(mu, dof, color="magenta", s=4)
    plt.ylabel("Degree of Freedom")
    #
    plt.subplot(552)
    plt.title("Local condion number")
    plt.scatter(lci, mu, color="g", s=4)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    # plt.subplot(557)
    # plt.scatter([], [])
    plt.subplot(5, 5, 12)
    plt.scatter(lci, time, color="r", s=4)
    plt.xlim(0, 1)
    plt.ylim(0, 7)
    plt.subplot(5, 5, 17)
    plt.scatter(lci, z, color="brown", s=4)
    plt.xlim(0, 1)
    plt.ylim(0, 0.51)
    plt.subplot(5, 5, 22)
    plt.scatter(lci, dof, color="orange", s=4)
    plt.xlim(0, 1)

    plt.subplot(553)
    plt.title("Time (sec)")
    plt.scatter(time, mu, color="black", s=4)
    plt.xlim(0, 7)
    plt.ylim(0, 1)
    plt.subplot(558)
    plt.scatter(time, lci, color="r", s=4)
    plt.xlim(0, 7)
    plt.ylim(0, 1)
    # plt.subplot(5, 5,13)
    # plt.scatter([], [])
    plt.subplot(5, 5, 18)
    plt.scatter(time, z, color="pink", s=4)
    plt.xlim(0, 7)
    plt.ylim(0, 0.51)
    plt.subplot(5, 5, 23)
    plt.scatter(time, dof, s=4)
    plt.xlim(0, 7)

    plt.subplot(554)
    plt.title("Mid-joint state")
    plt.xlim(0, 0.51)
    plt.ylim(0, 1)
    plt.scatter(z, mu, color="cyan", s=4)
    plt.subplot(559)
    plt.xlim(0, 0.51)
    plt.ylim(0, 1)
    plt.scatter(z, lci, color="brown", s=4)
    plt.subplot(5, 5, 14)
    plt.scatter(z, time, color="pink", s=4)
    plt.xlim(0, 0.51)
    plt.ylim(0, 7)
    # plt.subplot(5,5,19)
    # plt.scatter([], [])
    plt.subplot(5, 5, 24)
    plt.scatter(z, dof, color="y", s=4)
    plt.xlim(0, 0.51)

    plt.subplot(555)
    plt.title("Degree of Freedom")
    plt.ylim(0, 1)
    plt.scatter(dof, mu, color="magenta", s=4)
    plt.subplot(5, 5, 10)
    plt.ylim(0, 1)
    plt.scatter(dof, lci, color="orange", s=4)
    plt.subplot(5, 5, 15)
    plt.ylim(0, 7)
    plt.scatter(dof, time, s=4)
    plt.subplot(5, 5, 20)
    plt.scatter(dof, z, color="y", s=4)
    plt.ylim(0, 0.51)
    plt.subplot(5, 5, 24)
    # plt.scatter([], [])
    plt.show()


# for optimization
def assign_results(res_name="results_all"):
    """ assign results from csv file """
    results = MyCsv.read_csv(res_name, "dict")
    x = []
    y = []
    z = []
    name = []
    for r in results:
        # add only points thats succeded to reach the point
        if r["Z"] != '70' and r["name"] != "name":
            name.append(r["name"])
            z.append(float(r["dof"]))
            y.append(1 - float(r["mu"]))
            x.append(float(r["Z"]))
    return [x, y, z, name]


def assign_conf2concept(conf):
    conf_name, z, mu, dof = conf
    concepts = load_json("jsons/concepts")
    dict_type = {"configuration": "", "concept": "",  "mu": 1, "z": 0.5, "dof": 7}
    res_data = []  # [[] for i in conf_name]
    for k in range(len(conf_name)):
        for concept in concepts:
            if conf_name[k] in concepts[concept]:
                dict_type["configuration"] = conf_name[k]
                dict_type["concept"] = concept
                dict_type["mu"] = mu[k]
                dict_type["z"] = z[k]
                dict_type["dof"] = dof[k]
                res_data.append(dict_type)
                dict_type = {"configuration": "", "concept": "", "mu": 1, "z": 0.5, "dof": 7}
                break
    return res_data


def domination_check(conf):
    """ check domination in 3D"""
    # front = [[conf[0][0]], [conf[1][0]], [conf[2][0]], [conf[3][0]]]
    front = [[1], [1], [6], [conf[3][0]]]
    points = [[], [], [], []]
    for i, j, k, l in zip(conf[0][:], conf[1][:], conf[2][:], conf[3][:]):  # z, mu, dof, configuration
        # if k == 4 or k == 6:
        #     continue
        added = False
        for i_front, j_front, k_front, l_front in zip(front[0], front[1], front[2], front[3]):  # zip(x_front, y_front, z_front, conf_front):
            # check if the point is dominate the front
            if i <= i_front and j <= j_front and k <= k_front:
                if not added:
                    front[0].append(i)
                    front[1].append(j)
                    front[2].append(k)
                    front[3].append(l)
                    added = True
                ind = front[3].index(l_front)
                del front[0][ind]
                del front[1][ind]
                del front[2][ind]
                del front[3][ind]
                if l_front not in points[0]:
                    points[0].append(l_front)
                    points[1].append(i_front)
                    points[2].append(j_front)
                    points[3].append(k_front)
            # check if the front dominate the point
            elif i > i_front and j > j_front and k >= k_front:
                if l not in points[0]:
                    points[0].append(l)
                    points[1].append(i)
                    points[2].append(j)
                    points[3].append(k)
                    added = True
                # if l in front[3]:
                #     ind = front[3].index(l_front)
                #     del front[0][ind]
                #     del front[1][ind]
                #     del front[2][ind]
                #     del front[3][ind]
        if not added:
            front[0].append(i)
            front[1].append(j)
            front[2].append(k)
            front[3].append(l)

    ind = np.ndarray.tolist(np.asarray(front[2]).argsort(axis=0))
    front = [map(front[3].__getitem__, ind), map(front[0].__getitem__, ind), map(front[1].__getitem__, ind),
             map(front[2].__getitem__, ind)]
    return points, front


def plot_pareto(other_points, pareto_concepts):
    """ plot 3D with pareto front and  all other points"""
    # plot settings
    plt.figure(figsize=(24.0, 10.0))
    ax = plt.axes(projection='3d')
    plt.subplots_adjust(0.0, 0.0, 1.0, 1.0, 0.2, 0.16)
    ax.view_init(azim=-145, elev=15)
    ax.set_ylim(0, 1)
    ax.set_xlim(0, 0.5)
    ax.set_zticks(np.arange(4, 7, 1.0))
    ax.set_zlabel("DOF")
    ax.set_ylabel("Munibulability")
    ax.set_xlabel("Mid Proximity Joint")
    # add to each point in the front its number
    pareto, ax = text2points(ax, pareto_concepts)
    # add Table of the front points
    add_table2plot(pareto)
    # add 3d points of all the points that not on the front
    ax.scatter3D(other_points[1], other_points[2], other_points[3], cmap='Greens', c="b", marker=".", alpha=0.15)
    # add 3d points of all the points that on the front with their number
    for x, y, z, label in zip(pareto[1], pareto[2], pareto[3], pareto[0]):
        ax.scatter3D(x, y, z, label=label, cmap='Greens', c="r", marker="o")
    # Make 3D surface of the front
    tri = Triangulation(pareto[1], pareto[3]).triangles
    ax.plot_trisurf(pareto[1], pareto[2], pareto[3], triangles=tri, shade=False, color=(1, 1, 0.4, 0.49), edgecolor='')
    # # Plot each dof points
    # plt.figure(4)
    # for x, y in zip(pareto[1][:5], pareto[2][:5]):
    #     plt.scatter(x, y, c="r", marker="o")
    #     plt.annotate(str(x) + " " + str(y), (x, y))
    # plt.show()
    # plt.figure(5)
    # for x, y in zip(pareto[1][5:10], pareto[2][5:10]):
    #     plt.scatter(x, y, c="r", marker="o")
    #     plt.annotate(str(x) + " " + str(y), (x, y))
    # plt.show()
    # plt.figure(6)
    # for x, y in zip(pareto[1][10:], pareto[2][10:]):
    #     plt.scatter(x, y, c="r", marker="o")
    #     plt.annotate(str(x) + " " + str(y), (x, y))
    # plt.show()


def text2points(ax, points):
    mu = []
    z = []
    dof = []
    for_legend = []
    # add text to each point
    for i in range(len(points)):
        point = points[i]
        mu.append(point["mu"])
        z.append(point["z"])
        dof.append(point["dof"])
        for_legend.append(str(i) + " " + str(point["concept"]).replace("{", "").replace("'", "").replace("}", ""))
        ax.text(point["z"], point["mu"], point["dof"], str(i))
    # pareto = [for_legend, z, mu, dof]
    return [for_legend, z, mu, dof], ax


def add_table2plot(pareto):
    data = []
    k = 0
    for p in pareto[0]:
        p = p + "mu:" + str(pareto_front[2][k]) + "Z:" + str(pareto_front[1][k])
        data.append([''.join(i for i in x if i.isdigit() or i == ".") for x in p.replace(" ", "").split(":")[1:]])
        k += 1
    data = np.ndarray.tolist(np.asarray(data).T)
    rows = ('# long links', 'longest link', 'DOF', 'Parallel axes about y', '# pitch joints', "P/R ratio", "Acc Length",
            "mu", "Z")
    columns = [str(x) for x in range(len(pareto_with_concepts))]
    # create text labels for the table
    cell_text = []
    for row in range(len(rows)):
        cell_text.append(data[row])
    colwidths = [0.03]*len(columns)
    # Add a table at the bottom of the axes
    plt.table(cellText=cell_text, rowLabels=rows, colWidths=colwidths, colLabels=columns, loc='bottom')
    plt.subplots_adjust(left=0.1, bottom=0.15)  # Adjust layout to make room for the table:
    plt.show()


#  ### Json handaling
def save_json(name="data_file", data=None, write_method="a"):
    with open(name + ".json", write_method) as write_file:
        json.dump(data, write_file, indent=2)
                                                             

def load_json(name="data_file"):
        try:
            with open(name + ".json", "r") as read_file:
                return json.load(read_file)
        except:
            fix_json(name)
            with open(name + ".json", "r") as read_file:
                return json.load(read_file)


def fix_json(file_name):
        with open(file_name + ".json", 'r') as filehandler:
            file_reader = filehandler.readlines()
            data = []
            empty = True
            for row in file_reader:
                if len(row) > 0:
                    if '][' in row:
                        row = ',\n'
                    data.append(row)
                    empty = False
                else:
                    if not empty:
                        data = []
                    empty = True
        with open(file_name + ".json", 'w') as name:
            name.writelines(data)


#  ### Pickle handling
def pickle_load_data(file_name="bin.pkl"):
    try:
        with open(file_name + ".pkl") as f:
            x = pickle.load(f)
    except:
        print("can't load the file")
        x = []
    return x


def pickle_save_data(data, file_name):
    with open(file_name + ".pkl", "wb") as f:
        pickle.dump(data, f)


#  ###how many configurations allready simulated and which to create  ###
def how_many_to_create(all_concepts, all_data, how_many):
    v = {}
    for k in range(len(all_concepts)):
        v[all_concepts.keys()[k]] = [len(all_concepts[all_concepts.keys()[k]]), 0, len(all_concepts[all_concepts.keys()[k]])]
    simulated = []
    concepts2check = []
    for dat in tqdm(all_data):
        if dat["name"] not in simulated and dat["dof"] == "6":
            for concept in v.keys():
                if 219 < v[concept][0] < how_many and dat["dof"] == concept[43:44]:
                    if dat["name"] in all_concepts[concept]:
                        v[concept][1] += 1
                        v[concept][2] = v[concept][0] - v[concept][1]
                        simulated.append(dat["name"])
                        if concept not in concepts2check:
                            concepts2check.append(concept)
                        break
    return concepts2check, simulated, v


def which_confs2create(concepts2check, all_concepts, simulated, dof2check="5"):
    print("Start which_confs2create")
    conf2create = []
    for conc in tqdm(concepts2check):
        if conc[43:44] == dof2check:
            for conf in all_concepts[conc]:
                if conf not in simulated:
                    conf2create.append([conf])
    total = len(conf2create)
    print("About " + str(total) + " configurations lefts. with Avg time of 15 seconds per configuration"
        " it will take about\n " + str(total * 15 / 3600. / 24) + " days to simulate all of them")
    return conf2create


# def how_many_configs_left(dof="5"):
#     """ Check how many configurations left in the specific dof - return all the concepts of this dof"""
#     all_concept = load_json("confs_number")
#     total = 0
#     total2 = 0
#     concepts = []
#     for i in tqdm(range(len(all_concept))):
#         if all_concept.keys()[i][43:44] == dof:
#             total += all_concept[all_concept.keys()[i]][2]
#             total2 += all_concept[all_concept.keys()[i]][1]
#             concepts.append(all_concept.keys()[i])
#     print("About " + str(total) + " configurations lefts. with Avg time of 15 seconds per configuration"
#          " it will take about\n " + str(total*15/3600./24) + " days to simulate all of them")
#     return concepts


# def remain_configs(all_concept, all_dat, dof="5"):
#     """check which configuration allready simulated and return list of all the configurations
#     that havent simulated yet"""
#     to_create = []
#     for a in all_concept:
#         if a[43:44] == dof:
#             to_create += all_concept[a]
#     for dat in tqdm(all_dat):
#         if dat["dof"] == dof:
#             if dat["name"] in to_create:
#                 to_create.remove(dat["name"])
#     to_cr = []
#     # fix this to the create urdf function
#     to_create = np.ndarray.tolist(np.unique(np.asarray(to_create)))
#     for t in to_create:
#         to_cr.append([t])
#     total = len(to_cr)
#     print("About " + str(total) + " configurations lefts. with Avg time of 15 seconds per configuration"
#           " it will take about\n " + str(total * 15 / 3600. / 24) + " days to simulate all of them")
#     return to_cr


def create_configs(all_concept, all_dat, confs_in_concept=220000):
    # if we want to calculated with different data uncomment
      # all the concecpts with less than 220 configurations
    concepts2check, simulated, v = how_many_to_create(all_concept, all_dat, confs_in_concept)
    save_json("jsons/other/confs_number", v, "w+")
    save_json("jsons/other/concepts2check", concepts2check, "w+")
    # save_json("jsons/other/simulated", simulated, "w+")
    return concepts2check, simulated, v


def update_results(to_add_file="to_add", current_file="jsons/concepts+configs+results"):
    concepts_configs_results = load_json(current_file)
    to_add = MyCsv.read_csv(to_add_file, "dict")

    for add in tqdm(to_add):
        for res in concepts_configs_results:
            if res[43:44] == add["dof"]:
                for conf in concepts_configs_results[res]:
                    conf_name = conf.keys()[0]
                    if add["name"] == conf_name:
                        # concepts_configs_results[res][concepts_configs_results[res].index(conf)] = add
                        ind = [concepts_configs_results[res].index(conf)][0]
                        concepts_configs_results[res][ind][conf_name]["mu"] = unicode(add["mu"])
                        concepts_configs_results[res][ind][conf_name]["z"] = unicode(add["Z"])
                        break
    save_json(current_file, concepts_configs_results, "w+")


def remain_to_sim(all_concept, dof2check="5"):
    simulated = load_json("jsons/other/simulated")
    concepts2check = load_json("jsons/other/concepts2check")
    create_urdf = which_confs2create(concepts2check, all_concept, simulated, dof2check=dof2check)
    con = Concepts()
    con.create_files2sim(create_urdf, "5dof", "6dof")
    return create_urdf


def remain_conf_in_conc(all_concept):
    print("Start remain_conf_in_conc")
    ga_concept = {}
    for concept in all_concept:
        if len(all_concepts[concept]) > 220 and concept[43:44] == "6":
            ga_concept[concept] = all_concept[concept]
    save_json("jsons/concepts2ga", ga_concept)
    return ga_concept


def left_confs_concepts():
    # create CSV file with how many configs simulated and left at each concept
    print("Start left_confs_concepts")
    a = load_json("jsons/other/confs_number")
    b = list(a)
    c = [[]] * len(b)
    for i in range(len(b)):
        c[i] = [b[i], a[b[i]][0], a[b[i]][1], a[b[i]][2]]
    MyCsv.save_csv(c, "left_concepts")


def combine_res(all_data, all_concepts):
    # create one file of configurations with there results via concepts
    print("Start combine_res")
    new_data = {}
    all_data_ind = []
    for i in all_data:
        all_data_ind.append(i["name"])
    all_data_ind = np.asarray(all_data_ind)
    for dat in tqdm(all_concepts):
        flag = []
        mask = np.isin(all_data_ind, np.array(all_concepts[dat]))
        inds = np.argwhere(mask > 0)
        k = 0
        for ind in inds:
            try:
                dict_keys = {"name": all_data[ind[0]]["name"], "mu": all_data[ind[0]]["mu"],
                             "dof": all_data[ind[0]]["dof"], "z": all_data[ind[0]]["Z"]}
            except:
                dict_keys = {"name": all_concepts[dat][k], "mu": None, "dof": None, "z": None}
            flag.append({all_concepts[dat][k]: dict_keys})
            k += 1
        new_data[dat] = flag

    save_json("concepts+configs+results", new_data)
    return new_data
# ###   ###


if __name__ == '__main__':
    # while True:
    split = False
    calc_concepts = False
    create_urdf = False
    fix_all_from_json = False
    sumdata = False
    to_merge = False
    plotdata = False
    fix_from_json = False
    pareto_plot = False
    check_num_confs_in_concepts = True
    if calc_concepts:
        con = Concepts()
        concepts_with_values = con.calc()
        # create random urdf
        if create_urdf:
            all_to_sim = con.confs2simulate(concepts_with_values)
            filter2sim, res = con.filter_confs(all_to_sim)
            con.create_files2sim(filter2sim)
            MyCsv.save_csv(res, "tests/results", "dict")
    if split:
        split_files_to_several_folders(5000)
    if to_merge:
        merge_data = MergeData()
        merge_data.merge()
    if fix_all_from_json:
        FixFromJson(all_files=True)
    if sumdata:
        summed_data = sum_data()
    if fix_from_json:
        FixFromJson()
    if plotdata:
        plot_data(result_file="/home/tamir/Tamir/Master/Code/sim_results/results_all")
    if pareto_plot:
        con = assign_results()
        outer_points, pareto_front = domination_check(con)
        pareto_with_concepts = assign_conf2concept(pareto_front)
        plot_pareto(outer_points, pareto_with_concepts)
    if check_num_confs_in_concepts:
        create_new_file = True
        all_data = MyCsv.read_csv("results_all", "dict")  # all the results
        all_concepts = load_json("jsons/concepts")  # all the concepts and there configurations
        confs_in_concepts = 500000  # all the concecpts with less than 220 configurations
        if create_new_file:
            create_configs(all_concepts, all_data, confs_in_concept=confs_in_concepts)
        # else:
        #     update_results("to_add")
        # Create json file of the remaining concepts and their configurations
        ga_concepts = remain_conf_in_conc(all_concepts)
        save_json("results_all", all_data)
        # create CSV file with how many configs simulated and left at each concept
        left_confs_concepts()
        # create one file of configurations with there results via concepts
        combine_data = combine_res(all_data, all_concepts)
        # # create the urdf's for the remaining configurations in the selected dof
        # to_create = remain_to_sim(all_concepts, dof2check="6")

# todo - check how many that have been simulated more than once one mu bigger and one z bigger!!!!
# todo - update concepts+configs+results instead of recalculate + need to change jsons files
# todo - show the configurations that on the woi
# todo - to see which configuration are in several concepts
# todo - concept: "{'#long_link'	long_link	dof	par_axes_y	pitch_joint	p/r_ratio	acc_length
#  0	0.4	6	0	1	0.5	 1.5}"