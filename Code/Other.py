from os import environ, listdir, mkdir, path
import shutil
import csv
import tkFileDialog
from Tkinter import *
import json
import matplotlib.pyplot as plt
import numpy as np
from itertools import product


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
                        "LCI": float(row[9]), "dof": dof, "joints": "_".join(joints), "prev_axe": "_".join(prev_axe),
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
            # try:
            #     self.fix_json(files[i][:-4])
            #     self.save_json(data=self.load_json(files[i][:-4] + "_fixed"), name=new_file_name)
            # except ValueError:
            #     print("There are no Json files")

    @staticmethod
    def save_json(name="data_file", data=None):
        with open(name + ".json", "a") as write_file:
            json.dump(data, write_file, indent=2)

    @staticmethod
    def load_json(name="data_file"):
        with open(name + ".json", "r") as read_file:
            return json.load(read_file)

    @staticmethod
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
        with open(file_name + "_fixed.json", 'w') as name:
            name.writelines(data)


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
        MergeData.fix_json(file2fix)
        original_data = MergeData.load_json(file_fixed)
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
                lci = 1.1
                for jacobian, curr_pos in zip(jacobians, curr_poss):
                    if jacobian != -1:
                        mu_new, lci_new, z_new = self.indices_calc(row[2], jacobian, curr_pos)
                        if mu > mu_new:
                            mu = mu_new
                        if z < z_new:
                            z = z_new
                        if lci > lci_new:
                            lci = lci_new
                to_fix[to_fix.index(row)][7] = mu
                to_fix[to_fix.index(row)][8] = z
                to_fix[to_fix.index(row)][9] = lci
        return to_fix

    def indices_calc(self, names, jacobian, cur_pos):
        cur_pos = np.asarray(cur_pos)
        jacobian = np.asarray(jacobian)
        j_ev = np.linalg.svd(jacobian, compute_uv=False)  # [1]  # singular (eighen) values
        # Manipulability index
        mu = round(np.product(j_ev), 3)  # self.manipulability_index(jacobian)
        # Local Conditioning Index
        lci = round(j_ev[-1] / j_ev[0], 3)  # self.local_conditioning_index(jacobian)
        # Joint Mid-Range Proximity
        z = self.mid_joint_proximity(cur_pos, names)
        return mu, lci, np.diag(z).max()

    @staticmethod
    def manipulability_index(jacobian):
        n = jacobian.size / len(jacobian)
        if n == 5:
            det_j = np.linalg.det(np.matmul(np.transpose(jacobian), jacobian))
        else:
            det_j = np.linalg.det(np.matmul(jacobian, np.transpose(jacobian)))
        if det_j > 1e-18:  # preventing numeric problems
            return round(det_j ** (1/float(n)), 3)
            # return round(det_j ** 0.5, 3)
        else:
            return 0

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
        # print(name)
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

    @staticmethod
    def local_conditioning_index(jacobian):
        return round(1/(np.linalg.norm(jacobian)*np.linalg.norm(np.linalg.pinv(jacobian))), 3)


class Concepts:
    def __init__(self, file_name="/home/tamir/Tamir/Master/optimization/all_configs"):
        self.concepts = []
        self.joints_combs = []
        self.links = []
        self.configurations = []
        self.set_possible_links_length(0.1, 0.71, 0.3, 1)
        self.set_possible_joints_order(file_name)
        self.concepts_with_configuration = []

    def calc(self):
        self.determine_combinations()
        self.set_configurations()
        self.assign_configuration2concept()
        data = self.get_concepts_with_configuration()
        concepts_with_values = [[k, v] for k, v in data.items() if v != []]
        concepts_without_values = [[k, 0] for k, v in data.items() if v == []]
        combs_in_concept = []
        for c in concepts_with_values:
            combs_in_concept.append([c[0], len(c[1])])
        # MyCsv.save_csv(concepts_without_values, "concepts_without_values")
        MyCsv.save_csv(combs_in_concept, "concepts_sum")
        MergeData.save_json("concepts", data)

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
                # if c[0]*0.7 > c[4] or (c[5] == 0.4 and c[5] * (k+3) < c[4] - 0.6):
                #     continue
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
                conf_name = ""
                dof = len(cf)
                pitch = 0
                long_links = 0
                pris = 0
                acc_length = 0
                longest = 0.4
                for c in cf:
                    if "pitch" in "".join(c[0]) and "y" in "".join(c[0]):
                        par_axes_y += 1
                    elif "pris" in "".join(c[0]) and "z" in "".join(c[0]):
                        par_axes_y = par_axes_y
                    else:
                        par_axes_y = 0
                    conf_name += c[0][1:-2]+"_"+c[0][-2] + "_" + c[1].replace(".", "_")
                    acc_length += float(c[1])
                    if "0.7" in c[1]:
                        long_links += 1
                        longest = 0.7
                    if "pitch" in c[0]:
                        pitch += 1
                    if "pris" in c[0]:
                        pris += 1.0
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

    def set_possible_joints_order(self, file_name="/home/tamir/Tamir/Master/optimization/all_configs"):
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

    # @staticmethod
    # def calc_parallel_axes(configuration):
    #     around_y = 0
    #     for conf in configuration[1:]:
    #         if "pitch" in "".join(conf[0]) and "y" in "".join(conf[0]):
    #             around_y += 1
    #         elif "pris" in "".join(conf[0]) and "z" in "".join(conf[0]):
    #             continue
    #         else:
    #             around_y = 0
    #     if around_y <2:
    #         around_y = 0
    #     return around_y
    #
    # @staticmethod
    # def roataion_matrix(theta, axe):
    #     c = round(np.cos(theta), 3)
    #     s = round(np.sin(theta), 3)
    #     if axe == "x":
    #         r = np.mat([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
    #     elif axe == "y":
    #         r = np.mat([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
    #     elif axe == "z":
    #         r = np.mat([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    #     return r


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
    new_file_name = "/".join(res_files[0].split("/")[:8]) + "/" + res_files[0].split("/")[8] + "_all"
    mu_penalty = 0
    time_penalty = 20
    z_penalty = 70
    data = []
    data_no_success = []
    for res_file in res_files:
        csv_file = MyCsv.load_csv(res_file[:-4])
        # if len(data) == 0:
        #     first = MyCsv.load_csv(res_file[:-4])
        #     for fir in first:
        #         if fir["Z"] == -1:
        #             data_no_success.append(fir)
        #             continue
        #         data.append(fir)
        #     continue
        for v in csv_file:
            in_data = False
            if v["Z"] == -1:
                if len(data_no_success) == 0:
                    v["mu"] = mu_penalty
                    v["LCI"] = mu_penalty
                    v["Z"] = z_penalty
                    v["time"] = time_penalty
                    v["Passed"] = 0
                    data_no_success.append(v)
                    continue
                else:
                    v["mu"] = mu_penalty
                    v["LCI"] = mu_penalty
                    v["Z"] = z_penalty
                    v["time"] = time_penalty
                    v["Passed"] = 0
                    data_no_success.append(v)
                    # break
                continue
            for dat in data:
                if v["name"] in dat["name"]:
                    dat_index = data[data.index(dat)]
                    if dat_index["Z"] > v["Z"]:
                        dat_index["Z"] = v["Z"]
                    if dat_index["mu"] < v["mu"]:
                        dat_index["mu"] = v["mu"]
                    if dat_index["time"] > v["time"]:
                        dat_index["time"] = v["time"]
                    if dat_index["LCI"] < v["LCI"]:
                        dat_index["LCI"] = v["LCI"]
                    in_data = True
                    break
            if not in_data:
                v["Passed"] = 1
                data.append(v)
    data_no_success = [dict(t) for t in {tuple(d.items()) for d in data_no_success}]
    MyCsv.save_csv(data, new_file_name, "dict")
    all_data = data + data_no_success
    MyCsv.save_csv(all_data, new_file_name + "_with_failed", "dict")
    return data


def plot_data(result_file="/home/tamir/Tamir/Master/Code/results/recalculate/results_all"):
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


if __name__ == '__main__':
    split = False
    calc_concepts = True
    fix_all_from_json = False
    sumdata = False
    to_merge = False
    plotdata = False
    fix_from_json = False
    if calc_concepts:
        con = Concepts()
        con.calc()
    if split:
        split_files_to_several_folders(5000)
    if to_merge:
        merge_data = MergeData()
        merge_data.merge()
    if sumdata:
        summed_data = sum_data()
    if fix_from_json:
        FixFromJson()
    if fix_all_from_json:
        FixFromJson(all_files=True)
    if plotdata:
        plot_data(result_file="/home/tamir/Tamir/Master/Code/results/recalculate/results_all")
