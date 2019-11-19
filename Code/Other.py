from os import environ, listdir, mkdir, path
import shutil
import csv
import tkFileDialog
from Tkinter import *
import json
import matplotlib.pyplot as plt
import numpy as np


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
            return result

    @staticmethod
    def save_csv(data, file_name, csv_type="list"):
        """Save to csv format"""
        with open(file_name + ".csv", 'ab') as name:
            if csv_type == "list":
                writer = csv.writer(name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                writer.writerows(data)
            elif csv_type == "dict":  # for dictionary
                headers = ["name", "joints", "link_length", "prev_axe", "dof", "mu", "time", "Z", "LCI"]
                writer = csv.DictWriter(name, fieldnames=headers, delimiter=',')
                writer.writeheader()
                writer.writerows(data)
                # for dat in data:
                #     str_dat = []
                #     # for d in dat:
                #     #     str_dat.append(str(d))
                #     writer.writerow(str_dat)

    @staticmethod
    def read_csv(file_name, csv_type="list"):
        with open(file_name + ".csv", 'r') as _filehandler:
            data = []
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
                            data = []
                        empty = True
            elif csv_type == "dict":
                csv_file_reader = csv.DictReader(_filehandler)
                for row in csv_file_reader:
                    data.append(row)
        return data


class MergeData(object):

    def __init__(self):
        root = Tk()
        root.update()
        self.files = tkFileDialog.askopenfilenames(filetypes=(("csv files", "*.csv"), ("all files", "*.*")))
        root.destroy()
        self.new_file_name = "results" + self.files[0][-13:-5]

    def merge(self):
        files = self.files
        new_file_name = self.new_file_name
        for i in range(len(files)):
            MyCsv.save_csv(MyCsv.read_csv(files[i][:-4]), new_file_name)
            try:
                self.fix_json(files[i][:-4])
                self.save_json(data=self.load_json(files[i][:-4] + "_fixed"), name=new_file_name)
            except ValueError:
                print("There are no Json files")

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
        j_ev = np.linalg.svd(jacobian)[1]  # eighen values
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
        for joint in joints:
            if "pris" not in joint:
                theta_mean.append(0)
            else:
                theta_mean.append(float(link_length[joints.index(joint)])/2)
        # print(name)
        w = np.identity(len(joints)+1)*(cur_pos[:-1]-theta_mean)  # weighted diagonal matrix
        z = np.around(0.5*np.transpose(cur_pos[:-1]-theta_mean)*w, 3)
        return z

    @staticmethod
    def local_conditioning_index(jacobian):
        return round(1/(np.linalg.norm(jacobian)*np.linalg.norm(np.linalg.pinv(jacobian))), 3)


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
    data = []
    for res_file in res_files:
        csv_file = MyCsv.load_csv(res_file[:-4])
        if len(data) == 0:
            first = MyCsv.load_csv(res_file[:-4])
            for fir in first:
                if fir["Z"] == -1:
                    continue
                data.append(fir)
        for v in csv_file:
            in_data = False
            if v["Z"] == -1:
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
                data.append(v)
    MyCsv.save_csv(data, new_file_name, "dict")
    return data


def plot_data(result_file="/home/tamir/Tamir/Master/Code/results/recalculate/results_all_success"):
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
    # plt.scatter([], [])
    plt.ylabel("Manipulability")
    plt.subplot(556)
    plt.scatter(mu, lci, color="g", s=4)
    plt.ylabel("Local condion number")
    plt.subplot(5, 5, 11)
    plt.scatter(mu, time, color="black", s=4)
    plt.ylabel("Time (sec)")
    plt.subplot(5, 5, 16)
    plt.scatter(mu, z, color="cyan", s=4)
    plt.ylabel("Mid-joint state")
    plt.subplot(5, 5, 21)
    plt.scatter(mu, dof, color="magenta", s=4)
    plt.ylabel("Degree of Freedom")
    #
    plt.subplot(552)
    plt.title("Local condion number")
    plt.scatter(lci, mu, color="g", s=4)
    # plt.subplot(557)
    # plt.scatter([], [])
    plt.subplot(5, 5, 12)
    plt.scatter(lci, time, color="r", s=4)
    plt.subplot(5, 5, 17)
    plt.scatter(lci, z, color="brown", s=4)
    plt.subplot(5, 5, 22)
    plt.scatter(lci, dof, color="orange", s=4)

    plt.subplot(553)
    plt.title("Time (sec)")
    plt.scatter(time, mu, color="black", s=4)
    plt.subplot(558)
    plt.scatter(time, lci, color="r", s=4)
    # plt.subplot(5, 5,13)
    # plt.scatter([], [])
    plt.subplot(5, 5, 18)
    plt.scatter(time, z, color="pink", s=4)
    plt.subplot(5, 5, 23)
    plt.scatter(time, dof, s=4)

    plt.subplot(554)
    plt.title("Mid-joint state")
    plt.scatter(z, mu, color="cyan", s=4)
    plt.subplot(559)
    plt.scatter(z, lci, color="brown", s=4)
    plt.subplot(5, 5, 14)
    plt.scatter(z, time, color="pink", s=4)
    # plt.subplot(5,5,19)
    # plt.scatter([], [])
    plt.subplot(5, 5, 24)
    plt.scatter(z, dof, color="y", s=4)

    plt.subplot(555)
    plt.title("Degree of Freedom")
    plt.scatter(dof, mu, color="magenta", s=4)
    plt.subplot(5, 5, 10)
    plt.scatter(dof, lci, color="orange", s=4)
    plt.subplot(5, 5, 15)
    plt.scatter(dof, time, s=4)
    plt.subplot(5, 5, 20)
    plt.scatter(dof, z, color="y", s=4)
    plt.subplot(5, 5, 24)
    # plt.scatter([], [])
    plt.show()


if __name__ == '__main__':
    split = False
    sumdata = False
    to_merge = False
    plotdata = False
    fix_from_json = False
    fix_all_from_json = False
    if to_merge:
        merge_data = MergeData()
        merge_data.merge()
    if split:
        split_files_to_several_folders(5000)
    if sumdata:
        summed_data = sum_data()
    if plotdata:
        plot_data(result_file="/home/tamir/Tamir/Master/Code/results/recalculate/results_all")
    if fix_from_json:
        FixFromJson()
    if fix_all_from_json:
        FixFromJson(all_files=True)
