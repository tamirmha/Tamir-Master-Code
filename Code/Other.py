from os import environ, listdir, mkdir, path
import shutil
import csv
import tkFileDialog
from Tkinter import *
import json


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


def load_csv(file_name):
    with open(file_name + ".csv", "r") as data_file:
        csv_file_reader = csv.reader(data_file)
        result = []
        dof = int(data_file.name[data_file.name.find("dof")-1])
        for row in csv_file_reader:
            while "" in row:
                row.remove("")
            if len(row) > 0:
                if len(row) == 1:
                    row = row[0].split(",")
                name = row[2]
                joints = ["roll"]
                prev_axe = ["z"]
                link_length = [0.1]
                arm = name.split("_")
                for a in range(3, len(arm)-1):
                    if a % 3 == 0:
                        joints.append(arm[a][1:])
                    elif a % 3 == 1:
                        prev_axe.append(arm[a])
                    elif a % 3 == 2:
                        link_length.append(float(arm[a] + "." + arm[a+1][:1]))
                result.append({"name": row[2], "time": float(row[6]), "mu": float(row[7]), "LCI": float(row[8]),
                    "Z": float(row[9]), "dof": dof, "vars": [joints, prev_axe, link_length]})
        return result


def sum_data():
    root = Tk()
    root.update()
    res_files = tkFileDialog.askopenfilenames(filetypes=(("csv files", "*.csv"), ("all files", "*.*")))
    data = []

    for res_file in res_files:
        V = load_csv(res_file[:-4])
        if len(data) == 0:
            first = load_csv(res_file[:-4])
            for fir in first:
                if fir["Z"] == -1:
                    fir = {"name": fir["name"], "time": 10.0, "mu": -1.0, "LCI": -1.0,
                           "Z": 15, "dof": fir["dof"], "vars": fir["vars"]}
                data.append(fir)
        for v in V:
            in_data = False
            if v["Z"] == -1:
                v = {"name": v["name"], "time": 10.0, "mu": -1.0, "LCI": -1.0, "Z": 15, "dof": v["dof"], "vars": v["vars"]}
            for dat in data:
                if v["name"] in dat["name"]:
                    dat_index = data[data.index(dat)]
                    dat_index["Z"] = (dat_index["Z"] + v["Z"]) / 2.0
                    dat_index["mu"] = (dat_index["mu"] + v["mu"]) / 2.0
                    dat_index["time"] = (dat_index["time"] + v["time"]) / 2.0
                    dat_index["LCI"] = (dat_index["LCI"] + v["LCI"]) / 2.0
                    in_data = True
                    break
            if not in_data:
                data.append(v)
    root.destroy()


class MergeData(object):

    def __init__(self):
        self.root = Tk()
        self.root.update()
        self.files = tkFileDialog.askopenfilenames(filetypes=(("csv files", "*.csv"), ("all files", "*.*")))
        self.new_file_name = "results" + self.files[0][-13:-5]

    def merge(self):
        files = self.files
        new_file_name = self.new_file_name
        for i in range(len(files)):
            self.save_csv(self.read_csv(files[i][:-4]), new_file_name)
            self.fix_json(files[i][:-4])
            self.save_json(data=self.load_json(files[i][:-4] + "_fixed"), name=new_file_name)
        self.root.destroy()

    @staticmethod
    def save_csv(data, file_name):
        """Save to csv format"""
        with open(file_name + ".csv", 'ab') as name:
            writer = csv.writer(name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerows(data)

    @staticmethod
    def read_csv(file_name):
        with open(file_name + ".csv", 'r') as _filehandler:
            csv_file_reader = csv.reader(_filehandler)
            data = []
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
            return data

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


split = False
sumdata = False
to_merge = False
if to_merge:
    merge_data = MergeData()
    merge_data.merge()
if split:
    split_files_to_several_folders(5000)
if sumdata:
    sum_data()
