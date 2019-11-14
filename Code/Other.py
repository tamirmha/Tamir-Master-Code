from os import environ, listdir, mkdir, path
import shutil
import csv


def split_files_to_several_folders(files_in_folder=5000):
    name = environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/5dof/to_run/"
    if not path.exists(name):
        mkdir(name)

    full_path = environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/5dof/combined/"
    files = listdir(full_path)

    for j in range(len(files)/files_in_folder - 1):
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
                     "Z": float(row[9]), "ri": float(row[10]), "dof": dof, "vars": [joints, prev_axe, link_length]})
        return result


# name_file = "results_file12_11_6dof_4d_"
# v = load_csv(name_file)


def get_urdf(name="roll_z_0_1pris_y_0_7pris_y_0_7pitch_x_0_7pitch_x_0_7pris_x_0_7"):

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
    return [joints, prev_axe, link_length]


v = get_urdf()
