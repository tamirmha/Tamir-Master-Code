from os import environ, listdir, mkdir, path
import shutil


files_in_folder = 10000
name = environ['HOME'] + "/urdf_5dof/"
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
