import csv


class HandleCSV(object):

    @staticmethod
    def save_data(data, file_name):
        """Save to csv format"""
        with open(file_name + ".csv", 'w', newline='') as name:
            writer = csv.writer(name, delimiter=',')
            i = 0
            for row in data:
                # writer.writerow(list(map(list, zip(*row))))
                writer.writerow(row)
                i += 1
                if i % 6 == 0:
                    writer.writerow([])
                    writer.writerow([])
                    writer.writerow([])
                    if not i == 6:
                        writer.writerow([])

    def read_data(self, file_name):
        with open(file_name + ".csv", 'r') as _filehandler:
            csv_file_reader = csv.reader(_filehandler)
            data = []
            manip = []
            empty = True
            for row in csv_file_reader:
                while "" in row:
                    row.remove("")
                if len(row) > 0:
                    data.append(row)
                    empty = False
                else:
                    if not empty:
                        manip.append(self.read_data_action(data))
                        data = []
                    empty = True
            manip.append(self.read_data_action(data))  # append the last session
            return manip

    @staticmethod
    def read_data_action(data):
        manip = list(map(list, zip(*data[0:])))
        manip_array_of_dict = []
        for i in range(0, len(manip) - 1, 2):
            manip_array_of_dict.append({"joint": manip[i], "axe": manip[i + 1]})
        return manip_array_of_dict


def filter_configs(config):
    # for i in range(len(config["joint"])):
    a = [config.split("+")[i].split(",") for i in range(6)]
    joint = [a[i][0] for i in range(6)]
    axe = [a[i][1] for i in range(6)]
    dof = len(joint)-1
    if joint.count("pris") > 3:  # no more than 3 prismatics joints
        return False
    if (joint[dof-1] == "pris" and axe[dof-1] == "z" and joint[dof-2] == "roll") or\
         (joint[dof-2] == "pris" and joint[dof-1] == "roll" and axe[dof-1] == "z"):
        # after pris -> roll z \ roll -> pris z can be only: pris y / pitch y / roll y
        if joint[dof] == "roll" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pitch" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pris" and (axe[dof] == "x" or axe[dof] == "z"):
            return False

    if joint[dof-1] == "roll":  # after roll only pris z/y, pitch y, roll y
        if joint[dof] == "roll" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pitch" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pris" and axe[dof] == "x":
            return False

    if joint[dof-1] == "pris" and joint[dof] == "pris" and axe[dof] == "z":
        # after prismatic joint can not be prismatic in z axe
        return False

    if joint[dof - 2] == "pris" and joint[dof - 1] == "pris" and joint[dof] == "pris" and axe[dof] == "y":
        return False

    else:
        return True


csv_data = HandleCSV()
data_from_csv = csv_data.read_data("5dof_configs")
data = []
to_add = [["roll", "pitch", "pris", "pris", "pitch", "roll", "pris", "roll", "pitch"],
          ["y", "y", "z", "y", "z", "x", "x", "z", "x"]]
for i in range(len(data_from_csv)):
    dat = []
    for j in range(len(data_from_csv[i])):
        dat.append(data_from_csv[i][j])
        all_data = []
        for k in range(len(to_add[0])):
            conf = ""
            for z in range(len(dat[j]["joint"])):
                conf = conf + dat[j]["joint"][z] + "," + dat[j]["axe"][z] + "+"
            conf = conf + to_add[0][k] + "," + to_add[1][k] + "+"
            if filter_configs(conf):
                all_data.append(conf)
        data.append(all_data)


word_to_save = ""
save = []


for row in data:
    for k in range(len(row)):
        for word in row[k]:
            if word != "+":
                word_to_save = word_to_save + word
            else:
                word_to_save = word_to_save  # + "  "
                save.append(word_to_save)
                word_to_save = ""

n = 5760
cf = ([list(i) for i in zip(*[save[i:i + n] for i in range(0, len(save), n)])])
csv_data.save_data(cf, "6dof_configs_code")

# for i in range(len(save)+1):
#     if i%6 ==0:
#         a.append(i)
#         ccc.append(save[i])

# def recursive_len(item):
#     if type(item) == list:
#         return sum(recursive_len(subitem) for subitem in item)
#     else:
#         return 1

