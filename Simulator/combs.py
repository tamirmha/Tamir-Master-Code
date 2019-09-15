import csv
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import gridspec
import json


class HandleCSV(object):

    @staticmethod
    def save_data(data, file_name, dof=6):
        """Save to csv format"""
        with open(file_name + ".csv", 'w', newline='') as name:
            writer = csv.writer(name, delimiter=',', escapechar=' ', quoting=csv.QUOTE_NONE)
            i = 0
            for row in data:
                # writer.writerow(list(map(list, zip(*row))))
                writer.writerow(row)
                i += 1
                if i % dof == 0:
                    writer.writerow([])
                    writer.writerow([])
                    writer.writerow([])
                    if not i == dof:
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
                    data.append([row[0].strip(), row[1]])
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


class Tree:
    """"
    note: this class can be used only for 6 DOF, otherwise changes must be done
    this class create tree of all the possible joints combinations
    """
    def __init__(self, y_offset=4, x_offset=0, findegree=-90, pos_high=80, pos_low=50):
        self.pos_high = pos_high
        self.pos_low = pos_low
        self.y_offset = y_offset
        self.x_offset = x_offset
        self.findegree = findegree
        self.dofs = []
        self.G = nx.Graph()  # 4&5 DOF tree
        self.G1 = nx.Graph()  # 5&6 DOF tree - part1
        self.G2 = nx.Graph()  # 5&6 DOF tree - part2
        self.G3 = nx.Graph()  # 5&6 DOF tree - part3

    def tree_create(self, n_from=0, n_to=2, all_trees=True, plot=False, save=True):
        dofs = self.dofs
        if all_trees:
            n_from = 0
            n_to = len(dofs)
        for i in range(n_from, n_to):
            self.G = nx.Graph()  # 4&5 DOF tree
            self.G1 = nx.Graph()  # 5&6 DOF tree - part1
            self.G2 = nx.Graph()  # 5&6 DOF tree - part2
            self.G3 = nx.Graph()  # 5&6 DOF tree - part3
            conf4number = len(dofs[i][1])
            conf4_width = 4000
            x4_pos = [(conf4_width * (k - conf4number / 2)) for k in range(conf4number)]
            t = 0
            for j in range(conf4number):
                self.G.add_node(dofs[i][1][j], pos=(x4_pos[j], 80))
                conf5number = len(dofs[i][2][j])
                conf5_width = conf4_width // conf5number
                x5_pos = [(x4_pos[j] + conf5_width * (k - conf5number / 2)) for k in range(conf5number)]
                for z in range(conf5number):
                    self.G.add_node(dofs[i][2][j][z], pos=(x5_pos[z], 50))
                    if j < conf4number//3:
                        self.nodes_arrange(self.G1, self.G, x5_pos, conf5_width, i, j, z, t)
                    elif j > 2*conf4number//3-1:
                        self.nodes_arrange(self.G3, self.G, x5_pos, conf5_width, i, j, z, t)
                    else:
                        self.nodes_arrange(self.G2, self.G, x5_pos, conf5_width, i, j, z, t)
                    t += 1
            self.tree_plot(i, save)
        if plot:
            plt.show()

    def arrange_configs2tree(self, configs):
        dof = []
        dof3 = []
        dof4 = []
        dof5 = []
        indices = [[], []]
        dof_flags = []
        i = 0

        configs = sorted(configs)
        for config in configs:
            dof_flag = ".".join(config[0].split("+")[:3])
            if dof_flag not in dof3:
                dof3.append(dof_flag)
                indices[0].append(i)
            dof_flag = ".".join(config[0].split("+")[:4])
            if dof_flag not in dof4:
                new4conf = True
                dof4.append(dof_flag)
                indices[1].append(i)
            if new4conf:
                dof5.append(dof_flags)
                dof_flags = []
                new4conf = False
            dof_flag = ".".join(config[0].split("+")[:5])
            dof_flags.append(dof_flag)
            i = i + 1

        dof5.append(dof_flags)
        dof5.remove(dof5[0])
        # for d in range(len(dof5)):
        #     for do in dof5[d]:
        #         do.count(dof5[d])
        for i in range(len(indices[0]) - 1):
            dof4_indices = [j for j in range(indices[1].index(indices[0][i]), indices[1].index(indices[0][i + 1]))]
            dof.append([[dof3[i]], dof4[dof4_indices[0]:dof4_indices[-1] + 1], dof5[dof4_indices[0]:dof4_indices[-1] + 1],
                        configs[indices[0][i]:indices[0][i + 1]][:]])

        dof4_indices = [j for j in range(indices[1].index(indices[0][i + 1]), indices[1].index(indices[1][-1]) + 1)]
        dof.append([[dof3[i]], dof4[dof4_indices[0]:dof4_indices[-1] + 1], dof5[dof4_indices[0]:dof4_indices[-1] + 1],
                    configs[indices[0][i + 1]:]])

        for i in range(len(dof)):
            for j in range(len(dof[i][3])):
                for k in range(len(dof[i][3][j])):
                    dof[i][3][j][k] = ".".join(dof[i][3][j][k].split("+"))[:-1]
        self.dofs = dof
        return dof

    def tree_visuality(self, g):
        nodelabeldict = {}
        for n in g.nodes():
            nodelabeldict[n] = n.split(".")[-1]
        pos = nx.get_node_attributes(g, 'pos')
        pos_node = nx.get_node_attributes(g, 'pos')
        for p in pos:
            if pos[p][1] == self.pos_high:
                pos[p] = (pos[p][0] + self.x_offset, pos[p][1] + self.y_offset)
            elif pos[p][1] == self.pos_low:
                pos[p] = (pos[p][0] + self.x_offset, pos[p][1] - self.y_offset*2)
            else:
                pos[p] = (pos[p][0] + self.x_offset, pos[p][1] + self.y_offset)
        labeldescr = nx.draw_networkx_labels(g, pos=pos, font_size=9, labels=nodelabeldict)
        for n, l in labeldescr.items():
            if labeldescr[n]._y == self.pos_low-2*self.y_offset:
                l.set_rotation(self.findegree)
        nx.draw_networkx(g, pos_node, node_size=80,  with_labels=False, alpha=0.5)

    def nodes_arrange(self, g1, g, x5_pos, conf5_width, i, j, z, t):
        dofs = self.dofs
        g1.add_node(dofs[i][2][j][z], pos=(x5_pos[z], self.pos_high))
        if dofs[i][1][j] in dofs[i][2][j][z]:
            g.add_edge(dofs[i][1][j], dofs[i][2][j][z])
            conf6number = len(dofs[i][3][t])
            conf6_width = conf5_width // conf6number
            x6_pos = [(x5_pos[z] + conf6_width * (k - conf6number / 2)) for k in range(conf6number)]
            for r in range(len(dofs[i][3][t])):
                g1.add_node(dofs[i][3][t][r], pos=(x6_pos[r], self.pos_low))
                if dofs[i][2][j][z] in dofs[i][3][t][r]:
                    g1.add_edge(dofs[i][2][j][z], dofs[i][3][t][r])

    def tree_plot(self, i, save=True):
        plt.figure(i, figsize=(24.0, 10.0))
        plt.subplots_adjust(0.01, 0.01, 0.99, 0.96, 0.2, 0.16)
        gs = gridspec.GridSpec(4, 1, height_ratios=[1, 1, 1, 1])
        self.subplot_arrange(self.G, gs[0], self.dofs[i][0][0])
        self.subplot_arrange(self.G1, gs[1], self.dofs[i][0][0])
        self.subplot_arrange(self.G2, gs[2], self.dofs[i][0][0])
        self.subplot_arrange(self.G3, gs[3], self.dofs[i][0][0])
        if save:
            plt.savefig("combinations/" + str(i) + ".png", format='png')

    def subplot_arrange(self, g, gs, conf_name):
        plt.subplot(gs)
        self.tree_visuality(g)
        if gs.num1 == 0:
            plt.title("All possible 4&5 DOF combination for the 3DOF configuration: " + conf_name)
        else:
            plt.title("All possible 5&6 DOF combination for the 3DOF configuration: " + conf_name)
        pos = nx.get_node_attributes(g, 'pos')
        min_x = 100000
        max_x = -100000
        for p in pos:
            if pos[p][0] < min_x:
                min_x = pos[p][0]
            if pos[p][0] > max_x:
                max_x = pos[p][0]
        plt.ylim(35, 90)
        plt.xlim(min_x-150, max_x + 150)


def save_json(name="data_file", data=None):
    with open(name + ".json", "w") as write_file:
        json.dump(data, write_file, indent=2)


def load_json(name="data_file"):
    with open(name + ".json", "r") as read_file:
        return json.load(read_file)


def filter_configs(config, dof=6):
    """"
    check if the specific config is oass the assumptions
    """
    a = [config.split("+")[i].split(",") for i in range(dof)]
    joint = [a[i][0] for i in range(dof)]
    axe = [a[i][1] for i in range(dof)]
    dof = len(joint) - 1
    if joint.count("pris") > 3:  # no more than 3 prismatics joints
        return False
    if (joint[dof - 1] == "pris" and axe[dof - 1] == "z" and joint[dof - 2] == "roll") or \
            (joint[dof - 2] == "pris" and joint[dof - 1] == "roll" and axe[dof - 1] == "z"):
        # after pris -> roll z \ roll -> pris z can be only: pris y / pitch y / roll y
        if joint[dof] == "roll" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pitch" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pris" and (axe[dof] == "x" or axe[dof] == "z"):
            return False

    if joint[dof - 1] == "roll":  # after roll only pris z/y, pitch y, roll y
        if joint[dof] == "roll" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pitch" and (axe[dof] == "z" or axe[dof] == "x"):
            return False
        elif joint[dof] == "pris" and axe[dof] == "x":
            return False

    if joint[dof - 1] == "pris" and joint[dof] == "pris" and axe[dof] == "z":
        # after prismatic joint can not be prismatic in z axe
        return False

    if joint[dof - 2] == "pris" and joint[dof - 1] == "pris" and joint[dof] == "pris" and axe[dof] == "y":
        return False

    else:
        return True


def calc_configs(file_name="5dof_configs", dof=6):
    """
    calculate all the configuration of manipulators
    :param file_name: file name with previous configurations
    :return:  data - all the configurations
    """
    csv_data = HandleCSV()
    data_from_csv = csv_data.read_data(file_name)
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
                if filter_configs(conf, dof):
                    all_data.append(conf)
            if len(all_data)>0:
                data.append(all_data)
    return data


def save2csv(data, file_name="combinations/6dof_configs", dof=6):
    """"
    save the all the configuration of the specific DOF in CSV file
    data: data to save
    file_name: name of the file to save
    """
    csv_data = HandleCSV()
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
    # n = len(save)
    cf = ([list(i) for i in zip(*[save[i:i + len(save)] for i in range(0, len(save), len(save))])])
    csv_data.save_data(cf, file_name, dof)


if __name__ == "__main__":
    dof = 6  # number of DOF to calculate all the possible combinations
    sav2csv = False
    calc_config = True  # calc from file or use saved configuration from json file
    make_tree = True  # create tree
    configs2tree = True  # when creating tree to calculate from scratch or using exiting json file

    if calc_config:
        # calculate config from csv file
        prev_comb = "combinations/" + str(dof-1) + "dof_configs"  # the previuos combination file name
        configurations = calc_configs(prev_comb, dof)
        save_json("combinations/" + str(dof) + "dof configurations", configurations)
    else:
        # load configurations from previus run
        configurations = load_json("combinations/" + str(dof) + "dof " + "configurations")
    if sav2csv:
        # Save the configuration to scv file
        save2csv(configurations, "combinations/" + str(dof) + "dof_configs", dof)
    if make_tree:
        # plot \ save tree images
        tree = Tree()
        if configs2tree:
            all_dofs = tree.arrange_configs2tree(configurations)
            save_json("combinations/combs_tree", all_dofs)
        else:
            all_dofs = load_json("combinations/combs_tree")
            tree.dofs = all_dofs
        tree.tree_create()
