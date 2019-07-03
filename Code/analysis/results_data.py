import xlrd
import xlwt
import os
import matplotlib.pyplot as plt
import numpy as np
import csv
import itertools


class HandleCSV(object):

    def save_data(self, data, file_name):
        """Save to csv format"""
        with open(file_name + ".csv", 'w', newline='') as file_n:
            writer = csv.writer(file_n, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            data0csv = [[]]
            data2csv = []
            reached = 0

            for dat in data:
                for k in dat[0]:
                    if k[3] == "True":
                        reached = reached + 1
                    arm = k[2].split("_")
                    k.append(arm[13])  # one before last axe
                    k.append(self.joint_type(arm[12]))  # one before last joint
                    k.append(arm[16])  # last axe
                    k.append(self.joint_type(arm[15]))  # last joint
                    data0csv[0].append(k)
                data2csv.append(data0csv)
            tested = len(data)*len(data[0][0])

            writer.writerow(["Date", "Time", "combination", "Result", "axe one before last", "joint one before last",
                             "Last Axe", "Last joint", "Total Tested", tested, "Total Reached", reached])
            for dat in data2csv:
                # data2csv = [dat[0], last_axe, last_joints]
                writer.writerows(dat[0])
            return reached, file_name

    def read_data(self, file_name):
        with open(file_name, 'r') as _filehandler:
            csv_file_reader = csv.reader(_filehandler)
            data = []
            res = []
            empty = True
            for row in csv_file_reader:
                while "" in row:
                    row.remove("")
                if len(row) > 0:
                    data.append(row)
                    empty = False
                else:
                    if not empty:
                        res.append(self.read_data_action(data))
                        data = []
                    empty = True
            res.append(self.read_data_action(data))  # append the last session
            return res

    @staticmethod
    def read_data_action(in_data, true_limit=3):
        return_data = []
        if len(in_data) == 1638:
            le = 0
        else:
            le = 1
        for j in range(le, len(in_data)):
            if in_data[j][0] == "Date":
                continue
            [date, time] = in_data[j][0].split()
            arm = in_data[j][1]
            count = in_data[j][2].count("True")
            good = "False"
            if count >= true_limit:
                good = "True"
            return_data.append([date.strip(':'), time.strip(':'), arm.strip(':'), good.strip(':')])
        return return_data

    @staticmethod
    def joint_type(arm):
        if arm[-1] == 'l':
            return "roll"
        elif arm[-1] == 'h':
            return "pitch"
        elif arm[-1] == 's':
            return "pris"
        elif arm[-1] == 'w':
            return "yaw"


def autolabel(rects, ax, xpos='center'):
        """
        Attach a text label above each bar in *rects*, displaying its height.

        *xpos* indicates which side to place the text w.r.t. the center of
        the bar. It can be one of the following {'center', 'right', 'left'}.
        """

        ha = {'center': 'center', 'right': 'left', 'left': 'right'}
        offset = {'center': 0, 'right': 1, 'left': -1}

        for rect in rects:
            height = rect.get_height()
            ax.annotate('{}'.format(height),
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(offset[xpos] * 3, 3),  # use 3 points offset
                        textcoords="offset points",  # in both directions
                        ha=ha[xpos], va='bottom')


def data_plot(plot_data):
    width = 1  # the width of the bars
    fig, ax = plt.subplots()
    tickets_labels = [""]
    data = []
    for i in range(0, len(plot_data)):
        first = (plot_data[i][0])
        avarage_data = round((first / 3.0), 2)
        av = ax.bar(i, first, width / 4)
        autolabel(av, ax, "center")
        av = ax.bar(i, avarage_data, width/20, color="blue")
        autolabel(av, ax, "center")
        tickets_labels.append(plot_data[i][1])
        data.append(first)
    avg_data = np.average(data)
    std_data = np.std(data)
    av = ax.bar(i+1, avg_data, width / 4, color="cyan", yerr=std_data)
    autolabel(av, ax, "center")
    av = ax.bar(i+1,  round(avg_data/3, 2), width / 20, color="blue")
    autolabel(av, ax, "center")
    tickets_labels.append("Total")
    ax.set_xticklabels(tickets_labels)
    # ax.legend()
    fig.tight_layout()
    plt.show()


def sim2csv():
    res_files = []
    tests = []
    handle = HandleCSV()
    current_loc = os.getcwd()
    files = os.listdir(current_loc + "\csv_data_first simulations")
    for file in files:
        if file.endswith(".csv"):
            res_files.append([os.path.splitext(file)[0], os.path.splitext(file)[1]])
            name = os.path.splitext(file[21:])[0]
            if "x" in name:
                tests.append(name[:name.index("x")])
            else:
                tests.append(name)
    tests_no_duplicates = list(dict.fromkeys(tests))
    raeched_comb = []
    for test in tests_no_duplicates:
        res_test = []
        while test in tests:
            ind = tests.index(test)
            tests[ind] = None
            res_test.append(handle.read_data("csv_data_first simulations/" + files[ind]))
        raeched_comb.append(handle.save_data(res_test, "res" + str(test)))
    return raeched_comb


comb_reached = sim2csv()
data_plot(comb_reached)


def arms_combinations(number=6):
    joint_filtered = []
    links_length_filtered = []
    link_min = 0.1
    link_interval = 0.3
    link_max = 1
    lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
    joints = [list(tup) for tup in list(itertools.product(['Px', 'Py', 'Pz', 'Rollz', 'Rolly', 'Rollx',
                                'Pitchz', 'Pitchy',  'Pitchx', 'Yawz', 'Yawx', 'Yawy'], repeat=number))]
    # joints_axis = [list(tup) for tup in list(itertools.product(['x', 'y', 'z'], repeat=number))]
    links_length = [list(tup) for tup in list(itertools.product(lengths_2_check, repeat=number))]
    combinations = len(joints) * len(links_length)
    # print(combinations)
    for joint in joints:
        pris_num = 0
        if joint[0] == 'Rollz':
            for j in range(1, len(joint)):
                if 'Px' == joint[j] or 'Py' == joint[j] or 'Pz' == joint[j]:
                    pris_num = pris_num + 1
                if joint[j] == joint[j-1]:
                    if "Rollz" == joint[j]:
                        pris_num = 6
                    elif "Pz" == joint[j]:
                        pris_num = 6

            if pris_num < 4:
                joint_filtered.append(joint)

    for link in links_length:
        if link[0] == 0.1:
            if sum(link) > 1:
                links_length_filtered.append(link)
    combinations_filtered = len(joint_filtered) * len(links_length_filtered)
    # print(combinations_filtered)
    return combinations_filtered


# c = []
# total = 0
# for n in [3, 4, 5, 6]:
#     c.append(arms_combinations(n))
# for i in range(len(c)):
#     total = total + c[i]


# def data_from_excel():
#     # get Excel files
#     xlsx_files = []
#     curr_loc = os.getcwd()
#     exc_files = os.listdir(curr_loc)
#     for exc_file in exc_files:
#         if exc_file.endswith(".xlsx"):
#             xlsx_files.append([os.path.splitext(exc_file)[0], os.path.splitext(exc_file)[1]])
#
#     data = ()
#     for f in range(len(xlsx_files)):
#         a = []
#         temp_file = xlsx_files[f][0] + xlsx_files[f][1]
#         # To open Workbook
#         wb = xlrd.open_workbook(temp_file)
#         sheet = wb.sheet_by_index(0)
#         for j in range(3):
#             a.append(sheet.cell_value(0, 17 + j))
#         data = data + (xlsx_files[f][0], a)
#     data_plot(data)
#
#
# def write_excel():
#     book = xlwt.Workbook()
#     sheet1 = book.add_sheet("PySheet1")
#
#     cols = ["A", "B", "C", "D", "E"]
#     txt = "Row %s, Col %s"
#
#     for num in range(5):
#         row = sheet1.row(num)
#         for index, col in enumerate(cols):
#             value = txt % (num + 1, col)
#             row.write(index, value)
#
#     book.save("test.xls")
