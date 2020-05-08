from Other import save_json, load_json, plot_cr, plot_woi
import os
import pickle
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
from tqdm import tqdm
from optimization import Problem
from scipy.spatial import distance
import copy
from hv import HyperVolume
from scipy.stats import wilcoxon


def run_results(prob, gen_num=1, elite_archive=True):
    scores = []
    for i in range(gen_num):
        scores.append([])
    elites = []
    gen_num_old = gen_num
    for b in prob:
        gen_num = gen_num_old
        if elite_archive:
            if b.elit_confs_archive[0][0] != 1.0:
                elites.append(b.elit_confs_archive)
        else:
            if b.elit_confs[0][0] != 1.0:
                elites.append(b.elit_confs)
        if gen_num >= len(b.confs_archive):
            gen_num = len(b.confs_archive)
        conf = b.confs_archive[:gen_num]
        res = b.confs_results
        for r in res:
            for c in range(gen_num):
                if len(conf) < gen_num:
                    continue
                if r.keys()[0] == conf[c]:
                    if float(r[conf[c]]["z"]) == 2.0 or float(r[conf[c]]["z"]) == 1.0:
                        # continue
                        r[conf[c]]["z"] = "0.5"
                        r[conf[c]]["mu"] = "0"
                    scores[c].append([float(r[conf[c]]["z"]), 1 - float(r[conf[c]]["mu"])])
                    continue
    return scores, elites


def calc_dis(elite):
    # elite = mut_a[i][1][0][j]
    dis_a_min = 1.5
    for k in range(len(elite[0])):
        dis_a = distance.euclidean((elite[0][k], elite[1][k]), (0, 0))
        if dis_a < dis_a_min:
            dis_a_min = dis_a
    return dis_a_min


def mutation_check(ami,  tamir, comb):
    folder_name = os.getcwd() + ami
    folder_name_t = os.getcwd() + tamir
    folder_name_c = os.getcwd() + comb
    with open(folder_name+"problems.pkl") as f:
        a = pickle.load(f)
    with open(folder_name_t+"problems.pkl") as f:
        t = pickle.load(f)
    with open(folder_name_c + "problems.pkl") as f:
        c = pickle.load(f)
    mut_a = []
    mut_t = []
    mut_c = []
    for i in tqdm(range(6)):
        mut_a.append(run_results(a[i:i+1], 1240))
        mut_t.append(run_results(t[i:i+1], 1240))
        mut_c.append(run_results(c[i:i+1], 1240))
    delta = 1
    plt.ioff()
    # plt.figure(figsize=(24.0, 10.0)).canvas.set_window_title('Cr')
    fig, axs = plt.subplots(6, 1, figsize=(24, 10), facecolor='w', edgecolor='k')
    plt.subplots_adjust(left=0.05, bottom=0.07, right=0.99, top=0.99)
    for i in tqdm(range(len(a))):
        distances_a = [[], [], [], [], [], []]
        distances_t = [[], [], [], [], [], []]
        distances_c = [[], [], [], [], [], []]
        for j in range(0, 1240, delta):
            if mut_a[i][0][j]:
                elite = mut_a[i][1][0][j]
                dis_a_min = calc_dis(elite)
                distances_a[i].append(dis_a_min)
            if mut_t[i][0][j]:
                elite = mut_t[i][1][0][j]
                dis_t_min = calc_dis(elite)
                distances_t[i].append(dis_t_min)
            if mut_c[i][0][j]:
                elite = mut_c[i][1][0][j]
                dis_c_min = calc_dis(elite)
                distances_c[i].append(dis_c_min)
        axs[i].scatter(range(len(distances_a[i])), distances_a[i], color="b", marker="^", label="Ami mutation")
        axs[i].scatter(range(len(distances_t[i])), distances_t[i], color="k", marker="*", label="Tamir mutaion ")
        axs[i].scatter(range(len(distances_c[i])), distances_c[i], color="r", marker="+", label="comobined mutation")
        axs[i].legend(loc="best")
        axs[i].set_xticks(np.arange(0, 1240, step=40))
        axs[i].set_xlim(0)
        axs[i].set_ylim(0)
    plt.xlabel("Generations", fontsize=26)
    axs[3].set_ylabel("Distance to Origin", fontsize=26)
    plt.show()


def first_gen_and_elite_res(name="/opt_results/22_04_tamir_mut/"):
    folder_name = os.getcwd() + name
    with open(folder_name+"problems.pkl") as f:
        a = pickle.load(f)
    gen_nums = 1
    d, e = run_results(a, gen_nums, elite_archive=False)
    save_json(folder_name + "first_gen_res", d, "w+")
    save_json(folder_name + "elits", e, "w+")
    b = load_json(folder_name + "elits")


def domination_check(conf, front, concept_name):
    """ check if any of the configuration dominated the DWOI and update the DWOI
    :param conf - [list] the results of the configuration
    :param front - [list] the results of the DWOI
    :return front - [list] the new front
    """
    for i, j, k, l in zip(conf[0], conf[1], conf[2], conf[3]):  # z, mu, dof, configuration
        if i == -1.0 or i == -1 or i == -70.0 or i == -70:
            continue
        added = False
        point_dominated = False
        for i_front, j_front, k_front, l_front in zip(front[0], front[1], front[2], front[3]):
            # check if the point is dominate the front
            if i <= i_front and j <= j_front and k <= k_front:
                ind = front[3].index(l_front)
                del front[0][ind]
                del front[1][ind]
                del front[2][ind]
                del front[3][ind]
                if front[4] != concept_name:
                    del front[4][ind]
                if not added:
                    front[0].append(i)
                    front[1].append(j)
                    front[2].append(k)
                    front[3].append(l)
                    if front[4] != concept_name:
                        front[4].append(concept_name)
                    added = True
            # check if front dominate the point
            elif i >= i_front and j >= j_front and k >= k_front:
                point_dominated = True
                break
        if not (added or point_dominated):
            front[3].append(l)
            front[0].append(i)
            front[1].append(j)
            front[2].append(k)
            if front[4] != concept_name:
                front[4].append(concept_name)
    # print(front)
    return front


def no_woi_front(elites):
    front = elites[0]
    concepts = []
    for i in range(len(front[0])):
        concepts.append(front[4])
    front[4] = concepts
    for elite in elites[1:]:
        front = domination_check(elite, front, elite[4])
    return front


def woi_comprasion(names=["/results/mutauioncheck/22_04_tamir_mut/"]):
    # folder_name = os.getcwd() + names[1]
    folder_name = os.getcwd() + "/results/mutauioncheck/"
    t = load_json(os.getcwd() + names[1] + "woi_last")
    pre_woi = copy.deepcopy(t["dwoi"][0])
    # a = load_json(os.getcwd() + names[0] + "woi_last")
    # c = load_json(os.getcwd() + names[3] + "woi_last")
    # r = load_json(os.getcwd() + names[2] + "woi_last")
    # t_woi = copy.deepcopy(t["dwoi"][-1])
    # c_woi = copy.deepcopy(c["dwoi"][-1])
    # a_woi = copy.deepcopy(a["dwoi"][-1])
    # r_woi = copy.deepcopy(r["dwoi"][-1])
    t_woi = load_json(os.getcwd() + names[1] + "elits")
    t_woi = no_woi_front(t_woi)
    a_woi = load_json(os.getcwd() + names[0] + "elits")
    a_woi = no_woi_front(a_woi)
    c_woi = load_json(os.getcwd() + names[3] + "elits")
    c_woi = no_woi_front(c_woi)
    r_woi = load_json(os.getcwd() + names[2] + "elits")
    r_woi = no_woi_front(r_woi)
    all_points =[[], [], [], [], [], []]   # [[]] * 220  #
    k = 0
    concepts_names = []
    with open(folder_name+"problems.pkl") as f:
        problems = pickle.load(f)
    front = copy.deepcopy([pre_woi[1], pre_woi[0], pre_woi[2], pre_woi[3],  pre_woi[4]])
    d = []
    for prob in problems:
        for res in prob.confs_results:
            conf = [[1-float(res[res.keys()[0]]["mu"])], [float(res[res.keys()[0]]["z"])],
                    [float(res[res.keys()[0]]["dof"])], [res[res.keys()[0]]["name"]]]
            front = domination_check(conf, front, prob.concept_name)
            if conf[0][0] == 71 or conf[1] == 70 or conf[0][0] == 2.0 or conf[1] == 2.0:
                conf[0] = [1]
                conf[1] = [0.5]
            d.append(conf[0][0])
            all_points[k].append([conf[0][0], conf[1][0]])
        all_points[k] = np.asarray(all_points[k], dtype=float).T
        concepts_names.append(prob.concept_name)
        k += 1
    plt.figure(figsize=(24.0, 10.0)).canvas.set_window_title('WOI_comprasion')
    plt.subplots_adjust(left=0.07, bottom=0.05, right=0.98, top=0.95)
    x = [[], [], [], [], [], []]
    y = [[], [], [], [], [], []]
    for xt, yt, dt in zip(t_woi[0], t_woi[1], t_woi[2]):
        if dt == 6 or dt == 6.0:
           x[0].append(xt)
           y[0].append(yt)
    for xa, ya, da in zip(a_woi[0], a_woi[1], a_woi[2]):
        if da == 6 or da == 6.0:
            x[1].append(xa)
            y[1].append(ya)
    for xc, yc, dc in zip(c_woi[0], c_woi[1], c_woi[2]):
        if dc == 6 or dc == 6.0:
            x[5].append(xc)
            y[5].append(yc)
    for xr, yr, dr in zip(r_woi[0], r_woi[1], r_woi[2]):
        if dr == 6 or dr == 6.0:
            x[2].append(xr)
            y[2].append(yr)
    for xp, yp, dp in zip(pre_woi[0], pre_woi[1], pre_woi[2]):
        if dp == 6 or dp == 6.0:
            x[3].append(xp)
            y[3].append(yp)
    for xf, yf, df in zip(front[1], front[0], front[2]):
        if df == 6 or df == 6.0:
            x[4].append(xf)
            y[4].append(yf)
    colors = ["b", "k", "r", "g", "orange", "cyan"]
    shapes = ["*", "^", ">", "8", ".", "<"]
    # labels = ["Tamir_mut", "Ami_mut", "Com_mut", "pre_DWOI", "Best_DWOI"]
    labels = ["Tamir_mut", "Ami_mut", "Rand_mut", "pre_DWOI", "Best_DWOI", "Com_mut"]
    # labels = ["Vm1", "Vm2", "Vm3", "pre_DWOI", "Best_DWOI"]
    for i in range(6):
        a = np.asarray([x[i], y[i]])
        inds = np.argsort(a[0])
        x[i] = a[0, inds]
        y[i] = a[1, inds]
        plt.plot(x[i], y[i], color=colors[i], marker=shapes[i], label=labels[i])
    for conc_points, label in zip(all_points,concepts_names):
        x = conc_points[1]
        y = conc_points[0]
        plt.scatter(x, y, alpha=0.1, s=60)
    plt.xlabel("Mid Proximity", fontsize=20)
    plt.ylabel("1 - Manipulability Index", fontsize=20)
    plt.title("WOI Comprasion", fontsize=26)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.show()
    # plt.savefig()


def all_points():
    folder_name = os.getcwd() + "/results/mutauioncheck/"
    all_points =[[], [], [], [], [], []]
    k = 0
    concepts_names = []
    with open(folder_name+"problems.pkl") as f:
        problems = pickle.load(f)
    d = []
    for prob in problems:
        for res in prob.confs_results:
            conf = [[1-float(res[res.keys()[0]]["mu"])], [float(res[res.keys()[0]]["z"])],
                    [float(res[res.keys()[0]]["dof"])], [res[res.keys()[0]]["name"]]]
            if conf[0][0] == 71 or conf[1] == 70 or conf[0][0] == 2.0 or conf[1] == 2.0:
                conf[0] = [1]
                conf[1] = [0.5]
            d.append(conf[0][0])
            all_points[k].append([conf[0][0], conf[1][0]])
        all_points[k] = np.asarray(all_points[k], dtype=float).T
        concepts_names.append(prob.concept_name)
        k += 1
    return all_points, concepts_names


# For animation
def init_lines():
    for line in lines:
        line.set_data([],[])
    return lines


def animate_lines(o):
    i = o * d + d
    colors = ["b", "r", "k", "cyan"]
    for j, line in enumerate(lines):
        x = []
        y = []
        for p in range(len(woi[j][i % 1240])):
            x.append(woi[j][i % 1240][p][0])
            y.append(woi[j][i % 1240][p][1])
        line.set_data(x, y)
        line.set_label(labels[j])
        line.set_color(colors[j])
    ttl.set_text("Generation: " + str(i))
    plt.legend()
    return lines


def add_data(t):
        wo = []
        for k in range(len(t)):
            wo.append([])
            woi = np.asarray(t[k][t[k].keys()[0]][:3])
            inds = np.argsort(woi[0])
            woi[0] = woi[0, inds]
            woi[1] = woi[1, inds]
            woi[2] = woi[2, inds]
            for j in range(len(woi[2])):
                if woi[2][j] == 6.0 or woi[2][j] == 6:
                    wo[k].append([woi[0][j], woi[1][j]])
        if len(wo) < 1240:
            for t in range(len(wo), 1240):
                wo.append(wo[-1])
        return wo


def plot_init_woi():
    x = []
    y = []
    for k in range(len(woi[0][0])):
        x.append([woi[0][0][k][0]])
        y.append([woi[0][0][k][1]])
    plt.plot(x, y, label="Initial DWOI", color="g")


create_woi_cr = False
woi_n_generate = False
anim = True
plot_conceot_front = False
calc_hv = False

fol = "/results/mutauioncheck/woi_nadir/"
end_fol = ""
names = [fol + "ami/", fol + "Tamir/", fol + "rand/", fol + "comb/"]
# names = [fol + "ami/", fol + "Tamir/", fol + "rand/", fol + "comb/"]
if calc_hv:  # HV calculation
    referencePoint = [0.5, 1]
    volumes = []
    medians = []
    min_manip = []
    k = 0
    hv = HyperVolume(referencePoint)
    for fol in tqdm(names):
        volumes.append([])
        min_manip.append([])
        for dircetor in os.listdir(os.getcwd() + fol):
            if dircetor == "hv.json":
                continue
            name = fol + dircetor
            woi = load_json(os.getcwd() + name + "/woi_last")["dwoi"][-1]
            front = np.asarray(woi[:2]).T
            volumes[k].append(round(hv.compute(front), 4))
            min_manip[k].append(np.min(front[:,1] ))
        save_json(os.getcwd() + fol + "hv", volumes[k], "w+")
        medians.append(np.median(volumes[k]))
        k += 1
    # wil = wilcoxon(medians)
    wil2 = wilcoxon(volumes[1], volumes[0])
    wil3 = wilcoxon(volumes[0], volumes[1])
if create_woi_cr:
    for fol in tqdm(names):
        for dircetor in os.listdir(os.getcwd()+fol):
            created = False
            if os.path.isdir(os.getcwd() + fol + "/" + dircetor) and dircetor != "urdf":
                name = fol + "/" + dircetor + "/"
                first_gen_and_elite_res(name=name)
                plot_cr(os.getcwd() + name + "woi_last", to_save=True)
                plot_woi(os.getcwd() + name, to_save=True)
                created = True
            else:
                name = fol
                continue
        if not created:
            first_gen_and_elite_res(name=name)
            plot_cr(os.getcwd() + name + "woi_last", to_save=True)
            plot_woi(os.getcwd() + name, to_save=True)
if woi_n_generate:
    # mutation_check(names[0], names[1], names[2])
    woi_comprasion(names=names)
if plot_conceot_front:
    fig, axs = plt.subplots(6, 1, figsize=(24, 10), facecolor='w', edgecolor='k')
    plt.subplots_adjust(left=0.05, bottom=0.07, right=0.99, top=0.99)
    colors = ["b", "k", "r", "cyan", "y", "purple"]
    points, names = all_points()
    i = 0
    major_ticks = np.arange(0, 0.5, 0.1)
    minor_ticks = np.arange(0, 0.5, 0.01)

    for conc_points, label in zip(points, names):
        x = conc_points[1]
        y = conc_points[0]
        mid_delta = 0.01
        man_delta = 0.7
        mid = x < mid_delta
        man = y < man_delta
        com = np.logical_and(mid, man)
        label = str(round(100.*np.sum(com)/np.sum(x<0.6), 1)) \
                + "% of the result are smaller than " + str((mid_delta, man_delta))
        axs[i].scatter(x, y,  label=label, color=colors[i])
        axs[i].legend()
        axs[i].set_xticks(major_ticks)
        axs[i].set_xticks(minor_ticks, minor=True)
        axs[i].grid(which='both')
        axs[i].grid(which='minor', alpha=0.3)
        i += 1
    plt.xlabel("Mid Proximity", fontsize=26)
    axs[3].set_ylabel("1 - Manipulability", fontsize=26)
if anim:
    for cr in ["mut_cr_30/", "mut_cr_50/", "mut_cr_100/", "mut_cr_regular/"]:
        d = 3
        total_frames = 1240
        labels = ["ami","Tamir", "rand", "Comb"]
        # labels = ["VM1", "VM2", "VM3"]
        woi = []
        names = [fol + cr + "ami/", fol + cr  + "Tamir/", fol + cr  + "rand/", fol + cr  + "comb/"]
        for fold in names:
            woi.append(add_data(load_json(os.getcwd() + fold + "optimizaion_WOI")))
        fig = plt.figure()
        plt.subplots_adjust(left=0.1, bottom=0.1, right=0.98, top=0.95)
        ax = plt.axes(xlim=(-0.01, 0.5), ylim=(0, 1))
        line, = ax.plot([], [], lw=2)
        plt.grid()
        ttl = ax.text(.5, 1.02, '', transform=ax.transAxes, va='center')
        ax.set_ylabel("1 - Manipulability")
        ax.set_xlabel("Mid Proximity")
        n = 4
        lines = [plt.plot([], [])[0] for _ in range(n)]
        plot_init_woi()
        anim = animation.FuncAnimation(fig, animate_lines, init_func=init_lines,
                               frames=total_frames/d, interval=70, repeat=False)
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=10, metadata=dict(artist='Me'), bitrate=1800)
        anim.save(os.getcwd() + fol + cr+ 'WOI_comprasion.mp4',  writer=writer)
        # anim.save(os.getcwd() + names[0][:-4] + str(total_frames)
        #           + '_WOI_comprasion.gif',  writer='imagemagick')
        plt.show()
