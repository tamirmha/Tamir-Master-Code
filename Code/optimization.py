# coding=utf-8
""" Variables: x1 : Joints type - array [Roll, Pitch, Pris]
            x2: Previous axe - array [X, Y, Z]
            x3 : Link Length - array [0.1, 0.4, 0.7]
            x4 : DOF - Int [3, 4, 5, 6]
Objectives:
1)	Min  Degree of Redundancy     [-3:0]
2)	Max  manipulability [0-1]
3)	Min  z (Mid-Range Proximity)
Constrains:
•	Sum (X3) > 1
•	X1[0] = Roll
•	X2[0] = Z
•	X3[0]=0.1
•	No more than 3 Pris in X1
•	If (X1[i]==Roll and X2[i]==Z) than (X1[i+1]!=Roll and X2[i+1]!=Z)
•	Arrival points : reach to one from two upper points and to the middle and bottom points
"""
# from typing import Iterable

# from simulator import simulate
from ros import UrdfClass
from Other import load_json, save_json, clock, Concepts, MyCsv, get_key, listener, pickle_load_data, pickle_save_data
from scipy.spatial import distance
import numpy as np
import copy
from datetime import datetime, timedelta
from tqdm import tqdm
from time import time, sleep
import os
import shutil
import sys
import getpass
from multiprocessing import Process
from simulator import simulate
# from ros_error_check import listener
# import shlex
# import subprocess
# from ros import Ros


# np.random.seed(100100)
# # np.random.seed(1010101)
# # np.random.seed(111111)
# # np.random.seed(0)


class Optimization:
    """ This class run all the optimization method using the DWOI & Problem classes"""
    def __init__(self, run_time=7, num_gens=200, parents_number=1, large_concept=1000, arms_limit=[1, 0],
                 allocation_delta=10, greedy_allocation=True, percent2continue=90, min_cont_par=10, gen_start=0,
                 low_cr_treshhold=0.001, high_cr_treshhold=0.003, name="optimizaion_WOI"):
        # how many dats to run
        self.run_time = run_time
        # how many gens to run
        self.num_gens = num_gens
        # Start generation
        self.gen_start = gen_start
        # number of parents
        self.parents_number = parents_number
        # define what is a large concept
        self.large_concept = large_concept
        # population limit: arms_limit[0]: minimum number of configs, arms_limit[1]: % of population
        self.arms_limit = arms_limit
        # how many generation between resource allocation
        self.allocation_delta = allocation_delta
        # to use greedy aloocation or not
        self.greedy_allocation = greedy_allocation
        # when using greedy allocation - how many percent will continue
        self.percent2continue = percent2continue
        # when using greedy allocation - if Cr < this threshold the concept will be stopped
        self.low_cr_treshhold = low_cr_treshhold
        # when using greedy allocation - if Cr < this threshold the concept will be paused
        self.high_cr_treshhold = high_cr_treshhold
        # the name of the json file of the DWOI - saved every gen
        self.name=name
        # Initilize all the concepts GA
        print("initiliaze data")
        # load the first WOI
        self.woi = DWOI(run_time=self.run_time)
        self.probs = self.init_concepts(larg_concept=self.large_concept, arm_limit=self.arms_limit,
                                    number_of_parents=self.parents_number, delta_allocation=self.allocation_delta)
        min_cont = min_cont_par * len(self.probs) / 100
        self.ra = ResourceAllocation(greedy=self.greedy_allocation, cont_per_max=self.percent2continue,
                            cont_min=min_cont, t_low=self.low_cr_treshhold, t_high=self.high_cr_treshhold)
        if self.run_folder():
            self.probs = pickle_load_data("problems")
            self.woi = pickle_load_data("woi")
            self.woi.start_time = time()
            self.woi.run_time = self.run_time* 24 * 3600
            self.woi.stopped = False
        save_name = 'results_file' + datetime.now().strftime("%d_%m_") +  "6dof_4d_"
        MyCsv.save_csv([], save_name)

    def init_concepts(self, larg_concept=1500, arm_limit=None, number_of_parents=1, delta_allocation=10):
        """ Initilize all the concept and the first populations
        :param larg_concept: [int] minimum number of configurations in concept in order to concept will be large
        :param arm_limit: [2 elements list] the limits: arm_limit[0] is the minimum number of the population
                arm_limit[1] is the percent of number of configurations in the concept
        :param number_of_parents: [int] how much from the populatoin will be parents
        :param delta_allocation: how many generation between resource allocation
        :return prob - [list of objects] all the data of each concept
        """
        if arm_limit is None:
            arm_limit = [1, 0]
        # load all the concepts
        concepts_with_conf, confs_results = self.get_prev_data()
        prob = []
        for i in range(len(concepts_with_conf)):
            # Initiliaze each concept
            name_of_concept = list(concepts_with_conf)[i]
            number_of_arms = self.set_pop_size(len(concepts_with_conf[name_of_concept]), arm_limit)
            prob.append(Problem(concept_name=name_of_concept, confs_of_concepts=concepts_with_conf[name_of_concept],
                                confs_results=confs_results[name_of_concept], parents_number=number_of_parents,
                                pop_size=number_of_arms, larg_concept=larg_concept, delta_allocation=delta_allocation))
            # initiliaze population
            prob[i].set_population(prob[i].rand_pop())
        return prob

    @staticmethod
    def get_prev_data(all_concepts_json="jsons/concepts+configs+results", ga_json="jsons/concepts2ga"):
        """ get all the previous data and the concepts to enter into the ga and return only the relevant data
        :param all_concepts_json - [str] json file with all the simulated data
        :param ga_json - [str] all the concepts to check in the GA
        :return ga_data- dictoinary with all the configurations and there results per concept
        """
        all_concepts = load_json(all_concepts_json)
        ga_concepts = load_json(ga_json)
        ga_data = {}
        for k in ga_concepts:
            if k in all_concepts:
                ga_data[k] = all_concepts[k]
        return ga_concepts, ga_data

    @staticmethod
    def set_pop_size(num_concept_confs, min_configs=None):
        """decide the population size: going to be the bigger between min_configs[1] % of concepts number or min_configs[0]
         :param num_concept_confs: [int] number of configurations in this concept
         :param min_configs: [2 elements list] the limits: min_configs[0] is the minimum number of the population
                            min_configs[1] is the percent of number of configurations in the concept
        :return pop_size: [int] size of the pipulation
         """
        if min_configs is None:
            min_configs = [1, 0]
        if num_concept_confs * min_configs[1] / 100 > min_configs[0]:
            pop_size = num_concept_confs * min_configs[1] / 100
        else:
            pop_size = min_configs[0]
        return pop_size

    def run_folder(self):
        params = "Number of gens: " + str(self.num_gens) + "\nparents_number: " + str(self.parents_number) + \
                 "\nLarge concept: " + str(self.large_concept) + "\nRun Time (days): " + str(self.run_time) + \
                 "\nAllocation Delta: " + str(self.allocation_delta) + "\nPopulation: " + str(self.arms_limit[0]) + \
                 "\nGreedy Allocation: " + str(self.greedy_allocation) +\
                 "\nPercent to continue: " + str(self.percent2continue) + \
                 "\nLow Cr treshhold: " + str(self.low_cr_treshhold) +\
                 "\nHigh Cr treshhold: " + str(self.high_cr_treshhold)
        # enter all the results to one folder
        results_folder = "opt_results/" + datetime.now().strftime("%d_%m")  # + "-0"
        # while os.path.isdir(results_folder):
        #     results_folder = results_folder[:-1] + str(int(results_folder[-1]) + 1)
        if not os.path.isdir(results_folder):
            exist = False
            os.mkdir(results_folder)
            os.mkdir(results_folder + "/urdf")
            print(results_folder + " folder created \nStart Optimization")
            with open(results_folder + "/parameters.txt", "w+") as f:
                f.write(params)
                f.close()
        else:
            exist = True
        # change the working dir
        os.chdir(results_folder)
        return exist

    def run(self):
        woi = self.woi
        probs = self.probs
        cr = []
        # running each generation
        for n in range(self.gen_start-1, self.num_gens):
            # Save the current WOI
            save_json(self.name, [{"gen_" + str(woi.get_gen()): woi.get_last_dwoi()}])
            print("\033[34m" + "\033[47m" + "Generation " + str(n + 1) + " of " + str(self.num_gens) + " generations"
                  + "\033[0m")
            # simulate the population
            probs = self.sim(prob=probs)
            to_pop = []
            for t in range(len(probs)):
                if n == 0:
                    self.woi.cr[probs[t].concept_name] = []
                probs[t] = self.run_gen(probs[t])
                if probs[t].concept_name in woi.dwoi[-1][4]:
                    probs[t].in_dwoi = True
                # Check Convergance rate
                if not (n + 1) % self.allocation_delta:
                    start_ind = self.allocation_delta * ((n + 1) / self.allocation_delta - 1)
                    end_ind = n
                    # if probs[t].in_dwoi or probs[t].paused:
                    #     continue
                    if probs[t].stopped:
                        to_pop.append(t)
                        continue
                    probs[t].set_cr(probs[t].get_dis()[start_ind], probs[t].get_dis()[end_ind])
                    cr.append(probs[t].get_cr())
                    self.woi.cr[probs[t].concept_name].append(probs[t].get_cr())
            # Resource allocation
            if not (n + 1) % self.allocation_delta:  # and self.greedy_allocation:
                for p in range(len(to_pop)):
                    probs.pop(to_pop[p] - p)
                probs = self.ra.run(cr, probs)
                cr = []
            # Update generation
            woi.set_gen(n + 1)
            # Check global stop condition
            woi.stop_condition(probs)
            save_json("woi_All", woi.__dict__)
            if woi.stopped:
                break
        self.probs = probs
        self.woi = woi

    def run_gen(self, prob):
        woi = self.woi
        # Stop Condition
        prob.local_stop_condition()
        # check if the local stop condition applied
        if prob.stopped:
            return prob
        elif prob.paused or prob.in_dwoi:
            prob.add_dis(1.5)
            return prob
        population = prob.get_population()
        # insert previous configurations into archive
        prob.set_prev_confs(population)
        # Evaluation
        confs_results = prob.evalute(np.asarray(population))
        # Update DWOI if necessary
        front = prob.domination_check(confs_results, copy.deepcopy(woi.get_last_dwoi()))
        if front != woi.get_last_dwoi():
            woi.set_dwoi(front)
        # elitism \ Non dominated soloution
        confs_results_elite = prob.one_pop_elitism(confs_results)
        # Check if large concept
        if prob.large_concept:  # if large concept
            # Assign fitness
            fitness = prob.assign_fitness(confs_results_elite, woi.get_last_dwoi())  # calc minimum distance for each config
            # Selection (RWS)
            selection = prob.selection(fitness, prob.parents_number)
            selected_confs_ind = prob.confs_by_indices(selection, fitness)
            selected_confs = prob.get_conifgs_by_indices(selected_confs_ind, confs_results_elite)
            # Mating
            population = prob.mating(selected_confs)
        else:  # if small concept
            # Random Selection
            population = prob.rand_pop()
        prob.set_population(population)
        prob.calc_dis()
        return prob

    def finish(self):
        print("Saving data...")
        save_json("woi_last", self.woi.__dict__, "w+")
        print("saved_last_woi")
        sleep(1)
        if os.path.isfile("problems.pkl"):
            os.remove("problems.pkl")
        # for p in tqdm(self.probs):
            # save_json("problems", [p.__dict__])
        pickle_save_data(self.probs, "problems")
        pickle_save_data(self.woi, "woi")
        print("saved problems")
        self.set_new_data()
        # plot_cr(os.getcwd() + "/woi_last", to_save=True)
        print("Finished")

    def sim(self, prob):
        # configurations to create urdf
        to_sim = []
        con = Concepts()
        k = 0
        for r in prob:
            to_sim.append(self.check_exist(r))
            k += 1
        if with_sim:
            # create urdf files
            con.create_files2sim(filter(None, to_sim))
            # move the files into the desired place
            if self.move_folder():
                print("start simulating")
                # cmd = 'gnome-terminal -- python simulator.py 6 '
                self.simulating()
                prob = self.new_data(prob)
        return prob

    @staticmethod
    def simulating():
        p = Process(target=simulate)
        p.start()
        # listener()
        p2 = Process(target=listener)
        p2.start()
        while not os.path.exists("finish.txt"):
            sleep(3)
        os.remove("finish.txt")
        p2.terminate()
        p.terminate()
        sleep(2)
        cmd = "kill -9 $(ps aux | grep [r]os | grep -v grep | grep -v arya | awk '{print $2}')"
        os.system(cmd)
        sleep(2)

    @staticmethod
    def check_exist(problem):
        """ return which urdfs to create for simulation
        :param problem: [object] of specific concept
        :return to_sim: [list] names of urdfs to create
        """
        pop = problem.get_population()
        to_sim = []
        for r in pop:
            res = problem.get_result(r)
            # check if the configuration allready simulated
            if len(res) == 0:
                to_sim.append(r)
            elif res["z"] is None:
                to_sim.append(r)
        return to_sim

    @staticmethod
    def move_folder(src_folder_name="urdf/6dof/", dst_folder_name=""):
        if not dst_folder_name:
            dst_folder_name = os.environ['HOME'] + \
                              "/Tamir_Ws/src/manipulator_ros/Manipulator/man_gazebo/urdf/6dof/combined/"
        if os.path.exists(dst_folder_name):
            shutil.rmtree(dst_folder_name)
        if os.path.exists(src_folder_name):
            shutil.move(src_folder_name, dst_folder_name)
            return True
        return False

    @staticmethod
    def new_data(prob):
        """ update each concept results after the simulation
        :param prob - [list of objects] the data of all the objects
        :return prob - [list of objects] updated data of all the objects
        """
        pth = "results_file" + datetime.now().strftime("%d_%m_") + "6dof_4d_"
        if not os.path.isfile(pth+".csv"):
            pth = "results_file" + datetime.strftime(datetime.now() - timedelta(1), "%d_%m_") + "6dof_4d_"
        data = MyCsv.load_csv(pth)
        for dat in data:
            k = 0
            outer_loop_stop = False
            for con in prob:
                j = 0
                for c in con.confs_of_concepts:
                    if dat["name"] == c:
                        outer_loop_stop = True
                        if dat["Z"] == -1:
                            dat["Z"] = 1
                        prob[k].confs_results.append({dat["name"]: {"mu": dat["mu"], "z": dat["Z"], "dof": dat["dof"],
                                                      "name": unicode(dat["name"])}})
                        break
                    j += 1
                k += 1
                if outer_loop_stop:
                    break
        return prob

    def set_new_data(self, all_concepts_json="jsons/concepts+configs+results", ga_json="jsons/concepts2ga"):
        """ get all the previous data and the concepts to enter into the ga and return only the relevant data
        :param all_concepts_json - [str] json file with all the simulated data
        :param ga_json - [str] all the concepts to check in the GA
        """
        data = self.csvs2data()
        jsons_folder = os.environ['HOME'] + "/Tamir/Master/Code/"
        all_concepts_json = jsons_folder + all_concepts_json
        ga_json = jsons_folder + ga_json
        all_concepts = load_json(all_concepts_json)
        ga_concepts = load_json(ga_json)
        for dat in tqdm(data):
            second_loop_stop = False
            for con in ga_concepts:
                k = 0
                for concept in ga_concepts[con]:
                    if dat["name"] == concept:
                        second_loop_stop = True
                        all_concepts[con].append({unicode(dat["name"]): {"mu": dat["mu"], "z": dat["Z"],
                                                "dof": dat["dof"], "name": unicode(dat["name"])}})
                        break
                    k += 1
                if second_loop_stop:
                    break
        if with_sim:
            save_json(all_concepts_json, all_concepts, "w+")
        # pickle_save_data(all_concepts, all_concepts_json + "new")

    @staticmethod
    def csvs2data():
        """ Take all the created CSVs and insert them into one variable
         :return data -[list of dicts] the results of all the simulated configuration in this run
         """
        data = []
        for file_csv in os.listdir(os.getcwd()):
            if file_csv.endswith(".csv"):
                data.append(MyCsv.load_csv(file_csv[:-4]))
        data = [val for sublist in data for val in sublist]
        return data


class Problem:
    def __init__(self, concept_name, confs_of_concepts, confs_results, pop_size=1, parents_number=1,
                 number_of_objects=3, larg_concept=1500, delta_allocation=10):
        self.pop_size = pop_size  # size of population
        self.confs_of_concepts = confs_of_concepts  # all possible configs names of the concept
        self.confs_results = confs_results  # all the configurations of the concept and their indices
        self.confs_archive = []  # Archive of all the selected configurations
        self.large_concept = len(confs_of_concepts) > larg_concept  # True if large False otherwise
        self.concept_name = concept_name
        self.elit_confs = []
        self.parents_number = parents_number
        self.dof = int(str(concept_name).split(" ")[5].split(",")[0])
        self.stopped = False  # if local condition is true or very slow Cr
        self.paused = False  # if the Cr is slow
        self.in_dwoi = False  # the concept in the DWOI - is pasued
        self.number_of_objects = number_of_objects
        self.population = []
        self.dis = []  # distance from the origin
        self.cr = 0  # Convergence rate of the concept
        self.delta_allocation = delta_allocation

    def set_population(self, pop):
        self.population = pop
        save_json("pop", pop)

    def get_population(self):
        return self.population

    def rand_pop(self, pop_size=None):
        """ select random configurations that belongs to the concept
        :param pop_size-[int] the size of selected population - if nothing entred than take self.pop_size
        :return confs- [list]  names of configurations
        """
        if pop_size is None:
            pop_size = self.pop_size
        confs_of_concepts = self.get_configs()
        confs_archive = self.get_prev_confs()
        remain_confs = len(confs_of_concepts) - len(confs_archive)
        # check that the number of selected configuration wont be bigger than the number of remain configurations
        if remain_confs < pop_size:
            pop_size = remain_confs
        num_of_confs = len(confs_of_concepts)
        # select random indices
        indices = np.random.randint(0, num_of_confs, pop_size)
        # get the configurations of the random indices
        random_confs = np.ndarray.tolist(np.asarray(confs_of_concepts)[indices])
        confs = []
        while len(random_confs) != 0:
            for conf in random_confs:
                # if the random configuration in the archive
                if conf in confs_archive:
                    # delete the configuration and select new one
                    random_confs.remove(conf)
                    new_ind = np.random.randint(0, num_of_confs, 1)
                    while new_ind in indices:
                        new_ind = np.random.randint(0, num_of_confs, 1)
                    random_confs.append(confs_of_concepts[new_ind[0]])
                else:
                    confs.append(conf)
                    random_confs.remove(conf)
        return confs

    def evalute(self, pop):
        f1 = []
        f2 = []
        f3 = []
        pops = []
        for r in pop:
            pops.append(r)
            res = self.get_result(r)
            if not res:  # If error occured and the simulation stopped
                continue  # were break
            if res["z"] == "70":
                res["z"] = 2
                res["mu"] = -1
            if with_sim:
                try:
                    if res["z"] is not None:
                        f3.append(int(res["dof"]))  # dof
                        f2.append(1 - float(res["mu"]))  # manipulability
                        if res["z"] != -1.0:
                            f1.append(float(res["z"]))  # Mid-Range Proximity
                        else:
                            f1.append(2.0)
                    else:
                        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " + str(r))
                except:
                    print(res)
            else:
                if len(res) == 0:
                    f3.append(self.dof)  # dof
                    f2.append(np.around(np.random.normal(0.05, 0.01), 3))  # manipulability
                    f1.append(np.around(np.random.normal(0.05, 0.01), 3))
                # check if the configuration allready simulated
                elif res["z"] is not None:
                    f3.append(int(res["dof"]))  # dof
                    f2.append(1 - float(res["mu"]))  # manipulability
                    f1.append(float(res["z"]))  # Mid-Range Proximity
                else:
                    f3.append(self.dof)  # dof
                    f2.append(np.around(np.random.normal(0.05, 0.01), 3))  # manipulability
                    f1.append(np.around(np.random.normal(0.05, 0.01), 3))  # Mid-Range Proximity
        return [f1, f2, f3, pops, self.concept_name]

    def stop_condition(self):
        """Stop condition of the concept (not of all the problem)
        condition 1 - if all the configuration been checked
        """
        stop = False
        if self.get_prev_confs().shape[0] == self.get_configs().shape[0]:
            stop = True
        return stop

    def one_pop_elitism(self, new_gen):
        elite_confs = self.get_elite_confs()
        if not elite_confs:
            self.set_elite_confs(new_gen)
            return new_gen
        elite_confs = self.domination_check(new_gen, elite_confs)
        # elite_confs[0].append(new_gen[0][0])
        # elite_confs[1].append(new_gen[1][0])
        # elite_confs[2].append(new_gen[2][0])
        # elite_confs[3].append(new_gen[3][0])
        self.set_elite_confs(elite_confs)
        return elite_confs

    def elitism(self, new_gen):
        """Elitism - Make sure the offsprings will be better than the previous generation
        :param new_gen - [list of lists] the results of the last genration
        :return new_elite_gen -[list of lists] the best population from the elite and last offsprings
        """
        elite_confs = self.get_elite_confs()
        if not elite_confs:
            self.set_elite_confs(new_gen)
            return new_gen
        new_elite_gen = []
        new_elite_gen_name = []
        for i in range(self.pop_size):
            for j in range(self.pop_size):
                if elite_confs[0][i] >= new_gen[0][j] and elite_confs[1][i] >= new_gen[1][j]:
                    new_elite = [new_gen[0][j], new_gen[1][j], new_gen[2][j]]
                    new_elite_gen_name.append(new_gen[3][j])
                    new_gen[0][j] = 100
                    new_gen[1][j] = 100
                    new_elite_gen.append(new_elite)
                    break
            if j == self.pop_size - 1:
                new_elite = [elite_confs[0][i], elite_confs[1][i], elite_confs[2][i]]
                new_elite_gen_name.append(elite_confs[3][i])
                new_elite_gen.append(new_elite)
        new_elite_gen = np.ndarray.tolist(np.asarray(new_elite_gen).T) + [new_elite_gen_name] + [elite_confs[4]]
        self.set_elite_confs(new_elite_gen)
        return new_elite_gen

    @staticmethod
    def assign_fitness(points, dwoi):
        """ calculate the distance from each point to the DWOI
        :param points - [list of lists] the results of last evaluation
        :param dwoi - [list of lists] last Dynamic Window of Intrest
        :return [np array] of the shortest distance form each point to the DWOI
        """
        dwoi_loc = np.asarray(dwoi[:3]).T
        dist = distance.cdist(dwoi_loc, np.asarray(points[:3]).T, 'euclidean')
        return np.amin(np.around(dist, 3), axis=0)

    @staticmethod
    def selection(dis, num_of_returns=2):
        """Make selection by using RWS return the value
        :param dis: [np array]  distances
        :param num_of_returns:[int] how many elements to return
        :return :[np array] the selected indices that won the RWS
        """
        selected = []
        c = 3.
        fitnes = np.exp(-c * dis)
        fp = np.asarray([i / sum(fitnes) for i in fitnes])
        roullete = np.asarray([sum(fp[:x + 1]) for x in range(len(fp))])
        for i in range(num_of_returns):
            wheel = np.random.uniform(0, 1)
            ind = np.where((roullete - wheel) > 0, (roullete - wheel), np.inf).argmin()
            selected.append(round(fitnes[ind], 3))
        if selected == [0.0]:
            return np.asarray([100])  # (distance of the point 70,71) if the initial position didnt reach
        return np.abs(np.round(np.log(selected) / c, 3))  # [10-x for x in selected]

    def mating(self, parents, mutation_percent=100):
        """
        :param parents - [list] all the parents that go into the mating pool
        :param mutation_percent - [int] how much mutation wanted
        :return offspring - [list] names of the offsprings
         """
        cr = self.get_cr()
        offspring_size = self.pop_size
        num_mutations = int(offspring_size * mutation_percent / 100.)
        num_crossover = offspring_size - num_mutations
        total_attempts = 50  # to prevent infinite loop
        offspring = []
        prev_confs = self.get_prev_confs()
        mut_offspring = 0
        cross_offspring = 0
        for i in range(num_mutations):
            attempt = 0
            in_concept = False
            cross_ok = False
            if mutation_percent == 100:  # if the mutation % is 100 than no crossover
                cross_ok = True
            mut_ok = False
            spring = []
            while not in_concept:
                # select parents randomlly
                j = np.random.randint(0, len(parents))
                k = np.random.randint(0, len(parents))
                parent_1 = np.asarray(Concepts.arm2parts(str(parents[k]).split("_")))
                parent_2 = np.asarray(Concepts.arm2parts(str(parents[j]).split("_")))
                # calculate crossover and mutation and check if they are belongs to the concept
                if not cross_ok and cross_offspring < num_crossover:
                    cross_spring = self.crossover([parent_1, parent_2])
                    cross_conf = self.check_conf(cross_spring) and cross_spring not in offspring
                    if cross_conf:
                        cross_ok = cross_spring not in self.get_prev_confs()
                        spring.append(cross_spring)
                        cross_offspring += 1
                if not mut_ok and mut_offspring < num_mutations:
                    # mut_spring = self.mutation_round(parent_1)
                    nb = 1
                    if cr == 0 or len(self.get_population()) > 100:  # if the Cr=zero or more than 100 gens- mutate more
                        nb = 2
                    mut_spring = self.mutation_rand(parent_1, nb)
                    mut_conf = self.check_conf(mut_spring) and mut_spring not in offspring and mut_spring not in prev_confs
                    if mut_conf:
                        mut_ok = mut_spring not in self.get_prev_confs()
                        spring.append(mut_spring)
                        mut_offspring += 1
                in_concept = cross_ok and mut_ok or cross_ok and mut_offspring == num_mutations
                if attempt >= total_attempts:
                    # print("mating problem " + str(i))
                    if len(offspring) > 2 * num_mutations - 1:
                        spring = self.rand_pop(1)
                        if spring not in offspring:
                            break
                    else:
                        rand_spring = self.rand_pop(2 + abs(len(offspring) - 2 * i))
                        for s in rand_spring:
                            if s not in offspring:
                                spring.append(s)
                        if spring:
                            break
                        else:
                            self.stopped = True
                            break
                attempt += 1
            for s in spring:
                offspring.append(unicode(s))
        if len(offspring) > offspring_size:
            offspring = offspring[:offspring_size]
        elif len(offspring) < offspring_size:
            # the mating didn't success to create new offsprings
            self.stopped = True
            print("local stop condition - no more offsprings")
        return offspring

    def crossover(self, parents):
        """ select a random number (link) and all the links&joints&axes until this point
        taken from parent 1 and all the links&joints&axes from this point are taken from parent 2
        :param parents- [list] names of parents
        :return -[str] name of offspring
        """
        dof = self.dof
        for j in range(dof):
            point_of_split = np.random.randint(1, dof)
            child = np.ndarray.tolist(parents[0][:, :point_of_split].copy())
            [child[i].extend(np.ndarray.tolist(parents[1][i, point_of_split:])) for i in range(3)]
            child = self.to_urdf(child[0], child[1], child[2], "")
            if self.check_conf(child["name"]):
                break
        if j == dof - 1:
            child = np.ndarray.tolist(parents[0])
            child = self.to_urdf(child[0], child[1], child[2], "")
        return child["name"]

    def mutation_rand(self, parent, nb_prox=1):
        """ switch randomlly 2 links and joints
        :param parent: [np array] - names of parents
        :param nb_prox:  [int] - proximity of the neighboors: 1-first neigboor, 2-second neighboor
        :return -[str] name of offspring
        """
        dof = self.dof
        arm_index = []
        nbs_dict = {1: [[2, 4], [5, 8]], 2: [[1, 5, 8], [4, 7]], 3: [[6, 9], []],
                    4: [[1, 5, 7], [2, 8]], 5: [[2, 4, 8], [1, 7]], 6: [[3, 9], []],
                    7: [[1, 4, 8], [2, 5]], 8: [[2, 5, 7], [1, 4]], 9: [[3, 6], []],
                    11: [[12, 14, 17], [15, 18]], 12: [[11, 15, 18], [14, 17]], 13: [[16, 19], []],
                    14: [[11, 15, 17], [12, 18]], 15: [[12, 14, 18], [11, 17]], 16: [[13, 19], []],
                    17: [[11, 14, 18], [12, 15]], 18: [[12, 15, 17], [11, 14]], 19: [[13, 16], []],
                    21: [[22, 24, 27], [25, 28]], 22: [[21, 25, 28], [24, 27]], 23: [[26, 29], []],
                    24: [[21, 25, 27], [22, 28]], 25: [[22, 24, 28], [21, 27]], 26: [[23, 29], []],
                    27: [[21, 24, 28], [22, 25]], 28: [[22, 25, 27], [21, 24]], 29: [[23, 26], []]
                    }
        dict_voc = {'roll z 0.1': 1, 'roll z 0.4': 2, 'roll z 0.7': 3,
                    'roll y 0.1': 4, 'roll y 0.4': 5, 'roll y 0.7': 6,
                    'roll x 0.1': 7, 'roll x 0.4': 8, 'roll x 0.7': 9,
                    'pitch z 0.1': 11, 'pitch z 0.4': 12, 'pitch z 0.7': 13,
                    'pitch y 0.1': 14, 'pitch y 0.4': 15, 'pitch y 0.7': 16,
                    'pitch x 0.1': 17, 'pitch x 0.4': 18, 'pitch x 0.7': 19,
                    'pris z 0.1': 21, 'pris z 0.4': 22, 'pris z 0.7': 23,
                    'pris y 0.1': 24, 'pris y 0.4': 25, 'pris y 0.7': 26,
                    'pris x 0.1': 27, 'pris x 0.4': 28, 'pris x 0.7': 29
                    }
        for r in parent.T:
            arm_index.append(dict_voc["_".join(r).replace("_", " ")])
        ind = np.random.randint(1, dof)
        to_replace = arm_index[ind]
        nbs = nbs_dict[to_replace]
        if nb_prox == 1:
            rand_nb = np.random.choice(nbs[0])
        else:
            rand_nb = np.random.choice(nbs[0] + nbs[1])
        offspring = parent.T
        offspring[ind] = get_key(rand_nb, dict_voc).split(" ")
        offspring = offspring.T
        offspring = self.to_urdf(offspring[0], offspring[1], offspring[2], "")
        return offspring["name"]

    def mutation_round(self, parent):
        """ Switch the elements in circle. the last will be first first will be second and so on
        :param parent- [np array] names of parents
        :return -[str] name of offspring
        """
        dof = self.dof
        indices = np.concatenate((np.asarray([0, dof - 1]), np.arange(1, dof - 1)))
        offspring = parent[:, indices]
        offspring = self.to_urdf(offspring[0], offspring[1], offspring[2], "")
        if offspring["name"][15] == "x":  # for roll\pris x in start
            offspring["name"] = offspring["name"][:15] + "y" + offspring["name"][16:]
        elif offspring["name"][16] == "x":  # for pitch x in start
            offspring["name"] = offspring["name"][:16] + "y" + offspring["name"][17:]
        return offspring["name"]

    @staticmethod
    def confs_by_indices(select, fit):
        """ get the selected distances and return the indices of them if distance appear more
        than once it than take the second item
        :param select - [np array]  the selected fitnesses to find their indices
        :param fit - [np array]  array of the fitness of the configurations
        :return indices - [list]  indices of the selected configuration
         """
        indices = []
        if select[0] > 1:
            return [0]
        for x in select:
            ind = np.argwhere(fit == x)
            if ind.shape[0] > 1:
                fit[ind[0][0]] = 100
            if len(ind) == 0:
                ind = np.argwhere(np.abs(fit - x) < 0.003)
            try:
                indices.append(ind[0][0])
            except:
                print("ind=" + str(ind) + "  fit=" + str(fit) + "\nx=" + str(x) + "\nselect=" + str(select))
        return indices

    @staticmethod
    def get_conifgs_by_indices(conf_indices, get_configs):
        """get configuration by indices
        :param conf_indices - [list] indices of the configurations
        :param get_configs - [list] the configs that we want their indices
        :return - [list] - names of the configurations in the selected indices
        """
        get_configs = np.asarray(get_configs)
        return np.ndarray.tolist(np.asarray(get_configs.T[3])[conf_indices])
        # return np.ndarray.tolist(np.unique(get_configs[conf_indices]))

    def check_conf(self, confs):
        """Check if the configurations are belongs to the concept
        :param confs: [str] configuration to check if in the concept
        :return :[boolean] True if in the concept False otherwise
        """
        configs = self.get_configs()
        return confs in configs

    def get_configs(self):
        return self.confs_of_concepts

    def domination_check(self, conf, front):
        """ check if any of the configuration dominated the DWOI and update the DWOI
        :param conf - [list] the results of the configuration
        :param front - [list] the results of the DWOI
        :return front - [list] the new front
        """
        for i, j, k, l in zip(conf[0], conf[1], conf[2], conf[3]):  # z, mu, dof, configuration
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
                    if front[4] != self.concept_name:
                        del front[4][ind]
                    if not added:
                        front[0].append(i)
                        front[1].append(j)
                        front[2].append(k)
                        front[3].append(l)
                        if front[4] != self.concept_name:
                            front[4].append(self.concept_name)
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
                if front[4] != self.concept_name:
                    front[4].append(self.concept_name)
        return front

    def set_prev_confs(self, confs):
        """Add configurtions to the archive """
        self.confs_archive += confs

    def get_prev_confs(self):
        """ Return the configuration in the archive"""
        return self.confs_archive

    def get_result(self, config):
        result = []
        for i in range(len(self.confs_results)):
            if self.confs_results[i].keys()[-1] == config:
                result = self.confs_results[i][self.confs_results[i].keys()[0]]
                break
        return result

    def set_elite_confs(self, confs):
        self.elit_confs = confs

    def get_elite_confs(self):
        return self.elit_confs

    def local_stop_condition(self):
        if len(self.get_prev_confs()) == len(self.confs_of_concepts):
            # check if all the configurations simulated
            self.stopped = True

    def set_cr(self, dis_start, dis_end):
        self.cr = abs((dis_start - dis_end) / self.delta_allocation)

    def get_cr(self):
        return self.cr

    def get_dis(self):
        return self.dis

    def set_dis(self, dis):
        self.dis = dis

    def add_dis(self, d):
        self.dis.append(d)

    def calc_dis(self):
        """Calc distance of the Non-Dominate (elite) results from the origin """
        elite = self.get_elite_confs()
        dist = distance.cdist(np.asarray(elite[:2]).T, np.zeros((1, 2)), 'euclidean')
        min_dis = dist.min()
        self.add_dis(min_dis)
        return min_dis

    @staticmethod
    def to_urdf(interface_joints, joint_parent_axis, links, folder):
        """Create the desired confiuration
        :param interface_joints- [list] roll,pitch or prismatic (roll -revolute around own Z axis,
                                        pitch - revolute that not roll,  pris - prismatic along z)
        :param links -[list] length of links
        :param joint_parent_axis - [list] the axe, in the parent frame, which each joint use
        :param folder - [str] where the urdf saved - not in use
        :return -[dict] -contain the configuration name and all the data to the urdf file
            """
        joints = []
        joint_axis = []
        rpy = []
        # file_name = os.environ['HOME'] + "\Tamir_Ws\src\manipulator_ros\Manipulator\man_gazebo\urdf\"
        # + str(dof) + "dof\combined\"
        file_name = ""
        rolly_number = -1
        pitchz_number = 1
        prisy_number = -1
        for i in range(len(joint_parent_axis)):
            # file_name += interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + \
            #              links[i].replace(".", "_")
            file_name += interface_joints[i] + "_" + joint_parent_axis[i] + "_" + str(links[i]).replace(".", "_")
            if interface_joints[i] == "roll":
                joints.append("revolute")
                joint_axis.append('z')
                if joint_parent_axis[i] == "y":
                    rolly_rot = '${' + str(rolly_number) + '/2*pi} '
                    rpy.append([rolly_rot, '0 ', '0 '])
                    rolly_number = rolly_number * -1
                elif joint_parent_axis[i] == "x":
                    rpy.append(['0 ', '${pi/2} ', '0 '])
                elif joint_parent_axis[i] == "z":
                    rpy.append(['0 ', '0 ', '0 '])
            elif interface_joints[i] == "pitch":
                joints.append("revolute")
                joint_axis.append('y')
                if joint_parent_axis[i] == "y":
                    rpy.append(['0 ', '0 ', '0 '])
                elif joint_parent_axis[i] == "x":
                    rpy.append(['0 ', '0 ', '${-pi/2} '])
                elif joint_parent_axis[i] == "z":
                    # rpy.append(['${pi/2} ', '0 ', '0 '])
                    pitchz = '${' + str(pitchz_number) + '/2*pi} '
                    rpy.append([pitchz, '0 ', '0 '])
                    pitchz_number = pitchz_number * -1
            elif interface_joints[i] == "pris":
                joints.append("prismatic")
                joint_axis.append('z')
                if joint_parent_axis[i] == "y":
                    # rpy.append(['${pi/2} ', '0 ', '0 '])
                    prisy = '${' + str(prisy_number) + '/2*pi} '
                    rpy.append([prisy, '0 ', '0 '])
                    prisy_number = prisy_number * -1
                elif joint_parent_axis[i] == "x":
                    rpy.append(['0 ', '${-pi/2} ', '0 '])
                elif joint_parent_axis[i] == "z":
                    rpy.append(['0 ', '0 ', '0 '])
        arm = UrdfClass(links, joints, joint_axis, rpy)
        # arm.urdf_write(arm.urdf_data(), file_name)
        return {"arm": arm, "name": file_name, "folder": folder}


class DWOI:

    def __init__(self, concepts_file="jsons/front_concept", run_time=7):
        self.gen = 0
        self.dwoi = self.dwoi2conf(load_json(concepts_file))  # , self.gen])
        self.stopped = False
        self.start_time = time()
        self.run_time = run_time * 24 * 3600  # in seconds
        self.cr = {}

    def stop_condition(self, prbs):
        if self.run_time <= time() - self.start_time:
            print("Time Limit passed")
            self.stopped = True
        stopped = 0
        for q in prbs:
            if q.in_dwoi or q.stopped:
                stopped += 1
        if stopped == len(prbs):
            print("All concepts stopped or in DWOI")
            self.stopped = True

    def set_dwoi(self, dwoi):
        for i in range(len(dwoi[3])):
            if type(dwoi[3][i]) != unicode:
                dwoi[3][i] = unicode(dwoi[3][i])
        self.dwoi.append(dwoi)

    def get_all_dwoi(self):
        return self.dwoi

    def get_last_dwoi(self):
        return self.dwoi[-1][:]

    @staticmethod
    def dwoi2conf(dwoi):
        """ conf from the following format: 4D list:
        0 - list of z
        1 - list of mu
        2 - list of dof
        3 - list of configurations names
        4 - concept
        5 - number of gen
        """
        conf = [[], [], [], [], []]  # , dwoi[1]]
        for w in dwoi:
            conf[0].append(w["z"])
            conf[1].append(w["mu"])
            conf[2].append(w["dof"])
            conf[3].append(w["configuration"])
            conf[4].append(w["concept"])
        return [conf]

    def set_gen(self, gen):
        self.gen = gen

    def get_gen(self):
        return self.gen


class ResourceAllocation:
    def __init__(self, greedy=True, t_low=0.001, t_high=0.003, cont_per_max=90, cont_min=1.0):
        self.t_low = t_low
        self.t_high = t_high
        self.cont_per_max = cont_per_max
        self.cont_min = round(cont_min)
        self.greedy = greedy

    @staticmethod
    def sort_cr(crs):
        """ Get list of convergence rate and sort them from high to low
        :param crs: [list] - convergence rate
        :return sorted_cr, sorted_cr_ind: [np array, np array] - results sorted, indices of the results sorted
        """
        crs = np.asarray(crs)
        sorted_cr = np.sort(crs)
        sorted_cr_ind = np.argsort(crs)
        return sorted_cr, sorted_cr_ind  # [::-1]

    def set_group(self, sorted_cr):
        """ Each cr goes to his group:
        :param sorted_cr: [np array] - sorted lit of the results
        :return group_a: [np array] - continue any way
                group_b: [np array] - continue only to fullfil the amount unless paused
                group_c: [np array] - continue only to fullfil the amount unless stopped
        """
        group_a = sorted_cr[sorted_cr > self.t_high]
        group_b = sorted_cr[(sorted_cr <= self.t_high) & (sorted_cr > self.t_low)]
        group_c = sorted_cr[self.t_low >= sorted_cr]
        return [group_a, group_b, group_c]

    def set_decision(self, groups):
        """Decide if a concepts will continue / pause / stop
        :param groups: [list of 3 np arrays] - groups[0] - set 1, groups[1] - set 2, groups[2] - set 3
        :return: con_res, pause_res, stop_res - results to continue, pause, stop
        """
        total_concepts = 0
        for j in groups:
            total_concepts += len(j)
        if self.cont_per_max / 100. * total_concepts > self.cont_min:
            number_concepts_to_continue = self.cont_per_max / 100. * total_concepts
        else:
            number_concepts_to_continue = self.cont_min
        con_res = [i for i in groups[0]]
        pause_res = []
        stop_res = []
        for i in groups[1][::-1]:
            if len(con_res) < number_concepts_to_continue:
                con_res.append(i)
            else:
                pause_res = np.ndarray.tolist(groups[1][groups[1] > i])  # [j[0] for j in np.argwhere(groups[1] >= i)]
                stop_res = [i for i in groups[2]]
                return con_res, pause_res, stop_res
        for i in groups[2][::-1]:
            if len(con_res) < number_concepts_to_continue:
                con_res.append(i)
            else:
                stop_res.append(i)
        return con_res, pause_res, stop_res

    @staticmethod
    def assign2concepts(desicions, indx):
        """ decide to each concept's index if it will continue/pause/stopped
        :param desicions: [list of 3 lists] - decisions of continuation
        :param indx: [np array] - the original indices of the concepts
        :return continues, pauses, stopped : [np array, np array, np array]
        """
        continues = indx[:len(desicions[0])]
        pauses = indx[len(desicions[0]):len(desicions[1]) + len(desicions[0])]
        stopped = indx[len(desicions[1]) + len(desicions[0]):len(desicions[1]) + len(desicions[0]) + len(desicions[2])]
        return continues, pauses, stopped

    def run(self, crs, prob):
        """ Run all the sequance of the resource allocation
        :param crs: [list] - all the convergences rates
        :param prob: [list] - all the concepts
        :return: [list] - all the concepts
        """
        if not self.greedy:  # if fair method than no data allocation
            return prob
        print("Resource Allocation is made")
        cr_sorted, cr_sorted_ind = self.sort_cr(crs)
        decision = self.set_decision(self.set_group(cr_sorted))
        decisions = self.assign2concepts(decision, cr_sorted_ind[::-1])
        for i in tqdm(range(len(prob))):
            if prob[i].in_dwoi:
                continue
            elif i in decisions[0]:
                prob[i].stopped = False
                prob[i].paused = False
            elif i in decisions[1]:
                prob[i].stopped = False
                prob[i].paused = True
            elif i in decisions[2]:
                prob[i].stopped = True
                prob[i].paused = False
        return prob


if __name__ == '__main__':
    username = getpass.getuser()
    if username == "tamir":  # tamir laptop
        np.random.seed(100100)
        # np.random.seed(111011)
    gen_num = 50
    start_time = 0
    time_run = 0.5  # 7
    start_gen = 1
    greedy = True
    delta = 10
    per2cont = 90
    low_cr = 0.005
    high_cr = 0.01
    par_num = 1
    lar_con = 1500
    args = sys.argv
    if len(args) > 1:
        start_time = int(args[1])
        if len(args) > 2:
            start_gen = int(args[2])
            if len(args) > 3:
                delta = int(args[3])
                if len(args) > 4:
                    per2cont = int(args[4])
                    if len(args) > 5:
                        low_cr = float(args[5])
                        if len(args) > 6:
                            gen_num = int(args[6])
                            if len(args) > 7:
                                time_run = float(args[7])
                                if len(args) > 8:
                                    par_num = int(args[8])
                                    if len(args) > 9:
                                        lar_con = int(args[8])
    start_time = start_time/(3600.*24)
    c = Process(target=clock, args=((time_run-start_time)*3600*24,))
    c.start()
    tic = time()
    with_sim = True  # to run with simulatoin or with random results
    opt = Optimization(num_gens=gen_num, greedy_allocation=greedy, allocation_delta=delta, run_time=time_run-start_time,
                       large_concept=lar_con, percent2continue=per2cont, low_cr_treshhold=low_cr,
                       high_cr_treshhold=high_cr, parents_number=par_num, gen_start=start_gen)
    try:
        opt.run()
    finally:
        if with_sim:
            opt.finish()
        else:
            save_json("woi_last", opt.woi.__dict__, "w+")
        print(time()-tic)
        c.terminate()


# todo - check if after concept not in woi is return to run
# todo - decide: t_high, t_low, cont_per_max, cont_min @ resource allocation
