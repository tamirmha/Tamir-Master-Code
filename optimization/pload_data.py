import csv
# import autograd.numpy as anp
# import numpy as np
# from pymoo.util.misc import stack
# from pymoo.model.problem import Problem
#
# from pymoo.model.sampling import Sampling
#
# from pymoo.model.crossover import Crossover
#
# from pymoo.model.mutation import Mutation
#
# from pymoo.algorithms.nsga3 import NSGA3
# from pymoo.factory import get_problem, get_reference_directions
# from pymoo.optimize import minimize
# from pymoo.visualization.scatter import Scatter


# class MyProblem(Problem):
#
#     def __init__(self, scores, arms):
#         super().__init__(n_var=1, n_obj=4, n_constr=0, elementwise_evaluation=True)
#         self.scores = scores
#         self.arms = []
#         for arm in arms:
#             self.arms.append(arm["name"])
#
#     def _evaluate(self, x, out, *args, **kwargs):
#         # f1 = x[:, 0]**2 + x[:, 1]**2
#         # f2 = (x[:, 0]-1)**2 + x[:, 1]**2
#         for comb in x:
#             for res in self.scores:
#                 if comb == res["name"]:
#                     f1 = res["time"]
#                     f2 = res["mu"]
#                     f3 = res["LCI"]
#                     f4 = res["Z"]
#                     break
#                 else:
#                     f1 = 1000.0
#                     f2 = 1000.0
#                     f3 = 1000.0
#                     f4 = 1000.0
#
#         # g1 = 2*(x[:, 0]-0.1) * (x[:, 0]-0.9) / 0.18
#         # g2 = - 20*(x[:, 0]-0.4) * (x[:, 0]-0.6) / 4.8
#
#         out["F"] = anp.column_stack([f1, f2, f3, f4])
#         # out["G"] = anp.column_stack([g1, g2])
#
#     # def _calc_pareto_front(self, flatten=True, **kwargs):
#     #     f1_a = np.linspace(0.1**2, 0.4**2, 100)
#     #     f2_a = (np.sqrt(f1_a) - 1)**2
#     #
#     #     f1_b = np.linspace(0.6**2, 0.9**2, 100)
#     #     f2_b = (np.sqrt(f1_b) - 1)**2
#     #
#     #     a, b = np.column_stack([f1_a, f2_a]), np.column_stack([f1_b, f2_b])
#     #     return stack(a, b, flatten=flatten)
#     #
#     # def _calc_pareto_set(self, flatten=True, **kwargs):
#     #     x1_a = np.linspace(0.1, 0.4, 50)
#     #     x1_b = np.linspace(0.6, 0.9, 50)
#     #     x2 = np.zeros(50)
#     #
#     #     a, b = np.column_stack([x1_a, x2]), np.column_stack([x1_b, x2])
#     #     return stack(a, b, flatten=flatten)
#
#
# class MySampling(Sampling):
#
#     def _do(self, problem, n_samples, **kwargs):
#         x_samples = np.full((n_samples, 1), None, dtype=np.object)
#         for s in range(n_samples):
#             x_samples[s, 0] = [np.random.choice(problem.arms)]
#         return x_samples
#
#
# class MyCrossover(Crossover):
#     def __init__(self):
#         # define the crossover: number of parents and number of offsprings
#         super().__init__(1, 1)
#
#     def _do(self, problem, X, **kwargs):
#         return X  # [X[0][np.random.randint(0, X.shape[0])]]
#
#
# class MyMutation(Mutation):
#     def __init__(self):
#         super().__init__()
#
#     def _do(self, problem, X, **kwargs):
#
#         # for each individual
#         for i in range(len(X)):
#
#             r = 1  # np.random.random()
#
#             # with a probabilty of 40% - change the order of characters
#             if r < 0.4:
#                 perm = np.random.permutation(problem.n_characters)
#                 X[i, 0] = "".join(np.array([e for e in X[i, 0]])[perm])
#
#             # also with a probabilty of 40% - change a character randomly
#             elif r < 0.8:
#                 prob = 1 / problem.n_characters
#                 mut = [c if np.random.random() > prob
#                        else np.random.choice(problem.ALPHABET) for c in X[i, 0]]
#                 X[i, 0] = "".join(mut)
#
#         return X

# prob = MyProblem(passed_results, results)
# ref_dirs = get_reference_directions("das-dennis", 4, n_partitions=12)
# # samp = MySampling()
# # a = samp.do(prob, 5)
# # cross = MyCrossover()
# # b = cross._do(prob, a, parents=any)
#
# algorithm = NSGA3(pop_size=100, ref_dirs=ref_dirs, sampling=MySampling(), crossover=MyCrossover(), mutation=MyMutation(),
#                   eliminate_duplicates=False)
#
# res = minimize(prob, algorithm, seed=1, termination=('n_gen', 600), verbose=True)
# plot = Scatter(title="Objective Space")
# plot.add(res.F)
# # plot.add(res.X, s=30, facecolors='none', edgecolors='r')
#
# # plot.add(pf, plot_type="line", color="black", alpha=0.7)
# plot.show()


def save_data(data, file_name):
    """Save to csv format"""
    with open(file_name + ".csv", 'ab') as name:
        writer = csv.writer(name, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerows(data)


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
                name = row[1]
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
                result.append({"name": row[1], "time": float(row[5]), "mu": float(row[6]), "LCI": float(row[7]),
                     "Z": float(row[8]), "ri": float(row[9]), "dof": dof, "vars": [joints, prev_axe, link_length]})

        return result


from pymoo.model.problem import Problem
from pymoo.model.sampling import Sampling
from pymoo.model.crossover import Crossover
from pymoo.model.mutation import Mutation
import string
import numpy as np
from pymoo.algorithms.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter


class MyProblem(Problem):

    def __init__(self, n_characters=10):
        super().__init__(n_var=1, n_obj=2, n_constr=0, elementwise_evaluation=True)
        results = load_csv("C:\\Tamir\\Personal\\master\\Master_git\\Master\\optimization\\results_file23_10_4dof_4d_1930")
        self.n_characters = n_characters
        self.ALPHABET = []
        for arm in results:
            self.ALPHABET.append(arm["name"])
        # self.ALPHABET = [c for c in string.ascii_lowercase]

        self.passed_results = []
        penalty = 1000
        self.f_my = [[], [], [], []]
        for i in range(len(results)):
            if results[i]["time"] == -1:
                results[i]["time"] = penalty
                results[i]["mu"] = penalty
                results[i]["LCI"] = penalty
                results[i]["Z"] = penalty
                results[i]["ri"] = penalty
            else:
                self.passed_results.append(results[i])
            self.f_my[0].append(results[i]["time"])
            self.f_my[1].append(results[i]["mu"])
            self.f_my[2].append(results[i]["Z"])
            self.f_my[3].append(results[i]["dof"])
        # self.f_my = f


    def _evaluate(self, x, out, *args, **kwargs):
        f1 = 1000
        f2 = 1000
        for res in self.passed_results:
            if x == res["name"]:
                f1 = res["mu"]
                f2 = res["time"]

        # n_a, n_b = 80, 9
        # for c in x:
        #     if 'p' not in c:
        #         n_a += 1
        #     elif "o" not in c:
        #         n_b += 1

        out["F"] = np.array([f1, f2], dtype=np.float)


class MySampling(Sampling):

    def _do(self, problem, n_samples, **kwargs):
        X = np.full((n_samples, 1), None, dtype=np.object)
        for i in range(n_samples):
            X[i, 0] = np.random.choice(problem.ALPHABET)
            # X[i, 0] = "".join([np.random.choice(problem.ALPHABET) for _ in range(problem.n_characters)])
        return X


class MyCrossover(Crossover):
    def __init__(self):

        # define the crossover: number of parents and number of offsprings
        super().__init__(2, 2)

    def _do(self, problem, X, **kwargs):

        # The input of has the following shape (n_parents, n_matings, n_var)
        _, n_matings, n_var = X.shape

        # The output owith the shape (n_offsprings, n_matings, n_var)
        # Because there the number of parents and offsprings are equal it keeps the shape of X
        Y = np.full_like(X, None, dtype=np.object)

        # for each mating provided
        for k in range(n_matings):

            # get the first and the second parent
            a, b = X[0, k, 0], X[1, k, 0]

            # prepare the offsprings
            off_a = ["_"] * problem.n_characters
            off_b = ["_"] * problem.n_characters

            for i in range(problem.n_characters):
                if np.random.random() < 0.5:
                    off_a[i] = a[i]
                    off_b[i] = b[i]
                else:
                    off_a[i] = b[i]
                    off_b[i] = a[i]

            # join the character list and set the output
            Y[0, k, 0], Y[1, k, 0] = "".join(off_a), "".join(off_b)

        return Y


class MyMutation(Mutation):
    def __init__(self):
        super().__init__()

    def _do(self, problem, X, **kwargs):

        # for each individual
        for i in range(len(X)):

            r = np.random.random()

            # with a probabilty of 40% - change the order of characters
            if r < 0.4:
                perm = np.random.permutation(problem.n_characters)
                X[i, 0] = "".join(np.array([e for e in X[i, 0]])[perm])

            # also with a probabilty of 40% - change a character randomly
            elif r < 0.8:
                prob = 1 / problem.n_characters
                mut = [c if np.random.random() > prob
                       else np.random.choice(problem.ALPHABET) for c in X[i, 0]]
                X[i, 0] = "".join(mut)

        return X


# the input is the current population and a list of other populations.
# the function returns if an individual in pop is equal to any other individual
# in any other population.
def func_is_duplicate(pop, *other, **kwargs):
    if len(other) == 0:
        return np.full(len(pop), False)

    # value to finally return
    is_duplicate = np.full(len(pop), False)

    H = set()
    for e in other:
        for val in e:
            H.add(val.X[0])

    for i, (val,) in enumerate(pop.get("X")):
        if val in H:
            is_duplicate[i] = True
        H.add(val)

    return is_duplicate


# results = load_csv("C:\\Tamir\\Personal\\master\\Master_git\\Master\\optimization\\results_file23_10_4dof_4d_1930")
# passed_results = []
# penalty = 1000
# f = [[], [], [], []]
# for i in range(len(results)):
#     if results[i]["time"] == -1:
#         results[i]["time"] = penalty
#         results[i]["mu"] = penalty
#         results[i]["LCI"] = penalty
#         results[i]["Z"] = penalty
#         results[i]["ri"] = penalty
#     else:
#         passed_results.append(results[i])
#     f[0].append(results[i]["time"])
#     f[1].append(results[i]["mu"])
#     f[2].append(results[i]["Z"])
#     f[3].append(results[i]["dof"])

# prob = MyProblem(results)
algorithm = NSGA2(pop_size=50, sampling=MySampling(), crossover=MyCrossover(), mutation=MyMutation(),
                  eliminate_duplicates=func_is_duplicate)
res = minimize(MyProblem(), algorithm, ('n_gen', 30), seed=1)
#
# rsults = res.X[np.argsort(res.F[:, 0])]
# count = [np.sum([e == "a" for e in r]) for r in rsults[:, 0]]
# print(np.column_stack([rsults, count]))
#
Scatter().add(res.F).show()
