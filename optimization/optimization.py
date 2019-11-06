# from pymoo.algorithms.nsga3 import NSGA3
# from pymoo.factory import get_problem, get_reference_directions
# from pymoo.optimize import minimize
# from pymoo.visualization.scatter import Scatter
# import autograd.numpy as anp
# import numpy as np
# from pymoo.util.misc import stack
# from pymoo.model.problem import Problem
# from pymoo.algorithms.nsga2 import NSGA2
# from pymoo.performance_indicator.hv import Hypervolume
# from pymoo.factory import get_sampling, get_crossover, get_mutation
# from pymoo.factory import get_termination
# import matplotlib.pyplot as plt
#
#
# class MyProblem(Problem):
#
#     def __init__(self):
#         super().__init__(n_var=2, n_obj=2, n_constr=2, xl=anp.array([-2, -2]), xu=anp.array([2, 2]))
#
#     def _evaluate(self, x, out, *args, **kwargs):
#         f1 = x[:, 0]**2 + x[:, 1]**2
#         f2 = (x[:, 0]-1)**2 + x[:, 1]**2
#
#         g1 = 2*(x[:, 0]-0.1) * (x[:, 0]-0.9) / 0.18
#         g2 = - 20*(x[:, 0]-0.4) * (x[:, 0]-0.6) / 4.8
#
#         out["F"] = anp.column_stack([f1, f2])
#         out["G"] = anp.column_stack([g1, g2])
#
#     def _calc_pareto_front(self, flatten=True, **kwargs):
#         f1_a = np.linspace(0.1**2, 0.4**2, 100)
#         f2_a = (np.sqrt(f1_a) - 1)**2
#
#         f1_b = np.linspace(0.6**2, 0.9**2, 100)
#         f2_b = (np.sqrt(f1_b) - 1)**2
#
#         a, b = np.column_stack([f1_a, f2_a]), np.column_stack([f1_b, f2_b])
#         return stack(a, b, flatten=flatten)
#
#     def _calc_pareto_set(self, flatten=True, **kwargs):
#         x1_a = np.linspace(0.1, 0.4, 50)
#         x1_b = np.linspace(0.6, 0.9, 50)
#         x2 = np.zeros(50)
#
#         a, b = np.column_stack([x1_a, x2]), np.column_stack([x1_b, x2])
#         return stack(a, b, flatten=flatten)

#
# problem = MyProblem()
#
# algorithm = NSGA2(pop_size=40, n_offsprings=10, sampling=get_sampling("real_random"), crossover=get_crossover("real_sbx",
#                   prob=0.9, eta=15), mutation=get_mutation("real_pm", eta=20), eliminate_duplicates=True)
#
# termination = get_termination("n_gen", 40)
# res = minimize(problem, algorithm, termination, seed=1, pf=problem.pareto_front(use_cache=False), save_history=True,
#                verbose=True)
#
# ps = problem.pareto_set(use_cache=False, flatten=False)
# pf = problem.pareto_front(use_cache=False, flatten=False)
#
# plot = Scatter(title="Design Space", axis_labels="x")
# plot.add(res.X, s=30, facecolors='none', edgecolors='r')
# plot.add(ps, plot_type="line", color="black", alpha=0.7)
# plot.do()
# plot.apply(lambda ax: ax.set_xlim(-0.5, 1.5))
# plot.apply(lambda ax: ax.set_ylim(-2, 2))
# plot.show()
#
# plot = Scatter(title="Objective Space")
# plot.add(res.F)
# plot.add(pf, plot_type="line", color="black", alpha=0.7)
# plot.show()
#
# # create the performance indicator object with reference point (4,4)
# metric = Hypervolume(ref_point=np.array([1.0, 1.0]))
#
# # collect the population in each generation
# pop_each_gen = [a.pop for a in res.history]
#
# # receive the population in each generation
# obj_and_feasible_each_gen = [pop[pop.get("feasible")[:,0]].get("F") for pop in pop_each_gen]
#
# # calculate for each generation the HV metric
# hv = [metric.calc(f) for f in obj_and_feasible_each_gen]
#
# # visualze the convergence curve
# plt.plot(np.arange(len(hv)), hv, '-o')
# plt.title("Convergence")
# plt.xlabel("Generation")
# plt.ylabel("Hypervolume")
# plt.show()


from pymoo.model.problem import Problem
from pymoo.model.sampling import Sampling
from pymoo.model.crossover import Crossover
from pymoo.model.mutation import Mutation
import string
import numpy as np
from pymoo.algorithms.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter


# class MyProblem(Problem):
#
#     def __init__(self, n_characters=10):
#         super().__init__(n_var=1, n_obj=2, n_constr=0, elementwise_evaluation=True)
#         self.n_characters = n_characters
#         self.ALPHABET = [c for c in string.ascii_lowercase]
#
#     def _evaluate(self, x, out, *args, **kwargs):
#         n_a, n_b = 0, 0
#         for c in x[0]:
#             if c == 'a':
#                 n_a += 1
#             elif c == 'b':
#                 n_b += 1
#
#         out["F"] = np.array([- n_a, - n_b], dtype=np.float)
#
#
# class MySampling(Sampling):
#
#     def _do(self, problem, n_samples, **kwargs):
#         X = np.full((n_samples, 1), None, dtype=np.object)
#         for i in range(n_samples):
#             X[i, 0] = "".join([np.random.choice(problem.ALPHABET) for _ in range(problem.n_characters)])
#
#         return X
#
#
# class MyCrossover(Crossover):
#     def __init__(self):
#
#         # define the crossover: number of parents and number of offsprings
#         super().__init__(2, 2)
#
#     def _do(self, problem, X, **kwargs):
#
#         # The input of has the following shape (n_parents, n_matings, n_var)
#         _, n_matings, n_var = X.shape
#
#         # The output owith the shape (n_offsprings, n_matings, n_var)
#         # Because there the number of parents and offsprings are equal it keeps the shape of X
#         Y = np.full_like(X, None, dtype=np.object)
#
#         # for each mating provided
#         for k in range(n_matings):
#
#             # get the first and the second parent
#             a, b = X[0, k, 0], X[1, k, 0]
#
#             # prepare the offsprings
#             off_a = ["_"] * problem.n_characters
#             off_b = ["_"] * problem.n_characters
#
#             for i in range(problem.n_characters):
#                 if np.random.random() < 0.5:
#                     off_a[i] = a[i]
#                     off_b[i] = b[i]
#                 else:
#                     off_a[i] = b[i]
#                     off_b[i] = a[i]
#
#             # join the character list and set the output
#             Y[0, k, 0], Y[1, k, 0] = "".join(off_a), "".join(off_b)
#
#         return Y
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
#             r = np.random.random()
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
#
#
# # the input is the current population and a list of other populations.
# # the function returns if an individual in pop is equal to any other individual
# # in any other population.
# def func_is_duplicate(pop, *other, **kwargs):
#     if len(other) == 0:
#         return np.full(len(pop), False)
#
#     # value to finally return
#     is_duplicate = np.full(len(pop), False)
#
#     H = set()
#     for e in other:
#         for val in e:
#             H.add(val.X[0])
#
#     for i, (val,) in enumerate(pop.get("X")):
#         if val in H:
#             is_duplicate[i] = True
#         H.add(val)
#
#     return is_duplicate
#
#
# algorithm = NSGA2(pop_size=50, sampling=MySampling(), crossover=MyCrossover(), mutation=MyMutation(),
#                   eliminate_duplicates=func_is_duplicate)
# res = minimize(MyProblem(), algorithm, ('n_gen', 100), seed=1)
#
# results = res.X[np.argsort(res.F[:, 0])]
# count = [np.sum([e == "a" for e in r]) for r in results[:, 0]]
# print(np.column_stack([results, count]))
#
# Scatter().add(res.F).show()
