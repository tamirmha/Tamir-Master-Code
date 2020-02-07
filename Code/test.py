from multiprocessing import Pool, Process, Value, Array
from time import time
from tqdm import tqdm
from Other import *


def f(K):
    # T = 10
    z = 0.5
    for k in range(2, K+1):
        # for t in range(1, T+1):
        #     z += 0.5**k
        #     z += 1
        z *= k/(k+1.)
    return z


if __name__ == '__main__':
    T = 100
    K = 10
    a = f(K)
    b = 1./(K+1)
    # c = np.log(np.prod(np.exp(np.arange(1, K+1))))
    # d= K*((K+1)/2.0)
    # a = f(K)
    # b = T - T*0.5**K
    # d = range(510)
    # p = Pool(4)
    # tic = time()
    # a = list(tqdm(p.imap(f, d), total=10))
    # b = sum(a)
    # p.close()
    # toc = time()
    # print(toc-tic)
    # all_data = MyCsv.read_csv("results_all", "dict")
    # all_concepts = load_json("jsons/concepts+configs+results")
    # ga_concepts = load_json("jsons/concepts2ga")
    # ga_data = {}
    # for i in ga_concepts:
    #     if i in all_concepts:
    #         ga_data[i] = all_concepts[i]
    # mu = []
    # z= []
    # for conc in all_concepts:
    #     for conf in all_concepts[conc]:
    #         if conf[conf.keys()[0]]["mu"] != None and conf[conf.keys()[0]]["z"] != "70":
    #             mu.append(1-float(conf[conf.keys()[0]]["mu"]))
    #             z.append(float(conf[conf.keys()[0]]["z"]))
    # mu = np.asarray(mu)
    # mu_std = mu.std()
    # mu_mean = mu.mean()
    # z = np.asarray(z)
    # z_std = z.std()
    # z_mean = z.mean()
    # z.max()
