from multiprocessing import Pool, Process, Value, Array
from time import time
from tqdm import tqdm
from Other import *


def f(x):
    y=5
    z = 1
    q = 0
    for i in range(1,x+1):
        z *= i
        q += y
    return z, q


if __name__ == '__main__':
    d = range(1, 50)
    p = Pool(4)
    tic = time()
    a = list(tqdm(p.imap(f, d), total=len(d)))
    p.close()
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
